/**
 * @file main.cpp
 * @brief ESP32-S3 firmware main entry point.
 *
 * Target: LilyGO T-ETH-Lite-S3 (ESP32-S3-WROOM-1 + W5500 Ethernet)
 *
 * Two FreeRTOS tasks:
 *   - commTask  (Core 0): Ethernet/UDP, AOG protocol, HW status
 *   - controlTask (Core 1): 200 Hz control loop, PID, safety, actuators
 *
 * NOTE: Hardware init is done in setup():
 *       - normal mode: hal_esp32_init_all()
 *       - IMU bring-up mode: hal_esp32_init_imu_bringup()
 *       Tasks do NOT re-initialize anything.
 */

#include <Arduino.h>
#include <FreeRTOS.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>

#include "hal/hal.h"
#include "hal_esp32/hal_impl.h"
#include "logic/control.h"
#include "logic/dependency_policy.h"
#include "logic/features.h"
#include "logic/global_state.h"
#include "logic/hw_status.h"
#include "logic/imu.h"
#include "logic/modules.h"
#include "logic/net.h"
#include "logic/sd_ota.h"
#include "logic/sd_logger.h"

#include "logic/log_config.h"
#undef LOG_LOCAL_LEVEL          // Arduino.h already defined it via esp_log.h
#define LOG_LOCAL_LEVEL LOG_LEVEL_MAIN
#include "esp_log.h"
#include "logic/log_ext.h"

// ===================================================================
// Task handles
// ===================================================================
static TaskHandle_t s_control_task_handle = nullptr;
static TaskHandle_t s_comm_task_handle = nullptr;
static bool s_imu_bringup_active = false;

// ===================================================================
// Runtime/Debug logging knobs
// ===================================================================
static constexpr bool MAIN_VERBOSE_TASK_DBG = false;  // true => print loop Hz heartbeats
static constexpr uint32_t MAIN_HW_ERR_REMINDER_MS = 30000;

static inline bool shouldLogPeriodic(uint32_t now_ms, uint32_t* last_ms, uint32_t interval_ms) {
    if (now_ms - *last_ms < interval_ms) return false;
    *last_ms = now_ms;
    return true;
}

// ===================================================================
// Control Task – runs at 200 Hz on Core 1
// ===================================================================
static void controlTaskFunc(void* param) {
    (void)param;
    hal_log("Control: task started on core %d", xPortGetCoreID());

    // Wait for network + sensors to stabilise
    vTaskDelay(pdMS_TO_TICKS(500));

    const TickType_t interval = pdMS_TO_TICKS(5);  // 200 Hz = 5 ms
    TickType_t next_wake = xTaskGetTickCount();
    uint32_t ctrl_dbg_count = 0;
    uint32_t ctrl_freq_start = hal_millis();
    uint32_t log_divider = 0;
    uint32_t last_spi_tm_ms = 0;

    for (;;) {
        // ----------------------------- Input / Processing -----------------------------
        controlStep();

        // ------------------------------- Output phase --------------------------------
        // Keep potentially blocking logger writes out of every 200 Hz iteration.
        // Log only every 20 cycles (= 10 Hz).
        log_divider++;
        if (log_divider >= 20) {
            log_divider = 0;
            sdLoggerRecord();
        }

        if (MAIN_VERBOSE_TASK_DBG) {
            // Heartbeat DBG every 1s (= every 200 iterations)
            ctrl_dbg_count++;
            if (ctrl_dbg_count % 200 == 0) {
                uint32_t freq_now = hal_millis();
                float hz = (ctrl_dbg_count * 1000.0f) / (float)(freq_now - ctrl_freq_start);
                ctrl_freq_start = freq_now;
                ctrl_dbg_count = 0;
                Serial.printf("[DBG-CTRL] %.1f Hz\n", hz);
            }
        }

        const uint32_t now_ms = hal_millis();
        if (now_ms - last_spi_tm_ms >= 5000) {
            last_spi_tm_ms = now_ms;
            HalSpiTelemetry tm = {};
            hal_sensor_spi_get_telemetry(&tm);
            hal_log("SPI: util=%.1f%% bus_tx=%lu busy=%luus imu_tx=%lu was_tx=%lu miss(imu=%lu was=%lu)",
                    tm.bus_utilization_pct,
                    (unsigned long)tm.bus_transactions,
                    (unsigned long)tm.bus_busy_us,
                    (unsigned long)tm.imu_transactions,
                    (unsigned long)tm.was_transactions,
                    (unsigned long)tm.imu_deadline_miss,
                    (unsigned long)tm.was_deadline_miss);
        }

        // Maintain fixed 200 Hz timing with minimal jitter.
        vTaskDelayUntil(&next_wake, interval);
    }
}

// ===================================================================
// Communication Task – runs on Core 0
// ===================================================================
static void commTaskFunc(void* param) {
    (void)param;
    hal_log("Comm: task started on core %d", xPortGetCoreID());

    // Wait for network to initialise (done in setup, but give time to settle)
    vTaskDelay(pdMS_TO_TICKS(2000));

    Serial.println("[DBG-COMM] wait done, entering poll loop");

    const TickType_t poll_interval = pdMS_TO_TICKS(10);  // 100 Hz polling
    TickType_t next_wake = xTaskGetTickCount();

    // Hardware status update runs at ~1 Hz
    static uint32_t s_last_hw_status_ms = 0;
    static const uint32_t HW_STATUS_INTERVAL_MS = 1000;
    uint32_t comm_dbg_count = 0;
    uint32_t comm_freq_start = hal_millis();
    uint32_t last_hw_err_log_ms = 0;
    uint8_t last_hw_err_count = 0xFF;

    for (;;) {
        // ---------------------------------- Input -----------------------------------
        netPollReceive();

        // -------------------------------- Processing --------------------------------
        modulesUpdateStatus();

        // ---------------------------------- Output ----------------------------------
        netSendAogFrames();

        if (MAIN_VERBOSE_TASK_DBG) {
            // Heartbeat DBG every 5s (= every 500 iterations)
            comm_dbg_count++;
            if (comm_dbg_count % 500 == 0) {
                uint32_t freq_now = hal_millis();
                float hz = (comm_dbg_count * 1000.0f) / (float)(freq_now - comm_freq_start);
                comm_freq_start = freq_now;
                comm_dbg_count = 0;
                Serial.printf("[DBG-COMM] %.1f Hz\n", hz);
            }
        }

        // Hardware status monitoring (~1 Hz)
        uint32_t now = hal_millis();
        if (now - s_last_hw_status_ms >= HW_STATUS_INTERVAL_MS) {
            s_last_hw_status_ms = now;

            bool safety_ok = true;
            bool steer_quality_ok = false;
            uint32_t steer_ts_ms = 0;
            bool imu_quality_ok = false;
            uint32_t imu_ts_ms = 0;
            {
                StateLock lock;
                safety_ok = g_nav.safety_ok;
                steer_quality_ok = g_nav.steer_angle_quality_ok;
                steer_ts_ms = g_nav.steer_angle_timestamp_ms;
                imu_quality_ok = g_nav.imu_quality_ok;
                imu_ts_ms = g_nav.imu_timestamp_ms;
            }

            const bool steer_angle_valid =
                dep_policy::isSteerAngleInputValid(now, steer_ts_ms, steer_quality_ok);
            const ModuleHwStatus* hw = modulesGetHwStatus();
            const bool imu_hw_detected = hw ? hw->imu_detected : false;
            const bool imu_valid =
                imu_hw_detected && dep_policy::isImuInputValid(now, imu_ts_ms, imu_quality_ok);

            // Hardware status monitoring via hw_status subsystem
            uint8_t err_count = hwStatusUpdate(
                hal_net_is_connected(),     // Ethernet connected
                safety_ok,                  // Safety circuit OK
                steer_angle_valid,          // steer angle freshness + plausibility
                imu_valid                   // IMU freshness + plausibility
            );

            // Log only on count changes, plus occasional reminders.
            bool changed = (err_count != last_hw_err_count);
            bool reminder = (err_count > 0) &&
                            shouldLogPeriodic(now, &last_hw_err_log_ms, MAIN_HW_ERR_REMINDER_MS);
            if (changed || reminder) {
                last_hw_err_count = err_count;
                last_hw_err_log_ms = now;
            }

            if (changed) {
                hal_log("COMM: HW error count changed -> %u", (unsigned)err_count);
            } else if (reminder) {
                hal_log("COMM: %u HW error(s) active", (unsigned)err_count);
            }
        }

        vTaskDelayUntil(&next_wake, poll_interval);
    }
}

// ===================================================================
// Arduino setup()
// ===================================================================
void setup() {
    s_imu_bringup_active = imuBringupModeEnabled();
    if (s_imu_bringup_active) {
        // Explicit bring-up path: no actuator or network dependency.
        hal_esp32_init_imu_bringup();
    } else {
        // Normal operation path.
        hal_esp32_init_all();
    }

    // -----------------------------------------------------------------
    // Firmware Version & Build Info (always printed)
    // -----------------------------------------------------------------
    {
        SdOtaVersion ver = sdOtaGetCurrentVersion();
        const esp_partition_t* part = esp_ota_get_running_partition();
        hal_log("========================================");
        hal_log("  AgSteer Firmware v%u.%u.%u", ver.major, ver.minor, ver.patch);
        hal_log("  Build: %s %s", __DATE__, __TIME__);
        hal_log("  Partition: %s (0x%06X)", part ? part->label : "?",
                part ? (unsigned)part->address : 0);
        hal_log("  Flash: %d KB free", (int)(ESP.getFreeSketchSpace() / 1024));
        hal_log("========================================");
    }

    if (s_imu_bringup_active) {
        hal_log("Main: IMU bring-up mode active (FEAT_IMU_BRINGUP).");
        imuBringupInit();
        return;
    }

    // -----------------------------------------------------------------
    // SD-Card OTA Firmware Update
    // -----------------------------------------------------------------
    // Check if a firmware update file is present on the SD card.
    // This must happen BEFORE creating FreeRTOS tasks because the
    // update temporarily re-uses the sensor SPI bus (SPI2_HOST).
    //
    // If /firmware.bin (or /update.bin) is found on the SD card and
    // the version is newer than the current firmware, the update is
    // performed automatically.  On success the ESP32 reboots into the
    // new firmware.  On any error the old firmware continues to run.
    //
    // To skip the update check, simply remove the firmware file from
    // the SD card before booting.
    // -----------------------------------------------------------------
    {
        if (isFirmwareUpdateAvailableOnSD()) {
            hal_log("Main: firmware update detected on SD card – starting update");
            updateFirmwareFromSD();
            // If we reach here the update failed – continue with old firmware
            hal_log("Main: OTA update FAILED, continuing with current firmware");
        }
    }

    // Initialise module system – detect hardware for all modules
    modulesInit();

    // Initialise control system (PID controller with default gains).
    // NOTE: HAL-level init (imu, steer angle, actuator) was already done
    //       in hal_esp32_init_all().  controlInit() only sets up the PID.
    if (feat::control()) {
        controlInit();
    } else {
        hal_log("Main: control loop feature disabled");
    }

    // -----------------------------------------------------------------
    // Steering Angle Calibration
    // -----------------------------------------------------------------
    // On first boot (no calibration in NVS), the interactive calibration
    // runs automatically. On subsequent boots, the stored calibration
    // is loaded from NVS. Press 'c' within 3s to force recalibration.
    //
    // The user is prompted via Serial to move steering to left/right
    // stops. Values are stored in NVS and survive reboots.
    // -----------------------------------------------------------------
    if (feat::sensor()) {
        bool need_cal = !hal_steer_angle_is_calibrated();

        if (!need_cal) {
            // Already calibrated — give user a chance to recalibrate
            uint32_t cal_wait_start = millis();
            Serial.println();
            Serial.println("Druecke 'c' + ENTER fuer Neukalibrierung (3s)...");
            Serial.flush();
            while (millis() - cal_wait_start < 3000) {
                if (Serial.available()) {
                    int c = Serial.read();
                    if (c == 'c' || c == 'C') {
                        need_cal = true;
                        while (Serial.available()) Serial.read();
                        break;
                    }
                }
                delay(10);
            }
        }

        if (need_cal) {
            hal_log("Main: %s calibration",
                    hal_steer_angle_is_calibrated() ? "forced re" : "initial");
            hal_steer_angle_calibrate();
        } else {
            hal_log("Main: calibration OK (loaded from NVS)");
        }
    } else {
        hal_log("Main: steer angle calibration skipped (sensor feature disabled)");
    }

    // Initialise hardware status monitoring
    hwStatusInit();

    // Network is already initialised by hal_esp32_init_all()
    // (hal_net_init was called there, ETH link should be established)

    // -----------------------------------------------------------------
    // SD-Card Data Logger
    // -----------------------------------------------------------------
    // The logger is controlled by a hardware switch on GPIO 47.
    // When the switch is ON (closed to GND), navigation/steering data
    // is recorded to CSV files on the SD card at 10 Hz.
    // The logger runs as a low-priority FreeRTOS task that periodically
    // drains a ring buffer to the SD card (every 2 seconds).
    // -----------------------------------------------------------------
    if (feat::control()) {
        sdLoggerInit();
    }

    // Report initial hardware errors
    // Always call – reportError() will use UDP if network is up,
    // Serial-only if network is down.
    hal_delay_ms(100);
    modulesSendStartupErrors();

    // Create control task on Core 1
    if (feat::control()) {
        xTaskCreatePinnedToCore(
            controlTaskFunc,
            "ctrl",
            4096,
            nullptr,
            configMAX_PRIORITIES - 2,  // high priority
            &s_control_task_handle,
            1   // Core 1
        );
    } else {
        hal_log("Main: control task not started (feature disabled)");
    }

    // Create communication task on Core 0
    xTaskCreatePinnedToCore(
        commTaskFunc,
        "comm",
        4096,
        nullptr,
        configMAX_PRIORITIES - 3,  // slightly lower priority
        &s_comm_task_handle,
        0   // Core 0
    );

    hal_log("Main: tasks created, entering main loop");
}

// ===================================================================
// Arduino loop() – not used for real work (tasks handle everything)
// ===================================================================
static uint32_t s_loop_dbg_count = 0;

void loop() {
    if (s_imu_bringup_active) {
        imuBringupTick();
        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }

    // Feed watchdog to prevent trigger from this task
    esp_task_wdt_reset();

    // Periodic status output every 5 seconds
    static uint32_t s_last_status = 0;
    uint32_t now = hal_millis();
    if (now - s_last_status >= 5000) {
        s_last_status = now;
        StateLock lock;
        hal_log("STAT: hd=%.1f st=%.1f raw=%d safety=%s work=%s steer=%s spd=%.1f wdog=%s pid=%d tgt=%.1f net=%s cfg=%s",
                g_nav.heading_deg,
                g_nav.steer_angle_deg,
                (int)g_nav.steer_angle_raw,
                g_nav.safety_ok ? "OK" : "KICK",
                g_nav.work_switch ? "ON" : "OFF",
                g_nav.steer_switch ? "ON" : "OFF",
                g_nav.gps_speed_kmh,
                g_nav.watchdog_triggered ? "TRIG" : "OK",
                (int)g_nav.pid_output,
                desiredSteerAngleDeg,
                hal_net_is_connected() ? "UP" : "DOWN",
                g_nav.settings_received ? "Y" : "N");
    }

    if (MAIN_VERBOSE_TASK_DBG) {
        // Heartbeat DBG every 1s (= every ~10 iterations at 100ms delay)
        s_loop_dbg_count++;
        if (s_loop_dbg_count <= 5 || s_loop_dbg_count % 10 == 0) {
            static uint32_t s_loop_freq_start_ms = 0;
            static uint32_t s_loop_freq_samples = 0;
            if (s_loop_freq_start_ms == 0) s_loop_freq_start_ms = hal_millis();
            s_loop_freq_samples++;
            if (s_loop_freq_samples >= 10) {
                const uint32_t freq_now_ms = hal_millis();
                const float hz =
                    (s_loop_freq_samples * 1000.0f) / (float)(freq_now_ms - s_loop_freq_start_ms);
                s_loop_freq_start_ms = freq_now_ms;
                s_loop_freq_samples = 0;
                Serial.printf("[DBG-LOOP] %.1f Hz\n", hz);
            }
        }
    }

#if LOG_SERIAL_CMD
    // Runtime logging control:
    //   log <tag> <none|error|warn|info|debug|verbose>
    //   log all <level>
    //   log status
    //   filter <file[:line]> | filter off
    static char cmd_buf[96];
    static size_t cmd_len = 0;
    while (Serial.available()) {
        const int ch = Serial.read();
        if (ch == '\r' || ch == '\n') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                logProcessSerialCmd(cmd_buf);
                cmd_len = 0;
            }
        } else if (cmd_len + 1 < sizeof(cmd_buf)) {
            cmd_buf[cmd_len++] = static_cast<char>(ch);
        }
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(100));
}
