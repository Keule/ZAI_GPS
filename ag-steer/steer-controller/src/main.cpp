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
 * NOTE: All hardware init is done in hal_esp32_init_all() during setup().
 *       The tasks do NOT re-initialize anything.
 */

#include <Arduino.h>
#include <FreeRTOS.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>

#include "hal/hal.h"
#include "hal_esp32/hal_impl.h"
#include "logic/global_state.h"
#include "logic/control.h"
#include "logic/net.h"
#include "logic/modules.h"
#include "logic/hw_status.h"
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

// ===================================================================
// Control Task – runs at 200 Hz on Core 1
// ===================================================================
static void controlTaskFunc(void* param) {
    (void)param;
    LOGI("MAIN", "Control: task started on core %d", xPortGetCoreID());

    // Wait for network + sensors to stabilise
    vTaskDelay(pdMS_TO_TICKS(500));

    const TickType_t interval = pdMS_TO_TICKS(5);  // 200 Hz = 5 ms
    uint32_t ctrl_dbg_count = 0;

    for (;;) {
        uint32_t start = hal_millis();

        // Run one control step
        controlStep();

        // Buffer one log record (subsampled to 10 Hz internally)
        sdLoggerRecord();

        // Heartbeat DBG every 1s (= every 200 iterations)
        ctrl_dbg_count++;
        if (ctrl_dbg_count % 200 == 0) {
            Serial.printf("[DBG-CTRL] alive tick=%lu\n", (unsigned long)ctrl_dbg_count);
            Serial.flush();
        }

        // Maintain 200 Hz timing
        uint32_t elapsed = hal_millis() - start;
        if (elapsed < 5) {
            vTaskDelay(interval - pdMS_TO_TICKS(elapsed));
        }
    }
}

// ===================================================================
// Communication Task – runs on Core 0
// ===================================================================
static void commTaskFunc(void* param) {
    (void)param;
    LOGI("MAIN", "Comm: task started on core %d", xPortGetCoreID());

    // Wait for network to initialise (done in setup, but give time to settle)
    vTaskDelay(pdMS_TO_TICKS(2000));

    Serial.println("[DBG-COMM] wait done, entering poll loop");
    Serial.flush();

    const TickType_t poll_interval = pdMS_TO_TICKS(10);  // 100 Hz polling

    // Hardware status update runs at ~1 Hz
    static uint32_t s_last_hw_status_ms = 0;
    static const uint32_t HW_STATUS_INTERVAL_MS = 1000;
    uint32_t comm_dbg_count = 0;

    for (;;) {
        // Poll network for incoming frames
        netPollReceive();

        // Send periodic AOG frames (~10 Hz)
        netSendAogFrames();

        // Heartbeat DBG every 5s (= every 500 iterations)
        comm_dbg_count++;
        if (comm_dbg_count % 500 == 0) {
            Serial.printf("[DBG-COMM] alive tick=%lu\n", (unsigned long)comm_dbg_count);
            Serial.flush();
        }

        // Hardware status monitoring (~1 Hz)
        uint32_t now = hal_millis();
        if (now - s_last_hw_status_ms >= HW_STATUS_INTERVAL_MS) {
            s_last_hw_status_ms = now;

            // Update dynamic module status (safety changes)
            modulesUpdateStatus();

            // Hardware status monitoring via hw_status subsystem
            uint8_t err_count = hwStatusUpdate(
                hal_net_is_connected(),     // Ethernet connected
                g_nav.safety_ok,            // Safety circuit OK
                true,                       // Steer angle valid (TODO: actual check)
                modulesHwOk(AOG_MOD_STEER)  // Module-level: all steer subsystems OK
            );

            // Log error count periodically
            if (err_count > 0) {
                LOGW("MAIN", "COMM: %u HW error(s) active", (unsigned)err_count);
            }
        }

        // Yield to IDLE task – prevents watchdog trigger
        vTaskDelay(poll_interval);
    }
}

// ===================================================================
// Arduino setup()
// ===================================================================
void setup() {
    // Initialise ALL hardware (mutex, safety, SPI, sensors, W5500)
    hal_esp32_init_all();

    // -----------------------------------------------------------------
    // Firmware Version & Build Info (always printed)
    // -----------------------------------------------------------------
    {
        SdOtaVersion ver = sdOtaGetCurrentVersion();
        const esp_partition_t* part = esp_ota_get_running_partition();
        LOGI("MAIN", "========================================");
        LOGI("MAIN", "  AgSteer Firmware v%u.%u.%u", ver.major, ver.minor, ver.patch);
        LOGI("MAIN", "  Build: %s %s", __DATE__, __TIME__);
        LOGI("MAIN", "  Partition: %s (0x%06X)", part ? part->label : "?",
                part ? (unsigned)part->address : 0);
        LOGI("MAIN", "  Flash: %d KB free", (int)(ESP.getFreeSketchSpace() / 1024));
        LOGI("MAIN", "========================================");
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
            LOGI("MAIN", "firmware update detected on SD card – starting update");
            updateFirmwareFromSD();
            // If we reach here the update failed – continue with old firmware
            LOGE("MAIN", "OTA update FAILED, continuing with current firmware");
        }
    }

    // Initialise module system – detect hardware for all modules
    modulesInit();

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
    {
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
            LOGI("MAIN", "%s calibration",
                    hal_steer_angle_is_calibrated() ? "forced re" : "initial");
            hal_steer_angle_calibrate();
        } else {
            LOGI("MAIN", "calibration OK (loaded from NVS)");
        }
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
    sdLoggerInit();

    // Report initial hardware errors
    // Always call – reportError() will use UDP if network is up,
    // Serial-only if network is down.
    hal_delay_ms(100);
    modulesSendStartupErrors();

    // Create control task on Core 1
    xTaskCreatePinnedToCore(
        controlTaskFunc,
        "ctrl",
        4096,
        nullptr,
        configMAX_PRIORITIES - 2,  // high priority
        &s_control_task_handle,
        1   // Core 1
    );

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

    LOGI("MAIN", "tasks created, entering main loop");
}

// ===================================================================
// Arduino loop() – not used for real work (tasks handle everything)
// ===================================================================
static uint32_t s_loop_dbg_count = 0;

void loop() {
    // Feed watchdog to prevent trigger from this task
    esp_task_wdt_reset();

    // Periodic status output every 5 seconds
    static uint32_t s_last_status = 0;
    uint32_t now = hal_millis();
    if (now - s_last_status >= 5000) {
        s_last_status = now;
        StateLock lock;
        LOGI("MAIN", "STAT: hd=%.1f st=%.1f raw=%d safety=%s work=%s steer=%s spd=%.1f wdog=%s pid=%d tgt=%.1f net=%s cfg=%s",
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

    // Heartbeat DBG every 1s (= every ~10 iterations at 100ms delay)
    s_loop_dbg_count++;
    if (s_loop_dbg_count <= 5 || s_loop_dbg_count % 10 == 0) {
        Serial.printf("[DBG-LOOP] alive tick=%lu\n", (unsigned long)s_loop_dbg_count);
        Serial.flush();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}
