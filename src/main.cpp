/**
 * @file main.cpp
 * @brief ESP32-S3 firmware main entry point.
 *
 * Target: LilyGO T-ETH-Lite-S3 (ESP32-S3-WROOM-1 + W5500 Ethernet)
 *
 * Three FreeRTOS tasks:
 *   - commTask   (Core 0): Ethernet/UDP, AOG protocol, HW status
 *   - maintTask  (Core 0): SD flush, NTRIP connect, ETH monitor [TASK-029]
 *   - controlTask (Core 1): 200 Hz control loop, PID, safety, actuators
 *
 * NOTE: Hardware init is done in setup():
 *       - normal mode: hal_esp32_init_all()
 *       - IMU bring-up mode: hal_esp32_init_imu_bringup()
 *       - GNSS buildup mode: hal_esp32_init_gnss_buildup()
 *       Tasks do NOT re-initialize anything.
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include <cstdio>

#include "fw_config.h"
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
#include "logic/ntrip.h"
#include "logic/runtime_config.h"
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
static bool s_gnss_buildup_active = false;

// ===================================================================
// Runtime/Debug logging knobs
// ===================================================================
static constexpr bool MAIN_VERBOSE_TASK_DBG = false;  // true => print loop Hz heartbeats
static constexpr uint32_t MAIN_HW_ERR_REMINDER_MS = 30000;
static constexpr uint32_t MAIN_GNSS_BUILDUP_INIT_TIMEOUT_MS = 15000;
static constexpr uint32_t MAIN_GNSS_BUILDUP_STATUS_INTERVAL_MS = 2000;
static uint32_t s_gnss_buildup_start_ms = 0;
static bool s_gnss_buildup_fallback_latched = false;

static inline bool shouldLogPeriodic(uint32_t now_ms, uint32_t* last_ms, uint32_t interval_ms) {
    if (now_ms - *last_ms < interval_ms) return false;
    *last_ms = now_ms;
    return true;
}

// ===================================================================
// GNSS UART mirror (diagnostic bring-up path)
// ===================================================================
#if FEAT_CAP_GNSS_UART_MIRROR
static constexpr bool MAIN_GNSS_UART_MIRROR_ENABLED = true;
#else
static constexpr bool MAIN_GNSS_UART_MIRROR_ENABLED = false;
#endif

static constexpr size_t MAIN_GNSS_MIRROR_LINE_CAP = 96;
static constexpr size_t MAIN_GNSS_MIRROR_BINARY_CHUNK = 24;
static constexpr int MAIN_GNSS_MIRROR_MAX_BYTES_PER_PORT_POLL = 64;
static constexpr int MAIN_GNSS_MIRROR_MAX_FLUSH_PER_PORT_POLL = 2;

struct GnssMirrorPortState {
    HardwareSerial* uart = nullptr;
    const char* prefix = "";
    uint8_t line_buf[MAIN_GNSS_MIRROR_LINE_CAP] = {0};
    size_t line_len = 0;
    bool binary_mode = false;
    uint32_t dropped_bytes = 0;
    uint32_t last_drop_log_ms = 0;
};

static GnssMirrorPortState s_gnss_mirror_ports[] = {
    {&Serial1, "[UM980-A]"},
    {&Serial2, "[UM980-B]"},
};

static void gnssMirrorLogDropped(GnssMirrorPortState* port, uint32_t now_ms) {
    if (port->dropped_bytes == 0) return;
    if (now_ms - port->last_drop_log_ms < 2000) return;
    port->last_drop_log_ms = now_ms;
    hal_log("GNSS-MIRROR: %s dropped=%lu byte(s) due to local parser limits",
            port->prefix,
            (unsigned long)port->dropped_bytes);
    port->dropped_bytes = 0;
}

static bool gnssMirrorLooksLikeNmea(const uint8_t* data, size_t len) {
    return len > 0 && (data[0] == '$' || data[0] == '!');
}

static void gnssMirrorFlush(GnssMirrorPortState* port) {
    if (port->line_len == 0) return;

    if (!port->binary_mode) {
        char text[MAIN_GNSS_MIRROR_LINE_CAP + 1];
        size_t out = 0;
        for (size_t i = 0; i < port->line_len && out < MAIN_GNSS_MIRROR_LINE_CAP; ++i) {
            const uint8_t b = port->line_buf[i];
            if (b == '\r' || b == '\n') continue;
            if (b >= 32 && b <= 126) {
                text[out++] = static_cast<char>(b);
            }
        }
        text[out] = '\0';

        if (out > 0) {
            if (gnssMirrorLooksLikeNmea(port->line_buf, port->line_len)) {
                hal_log("GNSS-MIRROR: %s %s", port->prefix, text);
            } else {
                hal_log("GNSS-MIRROR: %s [RAW] %s", port->prefix, text);
            }
        }
    } else {
        char hex_line[(MAIN_GNSS_MIRROR_BINARY_CHUNK * 3) + 1];
        size_t pos = 0;
        for (size_t i = 0; i < port->line_len && i < MAIN_GNSS_MIRROR_BINARY_CHUNK; ++i) {
            const int n = std::snprintf(&hex_line[pos], sizeof(hex_line) - pos, "%02X",
                                        (unsigned)port->line_buf[i]);
            if (n <= 0) break;
            pos += static_cast<size_t>(n);
            if ((i + 1) < port->line_len && pos + 1 < sizeof(hex_line)) {
                hex_line[pos++] = ' ';
                hex_line[pos] = '\0';
            }
        }
        if (pos > 0) {
            hal_log("GNSS-MIRROR: %s [HEX] %s", port->prefix, hex_line);
        }
    }

    port->line_len = 0;
    port->binary_mode = false;
}

static void gnssMirrorPollPort(GnssMirrorPortState* port) {
    if (!port || !port->uart) return;

    int bytes_left = MAIN_GNSS_MIRROR_MAX_BYTES_PER_PORT_POLL;
    int flush_budget = MAIN_GNSS_MIRROR_MAX_FLUSH_PER_PORT_POLL;

    while (bytes_left-- > 0 && port->uart->available() > 0) {
        const int value = port->uart->read();
        if (value < 0) break;

        const uint8_t b = static_cast<uint8_t>(value);
        const bool is_newline = (b == '\n');
        const bool is_ascii = (b == '\r' || b == '\n' || b == '\t' || (b >= 32 && b <= 126));

        if (!is_ascii) {
            port->binary_mode = true;
        }

        if (!is_newline) {
            if (port->line_len < MAIN_GNSS_MIRROR_LINE_CAP) {
                port->line_buf[port->line_len++] = b;
            } else {
                port->dropped_bytes++;
                if (flush_budget > 0) {
                    gnssMirrorFlush(port);
                    flush_budget--;
                }
            }
        }

        const bool force_binary_flush = port->binary_mode &&
                                        port->line_len >= MAIN_GNSS_MIRROR_BINARY_CHUNK;
        if (is_newline || force_binary_flush) {
            if (flush_budget > 0) {
                gnssMirrorFlush(port);
                flush_budget--;
            } else {
                break;
            }
        }
    }

    gnssMirrorLogDropped(port, hal_millis());
}

static void gnssMirrorPoll(void) {
    if (!MAIN_GNSS_UART_MIRROR_ENABLED) return;
    for (GnssMirrorPortState& port : s_gnss_mirror_ports) {
        gnssMirrorPollPort(&port);
    }
}

static void gnssMirrorInit(void) {
    if (!MAIN_GNSS_UART_MIRROR_ENABLED) {
        hal_log("GNSS-MIRROR: disabled (compile with FEAT_GNSS_UART_MIRROR to enable)");
        return;
    }

    Serial1.begin(GNSS_MIRROR_BAUD, SERIAL_8N1, GNSS_MIRROR_UART1_RX_PIN, GNSS_MIRROR_UART1_TX_PIN);
    Serial2.begin(GNSS_MIRROR_BAUD, SERIAL_8N1, GNSS_MIRROR_UART2_RX_PIN, GNSS_MIRROR_UART2_TX_PIN);

    hal_log("GNSS-MIRROR: enabled baud=%lu UART1(rx=%d tx=%d)->%s UART2(rx=%d tx=%d)->%s",
            (unsigned long)GNSS_MIRROR_BAUD,
            (int)GNSS_MIRROR_UART1_RX_PIN,
            (int)GNSS_MIRROR_UART1_TX_PIN,
            s_gnss_mirror_ports[0].prefix,
            (int)GNSS_MIRROR_UART2_RX_PIN,
            (int)GNSS_MIRROR_UART2_TX_PIN,
            s_gnss_mirror_ports[1].prefix);
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
#if FEAT_CAP_SENSOR_SPI2
    uint32_t last_spi_tm_ms = 0;
#endif

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
#if FEAT_CAP_SENSOR_SPI2
        if (LOG_SPI_TIMING_INTERVAL_MS > 0 && now_ms - last_spi_tm_ms >= LOG_SPI_TIMING_INTERVAL_MS) {
            last_spi_tm_ms = now_ms;
            HalSpiTelemetry tm = {};
            hal_sensor_spi_get_telemetry(&tm);
            hal_log("SPI: util=%.1f%% bus_tx=%lu busy=%luus was_us=%lu/%lu imu_us=%lu/%lu act_us=%lu/%lu tx(was=%lu imu=%lu act=%lu) sw=%lu(wi=%lu iw=%lu oth=%lu) sw_gap_us(wi=%lu/%lu iw=%lu/%lu) sens_sw(wi=%lu iw=%lu) sens_gap_us(wi=%lu/%lu iw=%lu/%lu) miss(imu=%lu was=%lu)",
                    tm.bus_utilization_pct,
                    (unsigned long)tm.bus_transactions,
                    (unsigned long)tm.bus_busy_us,
                    (unsigned long)tm.was_last_us,
                    (unsigned long)tm.was_max_us,
                    (unsigned long)tm.imu_last_us,
                    (unsigned long)tm.imu_max_us,
                    (unsigned long)tm.actuator_last_us,
                    (unsigned long)tm.actuator_max_us,
                    (unsigned long)tm.was_transactions,
                    (unsigned long)tm.imu_transactions,
                    (unsigned long)tm.actuator_transactions,
                    (unsigned long)tm.client_switches,
                    (unsigned long)tm.was_to_imu_switches,
                    (unsigned long)tm.imu_to_was_switches,
                    (unsigned long)tm.other_switches,
                    (unsigned long)tm.was_to_imu_gap_last_us,
                    (unsigned long)tm.was_to_imu_gap_max_us,
                    (unsigned long)tm.imu_to_was_gap_last_us,
                    (unsigned long)tm.imu_to_was_gap_max_us,
                    (unsigned long)tm.sensor_was_to_imu_switches,
                    (unsigned long)tm.sensor_imu_to_was_switches,
                    (unsigned long)tm.sensor_was_to_imu_gap_last_us,
                    (unsigned long)tm.sensor_was_to_imu_gap_max_us,
                    (unsigned long)tm.sensor_imu_to_was_gap_last_us,
                    (unsigned long)tm.sensor_imu_to_was_gap_max_us,
                    (unsigned long)tm.imu_deadline_miss,
                    (unsigned long)tm.was_deadline_miss);
        }
#endif

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
#if FEAT_ENABLED(FEAT_NTRIP)
        ntripReadRtcm();
#endif

        // -------------------------------- Processing --------------------------------
        if (!s_gnss_buildup_active) {
            modulesUpdateStatus();
        }
#if FEAT_ENABLED(FEAT_NTRIP)
        // TASK-029: In normal mode, ntripTick() runs in maintTask (blocking
        // TCP connect is OK there). In GNSS buildup mode, maintTask is not
        // created, so we run ntripTick() here in the commTask.
        if (s_gnss_buildup_active) {
            ntripTick();
        }
#endif

        // ---------------------------------- Output ----------------------------------
        netSendAogFrames();
        gnssMirrorPoll();
#if FEAT_ENABLED(FEAT_NTRIP)
        ntripForwardRtcm();
#endif

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
        if (!s_gnss_buildup_active && now - s_last_hw_status_ms >= HW_STATUS_INTERVAL_MS) {
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
            const bool imu_data_valid =
                imu_hw_detected && dep_policy::isImuInputValid(now, imu_ts_ms, imu_quality_ok);

            // Hardware status monitoring via hw_status subsystem
            uint8_t err_count = hwStatusUpdate(
                hal_net_is_connected(),     // Ethernet connected
                safety_ok,                  // Safety circuit OK
                steer_angle_valid,          // steer angle freshness + plausibility
                imu_hw_detected,            // IMU hardware presence; data quality remains in g_nav
                moduleIsActive(MOD_NTRIP)   // NTRIP module active — TASK-030
            );

            (void)imu_data_valid;

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
#if defined(FEAT_IMU_BRINGUP) && defined(FEAT_GNSS_BUILDUP)
#error "FEAT_IMU_BRINGUP and FEAT_GNSS_BUILDUP are mutually exclusive."
#endif

    s_imu_bringup_active = imuBringupModeEnabled();
    s_gnss_buildup_active =
#if defined(FEAT_GNSS_BUILDUP)
        true;
#else
        false;
#endif

    if (s_imu_bringup_active) {
        // Explicit bring-up path: no actuator or network dependency.
        hal_esp32_init_imu_bringup();
    } else if (s_gnss_buildup_active) {
        // GNSS buildup path: communication + GNSS UART only.
        hal_esp32_init_gnss_buildup();
        s_gnss_buildup_start_ms = hal_millis();
        hal_log("Main: GNSS buildup mode active (FEAT_GNSS_BUILDUP).");
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
        gnssMirrorInit();
        return;
    }

    gnssMirrorInit();
    if (s_gnss_buildup_active) {
        // Reduced startup: no OTA, no module detection, no sensor/actuator stack.

        // Initialise soft config from compile-time defaults — TASK-028
        softConfigLoadDefaults(softConfigGet());
        softConfigLoadOverrides(softConfigGet());  // currently a no-op stub

#if FEAT_ENABLED(FEAT_NTRIP)
        // -----------------------------------------------------------------
        // NTRIP Client initialisation — TASK-025 / TASK-028
        // Configuration is loaded from RuntimeConfig (cfg:: defaults at
        // boot, overridable via Serial/SD/WebUI in future versions).
        // -----------------------------------------------------------------
        ntripInit();
        RuntimeConfig& rcfg = softConfigGet();
        ntripSetConfig(
            rcfg.ntrip_host,
            rcfg.ntrip_port,
            rcfg.ntrip_mountpoint,
            rcfg.ntrip_user,
            rcfg.ntrip_password
        );
        if (rcfg.ntrip_host[0] == '\0' || rcfg.ntrip_mountpoint[0] == '\0') {
            hal_log("Main: NTRIP not configured (host or mountpoint empty) — skipping");
        } else {
            hal_log("Main: NTRIP client configured (host=%s, port=%u, mp=%s)",
                    g_ntrip_config.host,
                    static_cast<unsigned>(g_ntrip_config.port),
                    g_ntrip_config.mountpoint);
        }
#endif

        xTaskCreatePinnedToCore(
            commTaskFunc,
            "comm",
            4096,
            nullptr,
            configMAX_PRIORITIES - 3,
            &s_comm_task_handle,
            0
        );
        hal_log("Main: GNSS buildup reduced init done (comm task only).");
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

    // -----------------------------------------------------------------
    // Initialise soft config from compile-time defaults — TASK-028
    // RuntimeConfig is the mutable RAM copy; cfg:: namespace holds
    // the compile-time defaults defined in include/soft_config.h.
    // -----------------------------------------------------------------
    softConfigLoadDefaults(softConfigGet());
    softConfigLoadOverrides(softConfigGet());  // currently a no-op stub

    // Initialise module system – detect hardware for all modules
    modulesInit();

    // -----------------------------------------------------------------
    // Activate default feature modules — TASK-027
    // The module system handles pin-claim arbitration and dependency
    // checking.  These calls must happen AFTER modulesInit() which
    // sets compiled/availability flags, and AFTER hal_esp32_init_all()
    // which already claimed pins for common init paths.
    //
    // Activation order matters: ACT depends on IMU+ADS, NTRIP depends
    // on ETH.  Activate dependencies first.
    // -----------------------------------------------------------------
    moduleActivate(MOD_IMU);     // IMU: no deps
    moduleActivate(MOD_ADS);     // ADS: no deps
    moduleActivate(MOD_ETH);     // ETH: no deps (pins already claimed by HAL init)
    moduleActivate(MOD_GNSS);    // GNSS: no deps
    moduleActivate(MOD_SAFETY);  // SAFETY: no deps
    moduleActivate(MOD_ACT);     // ACT: depends on IMU + ADS (must be after those)
#if FEAT_ENABLED(FEAT_NTRIP)
    moduleActivate(MOD_NTRIP);   // NTRIP: depends on ETH (must be after ETH)
#endif

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
    // Maintenance Task (TASK-029)
    // -----------------------------------------------------------------
    // The maintTask consolidates all blocking operations into one
    // low-priority task on Core 0:
    //   - SD card logging (PSRAM ring buffer → CSV, every 2 s)
    //   - NTRIP connect/reconnect (state machine, every 1 s)
    //   - ETH link monitoring (on change)
    //
    // Uses a PSRAM-backed ring buffer (~1 MB = ~53 min at 10 Hz)
    // so the control loop's sdLoggerRecord() call is ~1 µs with
    // no FSPI interaction.
    // -----------------------------------------------------------------
    if (feat::control()) {
        sdLoggerMaintInit();
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

    if (s_gnss_buildup_active) {
        const uint32_t now = hal_millis();
        static uint32_t s_last_gnss_buildup_log_ms = 0;
        if (now - s_last_gnss_buildup_log_ms >= MAIN_GNSS_BUILDUP_STATUS_INTERVAL_MS) {
            s_last_gnss_buildup_log_ms = now;

            uint8_t fix_type = 0;
            bool rtcm_active = false;
            uint32_t diff_age = 0;
            uint32_t status_age_ms = 0;
            {
                StateLock lock;
                fix_type = g_nav.um980_fix_type;
                rtcm_active = g_nav.um980_rtcm_active;
                diff_age = g_nav.gps_diff_age_x100_ms;
                status_age_ms = (g_nav.um980_status_timestamp_ms == 0)
                    ? 0
                    : (now - g_nav.um980_status_timestamp_ms);
            }

            hal_log("GNSS-BUILDUP: port_status net=%s rtcm_uart=%s drop=%lu | fix_status type=%u rtcm=%s diff_age_x100ms=%lu status_age_ms=%lu",
                    hal_net_is_connected() ? "UP" : "DOWN",
                    hal_gnss_rtcm_is_ready() ? "READY" : "DOWN",
                    static_cast<unsigned long>(hal_gnss_rtcm_drop_count()),
                    static_cast<unsigned>(fix_type),
                    rtcm_active ? "ON" : "OFF",
                    static_cast<unsigned long>(diff_age),
                    static_cast<unsigned long>(status_age_ms));
        }

        if (!s_gnss_buildup_fallback_latched &&
            now - s_gnss_buildup_start_ms >= MAIN_GNSS_BUILDUP_INIT_TIMEOUT_MS) {
            const bool ready = hal_net_is_connected() && hal_gnss_rtcm_is_ready();
            if (!ready) {
                s_gnss_buildup_fallback_latched = true;
                hal_log("GNSS-BUILDUP: init timeout after %lums -> fallback=degraded-comm (no abort, keep running for diagnostics)",
                        static_cast<unsigned long>(MAIN_GNSS_BUILDUP_INIT_TIMEOUT_MS));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }

    // Feed watchdog to prevent trigger from this task
    esp_task_wdt_reset();

    // Periodic serial telemetry every 5 seconds.
    // IMPORTANT: IMU serial telemetry must remain network-independent so
    // diagnostics stay visible even with Ethernet link down.
    static uint32_t s_last_status = 0;
    uint32_t now = hal_millis();
    if (now - s_last_status >= 5000) {
        s_last_status = now;
        float heading_deg = 0.0f;
        float steer_angle_deg = 0.0f;
        int steer_angle_raw = 0;
        bool safety_ok = false;
        bool work_switch = false;
        bool steer_switch = false;
        float gps_speed_kmh = 0.0f;
        bool watchdog_triggered = false;
        int pid_output = 0;
        bool settings_received = false;
        float roll_deg = 0.0f;
        float yaw_rate_dps = 0.0f;
        bool imu_quality_ok = false;
        uint32_t imu_timestamp_ms = 0;

        {
            StateLock lock;
            heading_deg = g_nav.heading_deg;
            steer_angle_deg = g_nav.steer_angle_deg;
            steer_angle_raw = (int)g_nav.steer_angle_raw;
            safety_ok = g_nav.safety_ok;
            work_switch = g_nav.work_switch;
            steer_switch = g_nav.steer_switch;
            gps_speed_kmh = g_nav.gps_speed_kmh;
            watchdog_triggered = g_nav.watchdog_triggered;
            pid_output = (int)g_nav.pid_output;
            settings_received = g_nav.settings_received;
            roll_deg = g_nav.roll_deg;
            yaw_rate_dps = g_nav.yaw_rate_dps;
            imu_quality_ok = g_nav.imu_quality_ok;
            imu_timestamp_ms = g_nav.imu_timestamp_ms;
        }

        const uint32_t imu_age_ms =
            (imu_timestamp_ms == 0U) ? 0U : (uint32_t)(now - imu_timestamp_ms);
        hal_log("STAT: hd=%.1f st=%.1f raw=%d safety=%s work=%s steer=%s spd=%.1f wdog=%s pid=%d tgt=%.1f roll_deg=%.2f yaw_rate_dps=%.2f imu_quality_ok=%s imu_age_ms=%lu net=%s cfg=%s",
                heading_deg,
                steer_angle_deg,
                steer_angle_raw,
                safety_ok ? "OK" : "KICK",
                work_switch ? "ON" : "OFF",
                steer_switch ? "ON" : "OFF",
                gps_speed_kmh,
                watchdog_triggered ? "TRIG" : "OK",
                pid_output,
                desiredSteerAngleDeg,
                roll_deg,
                yaw_rate_dps,
                imu_quality_ok ? "Y" : "N",
                (unsigned long)imu_age_ms,
                hal_net_is_connected() ? "UP" : "DOWN",
                settings_received ? "Y" : "N");
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
