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
    hal_log("Control: task started on core %d", xPortGetCoreID());

    // Wait for network + sensors to stabilise
    vTaskDelay(pdMS_TO_TICKS(500));

    const TickType_t interval = pdMS_TO_TICKS(5);  // 200 Hz = 5 ms

    for (;;) {
        uint32_t start = hal_millis();

        // Run one control step
        controlStep();

        // Buffer one log record (subsampled to 10 Hz internally)
        sdLoggerRecord();

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
    hal_log("Comm: task started on core %d", xPortGetCoreID());

    // Wait for network to initialise (done in setup, but give time to settle)
    vTaskDelay(pdMS_TO_TICKS(2000));

    const TickType_t poll_interval = pdMS_TO_TICKS(10);  // 100 Hz polling

    // Hardware status update runs at ~1 Hz
    static uint32_t s_last_hw_status_ms = 0;
    static const uint32_t HW_STATUS_INTERVAL_MS = 1000;

    for (;;) {
        // Poll network for incoming frames
        netPollReceive();

        // Send periodic AOG frames (~10 Hz)
        netSendAogFrames();

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
                hal_log("COMM: %u HW error(s) active", (unsigned)err_count);
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
        hal_log("========================================");
        hal_log("  AgSteer Firmware v%u.%u.%u", ver.major, ver.minor, ver.patch);
        hal_log("  Build: %s %s", __DATE__, __TIME__);
        hal_log("  Partition: %s (0x%06X)", part ? part->label : "?",
                part ? (unsigned)part->address : 0);
        hal_log("  Flash: %d KB free", (int)(ESP.getFreeSketchSpace() / 1024));
        hal_log("========================================");
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
            hal_log("Main: %s calibration",
                    hal_steer_angle_is_calibrated() ? "forced re" : "initial");
            hal_steer_angle_calibrate();
        } else {
            hal_log("Main: calibration OK (loaded from NVS)");
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

    hal_log("Main: tasks created, entering main loop");
}

// ===================================================================
// Arduino loop() – not used for real work (tasks handle everything)
// ===================================================================
void loop() {
    // Feed watchdog to prevent trigger from this task
    esp_task_wdt_reset();

    // Periodic status output every 5 seconds
    static uint32_t s_last_status = 0;
    uint32_t now = hal_millis();
    if (now - s_last_status >= 5000) {
        s_last_status = now;
        StateLock lock;
        hal_log("STAT: hd=%.1f st=%.1f raw=%d safety=%s work=%s steer=%s pid=%.1f tgt=%.1f net=%s",
                g_nav.heading_deg,
                g_nav.steer_angle_deg,
                (int)g_nav.steer_angle_raw,
                g_nav.safety_ok ? "OK" : "KICK",
                g_nav.work_switch ? "ON" : "OFF",
                g_nav.steer_switch ? "ON" : "OFF",
                desiredSteerAngleDeg,
                hal_net_is_connected() ? "UP" : "DOWN");
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}
