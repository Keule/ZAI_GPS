/**
 * @file main.cpp
 * @brief ESP32-S3 firmware main entry point.
 *
 * Target: LilyGO T-ETH-Lite-S3 (ESP32-S3-WROOM-1 + W5500 Ethernet)
 *
 * Two FreeRTOS tasks:
 *   - commTask  (Core 0): GNSS polling, Ethernet/UDP, AOG protocol
 *   - controlTask (Core 1): 200 Hz control loop, PID, safety, actuators
 */

#include <Arduino.h>
#include <FreeRTOS.h>

#include "hal/hal.h"
#include "hal_esp32/hal_impl.h"
#include "logic/global_state.h"
#include "logic/control.h"
#include "logic/gnss.h"
#include "logic/net.h"

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

    // Init control subsystem
    controlInit();

    const TickType_t interval = pdMS_TO_TICKS(5);  // 200 Hz = 5 ms

    for (;;) {
        uint32_t start = hal_millis();

        // Run one control step
        controlStep();

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

    // Wait for network to initialise
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Init GNSS
    gnssInit();

    const TickType_t poll_interval = pdMS_TO_TICKS(10);  // 100 Hz polling

    for (;;) {
        // Poll GNSS (main + heading)
        gnssPollMain();
        gnssPollHeading();

        // Poll network for incoming frames
        netPollReceive();

        // Send periodic AOG frames (~10 Hz)
        netSendAogFrames();

        vTaskDelay(poll_interval);
    }
}

// ===================================================================
// Arduino setup()
// ===================================================================
void setup() {
    // Initialise all hardware
    hal_esp32_init_all();

    // Create control task on Core 1 (PRO_CPU_NUM on ESP32-S3 = core 0 or 1)
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
    // Monitor / watchdog / debug output
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Optional: periodic status output
    {
        StateLock lock;
        hal_log("STAT: lat=%.6f lon=%.6f fix=%u sog=%.1f heading=%.1f "
                "steer=%.1f safety=%s pid_tgt=%.1f",
                g_nav.lat_deg, g_nav.lon_deg,
                (unsigned)g_nav.fix_quality,
                g_nav.sog_mps,
                g_nav.heading_deg,
                g_nav.steer_angle_deg,
                g_nav.safety_ok ? "OK" : "KICK",
                desiredSteerAngleDeg);
    }
}
