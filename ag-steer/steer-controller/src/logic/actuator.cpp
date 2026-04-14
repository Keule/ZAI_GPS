/**
 * @file actuator.cpp
 * @brief Steering actuator implementation.
 *
 * Uses HAL SPI interface. Actual command protocol is stub for now.
 */

#include "actuator.h"
#include "global_state.h"
#include "hal/hal.h"

#include "log_config.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_ACT
#include "esp_log.h"
#include "log_ext.h"

void actuatorInit(void) {
    hal_actuator_begin();
    LOGI("ACT", "initialised (SPI stub)");
}

void actuatorWriteCommand(uint16_t cmd) {
    if (cmd > 0) {
        const uint32_t now_ms = hal_millis();
        StateLock lock;
        if (!canActuateSteer(g_nav, now_ms)) {
            cmd = 0;
        }
    }
    hal_actuator_write(cmd);
}
