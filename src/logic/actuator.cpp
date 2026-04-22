/**
 * @file actuator.cpp
 * @brief Steering actuator implementation.
 *
 * Uses HAL SPI interface. Actual command protocol is stub for now.
 */

#include "actuator.h"
#include "hal/hal.h"

#include "log_config.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_ACT
#include "esp_log.h"
#include "log_ext.h"

void actuatorWriteCommand(uint16_t cmd) {
    hal_actuator_write(cmd);
}
