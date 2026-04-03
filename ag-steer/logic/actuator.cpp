/**
 * @file actuator.cpp
 * @brief Steering actuator implementation.
 *
 * Uses HAL SPI interface. Actual command protocol is stub for now.
 */

#include "actuator.h"
#include "hal/hal.h"

void actuatorInit(void) {
    hal_actuator_begin();
    hal_log("Actuator: initialised (SPI stub)");
}

void actuatorWriteCommand(uint16_t cmd) {
    hal_actuator_write(cmd);
}
