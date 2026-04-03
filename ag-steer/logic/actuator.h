/**
 * @file actuator.h
 * @brief Steering actuator driver over SPI.
 *
 * Writes a command (0..65535) to the steering actuator.
 */

#pragma once

#include <cstdint>

/// Initialise actuator SPI interface.
void actuatorInit(void);

/// Write command to actuator (0 = idle, >0 = steer command).
void actuatorWriteCommand(uint16_t cmd);
