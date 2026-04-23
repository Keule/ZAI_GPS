/**
 * @file imu.h
 * @brief IMU driver – BNO085 over SPI (stub implementation).
 *
 * Reads yaw rate and roll angle via HAL SPI interface.
 */

#pragma once

#include <cstdint>

#include "features.h"
#include "module_interface.h"

/// Compile-time check: is IMU compiled in?
constexpr bool imuIsEnabled() { return feat::imu(); }

/// Initialise IMU hardware (calls hal_imu_begin).
void imuInit(void);

/// Read IMU data and update global state (yaw_rate_dps, roll_deg).
/// Returns true if read was successful.
bool imuUpdate(void);

/// Check if IMU data is fresh.
bool imuIsHealthy(uint32_t now_ms);

/// True if IMU bring-up mode is enabled at compile time.
bool imuBringupModeEnabled(void);

/// Initialise IMU bring-up diagnostics.
void imuBringupInit(void);

/// Run periodic IMU bring-up diagnostics (non-blocking).
void imuBringupTick(void);

/// Module registry entry for IMU.
extern const ModuleOps imu_ops;
