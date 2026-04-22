/**
 * @file imu.h
 * @brief IMU driver – BNO085 over SPI (stub implementation).
 *
 * Reads yaw rate and roll angle via HAL SPI interface.
 */

#pragma once

/// Read IMU data and update global state (yaw_rate_dps, roll_deg).
/// Returns true if read was successful.
bool imuUpdate(void);

/// True if IMU bring-up mode is enabled at compile time.
bool imuBringupModeEnabled(void);

/// Initialise IMU bring-up diagnostics.
void imuBringupInit(void);

/// Run periodic IMU bring-up diagnostics (non-blocking).
void imuBringupTick(void);
