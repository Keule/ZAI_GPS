/**
 * @file imu.h
 * @brief IMU driver – BNO085 over SPI (stub implementation).
 *
 * Reads yaw rate and roll angle via HAL SPI interface.
 */

#pragma once

/// Initialise IMU hardware (calls hal_imu_begin).
void imuInit(void);

/// Read IMU data and update global state (yaw_rate_dps, roll_deg).
/// Returns true if read was successful.
bool imuUpdate(void);
