/**
 * @file imu.cpp
 * @brief IMU driver implementation (stub for BNO085 SPI).
 *
 * TODO: Implement full BNO085 SH-2 / sensor hub protocol over SPI.
 * Currently reads via HAL stub (PC sim returns dummy values).
 */

#include "imu.h"
#include "global_state.h"
#include "hal/hal.h"

void imuInit(void) {
    hal_imu_begin();
    hal_log("IMU: initialised (BNO085 SPI stub)");
}

bool imuUpdate(void) {
    float yaw_rate = 0.0f;
    float roll = 0.0f;

    if (!hal_imu_read(&yaw_rate, &roll)) {
        return false;
    }

    {
        StateLock lock;
        g_nav.yaw_rate_dps = yaw_rate;
        g_nav.roll_deg = roll;
        g_nav.timestamp_ms = hal_millis();
    }

    return true;
}
