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

#include "log_config.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_IMU
#include "esp_log.h"
#include "log_ext.h"

void imuInit(void) {
    hal_imu_begin();
    LOGI("IMU", "initialised (BNO085 SPI stub)");
}

bool imuUpdate(void) {
    float yaw_rate = 0.0f;
    float roll = 0.0f;
    const uint32_t now_ms = hal_millis();

    if (!hal_imu_read(&yaw_rate, &roll)) {
        markInputMeta(Capability::Imu, now_ms, 0, false);
        return false;
    }

    {
        StateLock lock;
        g_nav.yaw_rate_dps = yaw_rate;
        g_nav.roll_deg = roll;
        g_nav.timestamp_ms = now_ms;
    }
    markInputMeta(Capability::Imu, now_ms, 100, true);

    return true;
}
