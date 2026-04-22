/**
 * @file steer_angle.cpp
 * @brief Steering angle sensor implementation.
 *
 * Uses HAL SPI interface. Actual sensor protocol is stub for now.
 */

#include "steer_angle.h"
#include "dependency_policy.h"
#include "global_state.h"
#include "hal/hal.h"

#include "log_config.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_WAS
#include "esp_log.h"
#include "log_ext.h"

float steerAngleReadDeg(void) {
    float angle = hal_steer_angle_read_deg();
    const uint32_t now_ms = hal_millis();
    const bool plausible = dep_policy::isSteerAnglePlausible(angle);

    {
        StateLock lock;
        g_nav.steer_angle_deg = angle;
        g_nav.steer_angle_timestamp_ms = now_ms;
        g_nav.steer_angle_quality_ok = plausible;
    }

    return angle;
}
