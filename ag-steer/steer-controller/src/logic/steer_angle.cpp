/**
 * @file steer_angle.cpp
 * @brief Steering angle sensor implementation.
 *
 * Uses HAL SPI interface. Actual sensor protocol is stub for now.
 */

#include "steer_angle.h"
#include "global_state.h"
#include "hal/hal.h"

#include "log_config.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_WAS
#include "esp_log.h"
#include "log_ext.h"

#include <cmath>

void steerAngleInit(void) {
    hal_steer_angle_begin();
    LOGI("WAS", "initialised (SPI stub)");
}

float steerAngleReadDeg(void) {
    float angle = hal_steer_angle_read_deg();
    const uint32_t now_ms = hal_millis();
    const bool valid = std::isfinite(angle);

    {
        StateLock lock;
        g_nav.steer_angle_deg = angle;
    }
    markInputMeta(Capability::SteerAngle, now_ms, valid ? 100 : 0, valid);

    return angle;
}
