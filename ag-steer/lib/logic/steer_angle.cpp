/**
 * @file steer_angle.cpp
 * @brief Steering angle sensor implementation.
 *
 * Uses HAL SPI interface. Actual sensor protocol is stub for now.
 */

#include "steer_angle.h"
#include "global_state.h"
#include "hal/hal.h"

void steerAngleInit(void) {
    hal_steer_angle_begin();
    hal_log("SteerAngle: initialised (SPI stub)");
}

float steerAngleReadDeg(void) {
    float angle = hal_steer_angle_read_deg();

    {
        StateLock lock;
        g_nav.steer_angle_deg = angle;
    }

    return angle;
}
