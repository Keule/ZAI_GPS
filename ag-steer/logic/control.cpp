/**
 * @file control.cpp
 * @brief PID controller and control loop implementation.
 *
 * Runs at 200 Hz on Core 1.
 * Sequence: Safety -> IMU -> Steer Angle -> PID -> Actuator
 */

#include "control.h"
#include "imu.h"
#include "steer_angle.h"
#include "actuator.h"
#include "global_state.h"
#include "hal/hal.h"

#include <cmath>

// ===================================================================
// Globals – actual definition in global_state.cpp
// ===================================================================

/// PID instance for steering
static PidState s_steer_pid;

// ===================================================================
// PID Implementation
// ===================================================================

void pidInit(PidState* pid, float kp, float ki, float kd,
             float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->last_update_ms = hal_millis();
    pid->first_update = true;
}

void pidReset(PidState* pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->first_update = true;
}

float pidCompute(PidState* pid, float error, uint32_t dt_ms) {
    if (dt_ms == 0) dt_ms = 5;  // safety: assume 5 ms (= 200 Hz)

    float dt_s = dt_ms * 0.001f;

    // Proportional
    float p_term = pid->kp * error;

    // Integral with anti-windup
    pid->integral += error * dt_s;
    float i_term = pid->ki * pid->integral;

    // Clamp integral to prevent windup
    if (i_term > pid->output_max) {
        i_term = pid->output_max;
        pid->integral = i_term / pid->ki;
    } else if (i_term < pid->output_min) {
        i_term = pid->output_min;
        pid->integral = i_term / pid->ki;
    }

    // Derivative (on error, not measurement)
    float d_term = 0.0f;
    if (!pid->first_update && dt_s > 0.0f) {
        float derivative = (error - pid->prev_error) / dt_s;
        d_term = pid->kd * derivative;
    }
    pid->first_update = false;
    pid->prev_error = error;

    // Sum
    float output = p_term + i_term + d_term;

    // Clamp output
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

// ===================================================================
// Control System
// ===================================================================

void controlInit(void) {
    imuInit();
    steerAngleInit();
    actuatorInit();

    // Default PID gains – tune for actual actuator/sensor combo
    pidInit(&s_steer_pid,
            1.0f,    // Kp
            0.0f,    // Ki
            0.01f,   // Kd
            0.0f,    // min output
            65535.0f // max output
    );

    hal_log("Control: initialised (PID Kp=%.2f Ki=%.3f Kd=%.3f)",
            s_steer_pid.kp, s_steer_pid.ki, s_steer_pid.kd);
}

void controlStep(void) {
    uint32_t now_ms = hal_millis();

    // ----------------------------------------------------------
    // 1. Safety check
    // ----------------------------------------------------------
    bool safety = hal_safety_ok();
    {
        StateLock lock;
        g_nav.safety_ok = safety;
    }

    if (!safety) {
        // Emergency: disable actuator immediately
        actuatorWriteCommand(0);
        pidReset(&s_steer_pid);
        return;
    }

    // ----------------------------------------------------------
    // 2. Read IMU (yaw rate, roll)
    // ----------------------------------------------------------
    imuUpdate();

    // ----------------------------------------------------------
    // 3. Read steering angle
    // ----------------------------------------------------------
    float current_angle = steerAngleReadDeg();

    // ----------------------------------------------------------
    // 4. PID computation
    // ----------------------------------------------------------
    float setpoint = desiredSteerAngleDeg;
    float error = setpoint - current_angle;

    // Wrap error to [-180, +180]
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    uint32_t dt = now_ms - s_steer_pid.last_update_ms;
    if (dt > 100) dt = 5;  // prevent huge dt after pause
    s_steer_pid.last_update_ms = now_ms;

    float output = pidCompute(&s_steer_pid, error, dt);

    // ----------------------------------------------------------
    // 5. Write actuator command
    // ----------------------------------------------------------
    uint16_t cmd = static_cast<uint16_t>(output);
    actuatorWriteCommand(cmd);

    // ----------------------------------------------------------
    // 6. Update timestamp
    // ----------------------------------------------------------
    {
        StateLock lock;
        g_nav.timestamp_ms = now_ms;
    }
}
