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

void controlUpdateSettings(uint8_t kp, uint8_t ki, uint8_t kd,
                           uint16_t minPWM, uint16_t maxPWM) {
    // AgIO sends gains scaled by 10 (e.g., Kp=30 means 3.0)
    float new_kp = kp / 10.0f;
    float new_ki = ki / 10.0f;
    float new_kd = kd / 10.0f;

    // Only update if values actually changed
    if (new_kp != s_steer_pid.kp || new_ki != s_steer_pid.ki || new_kd != s_steer_pid.kd ||
        minPWM != (uint16_t)s_steer_pid.output_min || maxPWM != (uint16_t)s_steer_pid.output_max) {

        s_steer_pid.kp = new_kp;
        s_steer_pid.ki = new_ki;
        s_steer_pid.kd = new_kd;
        s_steer_pid.output_min = minPWM;
        s_steer_pid.output_max = maxPWM;

        // Reset integral on gain change to prevent windup from old gains
        s_steer_pid.integral = 0.0f;
        s_steer_pid.prev_error = 0.0f;
        s_steer_pid.first_update = true;

        hal_log("Control: settings updated Kp=%.1f Ki=%.1f Kd=%.1f minPWM=%u maxPWM=%u",
                new_kp, new_ki, new_kd, (unsigned)minPWM, (unsigned)maxPWM);
    }
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
        {
            StateLock lock;
            g_nav.pid_output = 0;
        }
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

    // Store raw ADC value for protocol messages
    {
        StateLock lock;
        g_nav.steer_angle_raw = hal_steer_angle_read_raw();
    }

    // ----------------------------------------------------------
    // 4. Check if auto-steer is enabled (work + steer switches)
    // ----------------------------------------------------------
    bool auto_steer = false;
    {
        StateLock lock;
        auto_steer = g_nav.work_switch && g_nav.steer_switch;
    }

    if (!auto_steer) {
        // Auto-steer disabled: hold current position or center
        // Reset PID to prevent windup while steering is off
        pidReset(&s_steer_pid);
        {
            StateLock lock;
            g_nav.pid_output = 0;
        }
        actuatorWriteCommand(0);
        return;
    }

    // ----------------------------------------------------------
    // 5. PID computation
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
    // 6. Write actuator command
    // ----------------------------------------------------------
    uint16_t cmd = static_cast<uint16_t>(output);
    actuatorWriteCommand(cmd);

    // ----------------------------------------------------------
    // 7. Update timestamp and PID output for status reporting
    // ----------------------------------------------------------
    {
        StateLock lock;
        g_nav.timestamp_ms = now_ms;
        g_nav.pid_output = cmd;
    }
}
