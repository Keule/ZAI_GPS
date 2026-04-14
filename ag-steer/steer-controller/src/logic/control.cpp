/**
 * @file control.cpp
 * @brief PID controller and control loop implementation.
 *
 * Runs at 200 Hz on Core 1.
 * Sequence: Safety -> IMU -> Steer Angle -> Watchdog -> PID -> Actuator
 */

#include "control.h"
#include "imu.h"
#include "steer_angle.h"
#include "actuator.h"
#include "global_state.h"
#include "hal/hal.h"

#include "log_config.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_CTL
#include "esp_log.h"
#include "log_ext.h"

#include <cmath>

// ===================================================================
// Constants
// ===================================================================

/// Watchdog: disable steering if no PGN 254 received for this many ms
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 2500;

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
    // NOTE: HAL-level init (hal_imu_begin, hal_steer_angle_begin,
    // hal_actuator_begin) is already done in hal_esp32_init_all().
    // imuInit/steerAngleInit/actuatorInit would redundantly call
    // the HAL begin functions and produce duplicate log messages.
    // We only need to init the PID controller here.

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

void controlUpdateSettings(uint8_t kp, uint8_t highPWM, uint8_t lowPWM,
                           uint8_t minPWM, uint8_t countsPerDegree,
                           int16_t wasOffset, uint8_t ackerman) {
    // AgOpenGPS sends Kp as raw value (e.g. 30 = Kp 3.0)
    float new_kp = kp;

    // Reference applies: lowPWM = minPWM * 1.2 (overrides AgIO value)
    uint8_t effective_lowPWM = static_cast<uint8_t>(minPWM * 1.2f);

    float new_out_min = static_cast<float>(minPWM);
    float new_out_max = static_cast<float>(highPWM);

    // Only update if values actually changed
    if (new_kp != s_steer_pid.kp ||
        new_out_min != s_steer_pid.output_min ||
        new_out_max != s_steer_pid.output_max) {

        s_steer_pid.kp = new_kp;
        s_steer_pid.output_min = new_out_min;
        s_steer_pid.output_max = new_out_max;

        // Reset integral on gain change to prevent windup from old gains
        s_steer_pid.integral = 0.0f;
        s_steer_pid.prev_error = 0.0f;
        s_steer_pid.first_update = true;

        LOGI("CTL", "settings updated Kp=%.0f hiPWM=%u loPWM=%u(eff=%u) minPWM=%u counts=%u ack=%u",
                (float)kp, (unsigned)highPWM, (unsigned)lowPWM,
                (unsigned)effective_lowPWM, (unsigned)minPWM,
                (unsigned)countsPerDegree, (unsigned)ackerman);
    }

    // Store all settings in global state for status reporting
    {
        StateLock lock;
        g_nav.settings_kp           = kp;
        g_nav.settings_high_pwm     = highPWM;
        g_nav.settings_low_pwm      = lowPWM;
        g_nav.settings_min_pwm      = minPWM;
        g_nav.settings_counts       = countsPerDegree;
        g_nav.settings_was_offset   = wasOffset;
        g_nav.settings_ackerman     = ackerman;
        g_nav.settings_received     = true;
    }
}

void controlStep(void) {
    uint32_t now_ms = hal_millis();

    // ----------------------------------------------------------
    // 1. Safety check (hardware safety circuit)
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
    // 5. Capability guard: watchdog/freshness + speed + validity
    // ----------------------------------------------------------
    bool allow_actuation = false;
    {
        StateLock lock;
        g_nav.watchdog_triggered = !isValidAndFresh(
            g_nav.meta_steer_data, now_ms, WATCHDOG_TIMEOUT_MS);
        allow_actuation = canActuateSteer(g_nav, now_ms);
    }

    if (!allow_actuation) {
        // One of the capability conditions is not met -> force safe output.
        pidReset(&s_steer_pid);
        {
            StateLock lock;
            g_nav.pid_output = 0;
        }
        actuatorWriteCommand(0);
        return;
    }

    // ----------------------------------------------------------
    // 6. PID computation
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
    // 7. Write actuator command
    // ----------------------------------------------------------
    uint16_t cmd = static_cast<uint16_t>(output);
    actuatorWriteCommand(cmd);

    // ----------------------------------------------------------
    // 8. Update timestamp and PID output for status reporting
    // ----------------------------------------------------------
    {
        StateLock lock;
        g_nav.timestamp_ms = now_ms;
        g_nav.pid_output = cmd;
    }
}
