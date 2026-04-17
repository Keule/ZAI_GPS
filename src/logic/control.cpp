/**
 * @file control.cpp
 * @brief PID controller and control loop implementation.
 *
 * Runs at 200 Hz on Core 1.
 * Sequence: Safety -> IMU -> Steer Angle -> Watchdog -> PID -> Actuator
 */

#include "control.h"
#include "dependency_policy.h"
#include "imu.h"
#include "steer_angle.h"
#include "actuator.h"
#include "global_state.h"
#include "steer_config_bits.h"
#include "hal/hal.h"

#include "log_config.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_CTL
#include "esp_log.h"
#include "log_ext.h"

#include <cmath>

// ===================================================================
// Constants
// ===================================================================

/// Minimum GPS speed [km/h] to allow auto-steering.
/// Reference disables steering below 0.1 km/h for safety.
constexpr float MIN_STEER_SPEED_KMH = 0.1f;

/// Hard command limits for the 48V motor stage.
constexpr uint16_t ACTUATOR_PWM_ABSOLUTE_MAX = 220;
constexpr uint16_t ACTUATOR_RAMP_STEP_PER_CYCLE = 8;
constexpr uint32_t STEER_DATA_FRESHNESS_TIMEOUT_MS = 300;

// ===================================================================
// Globals – actual definition in global_state.cpp
// ===================================================================

/// PID instance for steering
static PidState s_steer_pid;
static uint32_t s_last_was_diag_ms = 0;
static uint16_t s_last_actuator_cmd = 0;

enum class GateBlockReason : uint8_t {
    None = 0,
    Safety,
    AutoSteerDisabled,
    WatchdogTimeout,
    SpeedBelowMinimum,
    DataStale,
    SteerSensorInvalid,
};

static const char* gateBlockReasonToStr(GateBlockReason reason) {
    switch (reason) {
        case GateBlockReason::None: return "NONE";
        case GateBlockReason::Safety: return "SAFETY";
        case GateBlockReason::AutoSteerDisabled: return "AUTO_STEER_OFF";
        case GateBlockReason::WatchdogTimeout: return "WATCHDOG";
        case GateBlockReason::SpeedBelowMinimum: return "SPEED";
        case GateBlockReason::DataStale: return "STALE_DATA";
        case GateBlockReason::SteerSensorInvalid: return "STEER_SENSOR";
        default: return "UNKNOWN";
    }
}

static uint16_t clampActuatorCmd(uint16_t cmd) {
    return (cmd > ACTUATOR_PWM_ABSOLUTE_MAX) ? ACTUATOR_PWM_ABSOLUTE_MAX : cmd;
}

static uint16_t applyRampLimit(uint16_t previous_cmd, uint16_t target_cmd) {
    if (target_cmd > previous_cmd) {
        const uint16_t max_up = static_cast<uint16_t>(previous_cmd + ACTUATOR_RAMP_STEP_PER_CYCLE);
        return target_cmd > max_up ? max_up : target_cmd;
    }

    const uint16_t max_down = (previous_cmd > ACTUATOR_RAMP_STEP_PER_CYCLE)
        ? static_cast<uint16_t>(previous_cmd - ACTUATOR_RAMP_STEP_PER_CYCLE)
        : 0u;
    return target_cmd < max_down ? max_down : target_cmd;
}

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
    struct ControlInputSnapshot {
        uint32_t now_ms = 0;
        bool safety_ok = false;
        bool auto_steer_enabled = false;
        float gps_speed_kmh = 0.0f;
        uint32_t watchdog_timer_ms = 0;
        float setpoint_deg = 0.0f;
        float current_angle_deg = 0.0f;
        int16_t steer_raw = 0;
        uint8_t config_set0 = 0;
        uint8_t config_min_speed = 0;
    };

    struct ControlOutputSnapshot {
        uint16_t actuator_cmd = 0;
        bool watchdog_triggered = false;
        bool reset_pid = false;
        bool gate_enabled = false;
        GateBlockReason gate_block_reason = GateBlockReason::None;
    };

    // ----------------------------------------------------------
    // Input phase (read once, then process from snapshot)
    // ----------------------------------------------------------
    ControlInputSnapshot in;
    in.now_ms = hal_millis();
    in.safety_ok = hal_safety_ok();
    imuUpdate();
    in.current_angle_deg = steerAngleReadDeg();
    in.steer_raw = hal_steer_angle_read_raw();
    in.setpoint_deg = desiredSteerAngleDeg;

#if LOG_WAS_DIAG_INTERVAL_MS > 0
    if (in.now_ms - s_last_was_diag_ms >= LOG_WAS_DIAG_INTERVAL_MS) {
        s_last_was_diag_ms = in.now_ms;
        hal_log("WAS-DIAG: angle=%.2f deg raw=%d safety=%s",
                in.current_angle_deg,
                (int)in.steer_raw,
                in.safety_ok ? "OK" : "KICK");
    }
#endif

    {
        StateLock lock;
        in.auto_steer_enabled = g_nav.work_switch && g_nav.steer_switch;
        in.gps_speed_kmh = g_nav.gps_speed_kmh;
        in.watchdog_timer_ms = g_nav.watchdog_timer_ms;
        in.config_set0 = g_nav.config_set0;
        in.config_min_speed = g_nav.config_min_speed;
    }

    // ----------------------------------------------------------
    // Processing phase (pure computation / decisions)
    // ----------------------------------------------------------
    ControlOutputSnapshot out;
    // Arm watchdog only after first valid PGN 254 heartbeat was received.
    out.watchdog_triggered = (in.watchdog_timer_ms != 0u) &&
                             (in.now_ms - in.watchdog_timer_ms > dep_policy::WATCHDOG_TIMEOUT_MS);
    const bool steer_data_fresh = dep_policy::isFresh(
        in.now_ms, in.watchdog_timer_ms, STEER_DATA_FRESHNESS_TIMEOUT_MS);
    const bool steer_sensor_quality_ok =
        dep_policy::isSteerAnglePlausible(in.current_angle_deg) &&
        dep_policy::isSteerAngleRawPlausible(in.steer_raw);

    if (!in.safety_ok) {
        out.gate_block_reason = GateBlockReason::Safety;
    } else if (!in.auto_steer_enabled) {
        out.gate_block_reason = GateBlockReason::AutoSteerDisabled;
    } else if (out.watchdog_triggered) {
        out.gate_block_reason = GateBlockReason::WatchdogTimeout;
    } else if (in.gps_speed_kmh < MIN_STEER_SPEED_KMH) {
        out.gate_block_reason = GateBlockReason::SpeedBelowMinimum;
    } else if (!steer_data_fresh) {
        out.gate_block_reason = GateBlockReason::DataStale;
    } else if (!steer_sensor_quality_ok) {
        out.gate_block_reason = GateBlockReason::SteerSensorInvalid;
    } else {
        out.gate_enabled = true;
    }

    static bool s_prev_gate_enabled = false;
    static GateBlockReason s_prev_gate_reason = GateBlockReason::None;

    if (!out.gate_enabled) {
    const bool invert_motor_direction = (in.config_set0 & steer_cfg_set0::MOTOR_DRIVE_DIRECTION) != 0u;
    const float cfg_min_speed_kmh = static_cast<float>(in.config_min_speed) * 0.1f;
    const float effective_min_speed_kmh =
        (cfg_min_speed_kmh > MIN_STEER_SPEED_KMH) ? cfg_min_speed_kmh : MIN_STEER_SPEED_KMH;
    }
    if (!in.safety_ok || !in.auto_steer_enabled ||
        out.watchdog_triggered || in.gps_speed_kmh < effective_min_speed_kmh) {
        out.actuator_cmd = 0;
        out.reset_pid = true;

        if (s_prev_gate_enabled || s_prev_gate_reason != out.gate_block_reason) {
            LOGW("CTL", "gate OFF reason=%s safety=%d auto=%d watchdog=%d speed=%.2f dataFresh=%d",
                    gateBlockReasonToStr(out.gate_block_reason),
                    in.safety_ok ? 1 : 0,
                    in.auto_steer_enabled ? 1 : 0,
                    out.watchdog_triggered ? 1 : 0,
                    (double)in.gps_speed_kmh,
                    steer_data_fresh ? 1 : 0);
        }
    } else {
        if (!s_prev_gate_enabled) {
            LOGI("CTL", "gate ON safety=%d auto=%d speed=%.2f dataFresh=%d",
                    in.safety_ok ? 1 : 0,
                    in.auto_steer_enabled ? 1 : 0,
                    (double)in.gps_speed_kmh,
                    steer_data_fresh ? 1 : 0);
        }

        float error = in.setpoint_deg - in.current_angle_deg;
        while (error > 180.0f)  error -= 360.0f;
        while (error < -180.0f) error += 360.0f;
        if (invert_motor_direction) {
            error = -error;
        }

        uint32_t dt = in.now_ms - s_steer_pid.last_update_ms;
        if (dt > 100) dt = 5;  // prevent huge dt after pause
        s_steer_pid.last_update_ms = in.now_ms;

        const float output = pidCompute(&s_steer_pid, error, dt);
        const uint16_t raw_cmd = static_cast<uint16_t>(output);
        const uint16_t clamped_cmd = clampActuatorCmd(raw_cmd);
        out.actuator_cmd = applyRampLimit(s_last_actuator_cmd, clamped_cmd);
    }

    s_prev_gate_enabled = out.gate_enabled;
    s_prev_gate_reason = out.gate_block_reason;

    // ----------------------------------------------------------
    // Output phase (single writer update + actuator command)
    // ----------------------------------------------------------
    if (out.reset_pid) {
        pidReset(&s_steer_pid);
    }

    actuatorWriteCommand(out.actuator_cmd);
    s_last_actuator_cmd = out.actuator_cmd;

    {
        const bool steer_quality_ok =
            dep_policy::isSteerAnglePlausible(in.current_angle_deg) &&
            dep_policy::isSteerAngleRawPlausible(in.steer_raw);

        StateLock lock;
        g_nav.safety_ok = in.safety_ok;
        g_nav.steer_angle_raw = in.steer_raw;
        g_nav.steer_angle_timestamp_ms = in.now_ms;
        g_nav.steer_angle_quality_ok = steer_quality_ok;
        g_nav.watchdog_triggered = out.watchdog_triggered;
        g_nav.timestamp_ms = in.now_ms;
        g_nav.pid_output = out.actuator_cmd;
    }
}
