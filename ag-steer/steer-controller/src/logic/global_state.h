/**
 * @file global_state.h
 * @brief Global navigation and vehicle state, shared between tasks.
 *
 * Pure C++ – no Arduino / ESP32 headers.
 * All accesses should be protected with the mutex helpers provided.
 */

#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// Mutex abstraction – implemented in HAL
// ---------------------------------------------------------------------------
extern "C" {
    /// Create / init the global state mutex. Called once during setup.
    void  hal_mutex_init(void);

    /// Lock the global state mutex.
    void  hal_mutex_lock(void);

    /// Unlock the global state mutex.
    void  hal_mutex_unlock(void);
}

// ---------------------------------------------------------------------------
// Navigation state – single source of truth for control + comms tasks.
// ---------------------------------------------------------------------------
struct NavigationState {
    // --- Heading / IMU ---
    float   heading_deg;      // fused heading [degrees, 0-360]
    float   roll_deg;         // roll angle [degrees]
    float   yaw_rate_dps;     // yaw rate [degrees/second]

    // --- Steering ---
    float   steer_angle_deg;  // current measured steering angle [degrees]
    int16_t steer_angle_raw;  // raw ADC value from ADS1118
    bool    safety_ok;        // true = safety circuit OK, false = KICK

    // --- AgIO switches (from PGN 254 steer data status byte) ---
    bool    work_switch;      // bit 0 of status: work switch active
    bool    steer_switch;     // bit 1 of status: auto-steer enabled
    uint8_t last_status_byte; // raw status byte from last PGN 254

    // --- Steering settings from AgIO (PGN 252) ---
    uint8_t settings_ack;     // last ack number received (for tracking)
    float   settings_kp;      // proportional gain from AgIO
    float   settings_ki;      // integral gain from AgIO
    float   settings_kd;      // derivative gain from AgIO
    uint16_t settings_min_pwm;// actuator min PWM from AgIO
    uint16_t settings_max_pwm;// actuator max PWM from AgIO
    uint16_t settings_counts; // sensor total counts from AgIO
    int8_t  settings_hi;      // angle limit left [degrees]
    int8_t  settings_lo;      // angle limit right [degrees]
    int16_t settings_was_offset; // sensor zero offset from AgIO
    bool    settings_received; // true after first settings from AgIO

    // --- PID output (for status reporting) ---
    uint16_t pid_output;      // current PID output (actuator command)

    // --- Timing ---
    uint32_t timestamp_ms;    // last update timestamp [ms]
};

// ---------------------------------------------------------------------------
// Global instance (defined in control.cpp or main)
// ---------------------------------------------------------------------------
extern NavigationState g_nav;

// ---------------------------------------------------------------------------
// Setpoint – written by commTask when steer data from AgIO arrives
// ---------------------------------------------------------------------------
extern volatile float desiredSteerAngleDeg;

// ---------------------------------------------------------------------------
// RAII helper: lock mutex in ctor, unlock in dtor.
// ---------------------------------------------------------------------------
class StateLock {
public:
    StateLock()  { hal_mutex_lock(); }
    ~StateLock() { hal_mutex_unlock(); }
};

// ---------------------------------------------------------------------------
// Simple scoped lock for arbitrary lock/unlock function pointers.
// ---------------------------------------------------------------------------
template <void (*LockFn)(), void (*UnlockFn)()>
class ScopedLock {
public:
    ScopedLock()  { LockFn(); }
    ~ScopedLock() { UnlockFn(); }
    ScopedLock(const ScopedLock&) = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;
};
