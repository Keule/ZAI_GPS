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
// Input metadata + capability guards
// ---------------------------------------------------------------------------
struct InputMeta {
    uint32_t timestamp_ms = 0;  // Last update timestamp [ms since boot]
    uint8_t  quality = 0;       // 0..100 quality score
    bool     valid = false;     // true if latest value passed validation
};

struct NavigationState;

enum class Capability : uint8_t {
    SteerDataIn,
    Imu,
    SteerAngle,
    SteerSettings,
    SteerConfig
};

/// Mark input metadata for a specific capability.
void markInputMeta(Capability capability,
                   uint32_t timestamp_ms,
                   uint8_t quality,
                   bool valid);

/// Utility: check if metadata is fresh (handles uint32 wraparound).
bool isFresh(const InputMeta& meta, uint32_t now_ms, uint32_t max_age_ms);

/// Utility: input must be valid and fresh.
bool isValidAndFresh(const InputMeta& meta, uint32_t now_ms, uint32_t max_age_ms);

/// Output guard: build/send PGN 253 (Steer Status Out) allowed.
bool canBuildSteerStatusOut(const NavigationState& nav, uint32_t now_ms);

/// Output guard: build/send PGN 250 (From Autosteer 2) allowed.
bool canBuildFromAutosteer2(const NavigationState& nav, uint32_t now_ms);

/// Output guard: actuator command path allowed.
bool canActuateSteer(const NavigationState& nav, uint32_t now_ms);

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
    uint8_t  settings_kp;           // proportional gain from AgIO
    uint8_t  settings_high_pwm;     // maximum actuator PWM from AgIO
    uint8_t  settings_low_pwm;      // deadband PWM from AgIO
    uint8_t  settings_min_pwm;      // minimum actuator PWM from AgIO
    uint8_t  settings_counts;       // sensor counts per degree from AgIO
    int16_t  settings_was_offset;   // sensor zero offset from AgIO
    uint8_t  settings_ackerman;     // Ackerman correction factor from AgIO (*100)
    bool     settings_received;     // true after first settings from AgIO

    // --- Steering config from AgIO (PGN 251) ---
    uint8_t  config_set0;           // hardware config bits from AgIO
    uint8_t  config_max_pulse;      // pulse count max threshold
    uint8_t  config_min_speed;      // minimum speed for steering
    bool     config_received;       // true after first config from AgIO

    // --- Watchdog (AgIO heartbeat) ---
    uint32_t watchdog_timer_ms;     // ms since last valid PGN 254
    bool     watchdog_triggered;     // true = no PGN 254 received for >2.5s

    // --- Speed safety ---
    float    gps_speed_kmh;          // current GPS speed [km/h] from PGN 254

    // --- PID output (for status reporting) ---
    uint16_t pid_output;      // current PID output (actuator command)

    // --- Timing ---
    uint32_t timestamp_ms;    // last update timestamp [ms]

    // --- Input metadata ---
    InputMeta meta_steer_data;
    InputMeta meta_imu;
    InputMeta meta_steer_angle;
    InputMeta meta_steer_settings;
    InputMeta meta_steer_config;
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
