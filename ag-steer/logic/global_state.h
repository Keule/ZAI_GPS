/**
 * @file global_state.h
 * @brief Global navigation and vehicle state, shared between tasks.
 *
 * Pure C++ – no Arduino / ESP32 headers.
 * All accesses should be protected with the mutex helpers provided.
 */

#pragma once

#include <cstdint>
#include <cstdbool>

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
    // --- GNSS Main ---
    double  lat_deg;          // WGS84 latitude  [degrees]
    double  lon_deg;          // WGS84 longitude [degrees]
    float   alt_m;            // altitude above ellipsoid [m]
    float   sog_mps;          // speed over ground [m/s]
    float   cog_deg;          // course over ground [degrees, 0-360]
    uint8_t fix_quality;      // 0=none, 1=GPS, 2=DGPS, 4=RTK fix, 5=RTK float

    // --- Heading / IMU ---
    float   heading_deg;      // fused or GNSS heading [degrees, 0-360]
    float   roll_deg;         // roll angle [degrees]
    float   yaw_rate_dps;     // yaw rate [degrees/second]

    // --- Steering ---
    float   steer_angle_deg;  // current measured steering angle [degrees]
    bool    safety_ok;        // true = safety circuit OK, false = KICK

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
