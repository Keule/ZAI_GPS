/**
 * @file global_state.h
 * @brief Global navigation and vehicle state, shared between tasks.
 *
 * Pure C++ – no Arduino / ESP32 headers.
 * All accesses should be protected with the mutex helpers provided.
 */

#pragma once

#include <cstddef>
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
    uint32_t heading_timestamp_ms; // last successful heading update [ms]
    bool    heading_quality_ok; // true when heading is plausible/available
    uint32_t imu_timestamp_ms; // last successful IMU update [ms]
    bool    imu_quality_ok;    // true when most recent IMU sample is plausible

    // --- Steering ---
    float   steer_angle_deg;  // current measured steering angle [degrees]
    int16_t steer_angle_raw;  // raw ADC value from ADS1118
    uint32_t steer_angle_timestamp_ms; // last steering sensor update [ms]
    bool    steer_angle_quality_ok;    // true when steering sample is plausible
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

    // --- GNSS/UM980 status for PGN 214 ---
    uint8_t  gps_fix_quality;        // AOG PGN214 fix quality code (0/1/2/4/5)
    int16_t  gps_diff_age_x100_ms;   // Differential age encoded as ms*100 (saturated int16)
    uint8_t  um980_fix_type;         // Raw UM980 fix type / GGA quality code for diagnostics
    bool     um980_rtcm_active;      // true when RTCM corrections are currently active
    uint32_t um980_status_timestamp_ms; // last UM980 status update timestamp [ms]

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
// NTRIP state — TASK-025
// ---------------------------------------------------------------------------
/// NTRIP connection state machine states.
enum class NtripConnState : uint8_t {
    IDLE           = 0,   ///< Not started / idle
    CONNECTING     = 1,   ///< TCP connect in progress
    AUTHENTICATING = 2,   ///< HTTP request sent, waiting for response
    CONNECTED      = 3,   ///< Connected, receiving RTCM stream
    ERROR          = 4,   ///< Error state, will transition to IDLE for reconnect
    DISCONNECTED   = 5    ///< Clean disconnect, will reconnect after delay
};

/// NTRIP client configuration.
struct NtripConfig {
    char     host[64];          ///< NTRIP caster hostname or IP
    uint16_t port;              ///< NTRIP caster port
    char     mountpoint[48];    ///< NTRIP mountpoint (e.g. "VRS")
    char     user[32];          ///< NTRIP username (empty = no auth)
    char     password[32];      ///< NTRIP password
    uint32_t reconnect_delay_ms; ///< Delay before reconnect after error (default 5000)
};

/// NTRIP client runtime state and statistics.
struct NtripState {
    NtripConnState conn_state;       ///< Current state machine state
    uint32_t       state_enter_ms;   ///< millis() when current state was entered
    uint32_t       last_rtcm_ms;     ///< millis() of last RTCM data received
    uint32_t       rx_bytes;         ///< Total RTCM bytes received from caster
    uint32_t       forwarded_bytes;  ///< Total RTCM bytes forwarded to GNSS receivers
    uint32_t       connect_failures; ///< Count of failed connection attempts
    uint8_t        last_http_status; ///< Last HTTP response code (0 = no response)
    char           last_error[64];   ///< Last error description (empty = none)
};

/// GNSS receiver RTCM source type.
enum class RtcmSource : uint8_t {
    LOCAL = 0,   ///< Gets RTCM from local NTRIP client
    OWN   = 1    ///< Gets RTCM independently (e.g. remote receiver)
};

/// GNSS receiver configuration (compile-time, board-specific).
struct GnssRxConfig {
    uint8_t    uart_inst;        ///< UART instance (0=UART0, 1=UART1, 2=UART2)
    int8_t     tx_pin;           ///< UART TX GPIO
    int8_t     rx_pin;           ///< UART RX GPIO (-1 = unbound)
    uint32_t   baud;             ///< UART baud rate
    RtcmSource rtcm_source;      ///< Where this receiver gets RTCM from
};

/// Per-receiver runtime state.
struct GnssRxState {
    bool     ready;              ///< true after successful hal_gnss_uart_begin()
    uint32_t forwarded_bytes;    ///< Total RTCM bytes forwarded to this receiver
    uint32_t drop_count;         ///< Total bytes dropped (UART buffer full)
};

/// Maximum NTRIP RTCM buffer size (ring buffer for RTCM data).
static constexpr size_t NTRIP_RTCM_BUF_SIZE = 2048;

// ---------------------------------------------------------------------------
// Global instances — TASK-025
// ---------------------------------------------------------------------------
extern NtripConfig g_ntrip_config;
extern NtripState  g_ntrip;

// ---------------------------------------------------------------------------
// Scoped lock for arbitrary lock/unlock function pointers.
// ---------------------------------------------------------------------------
template <void (*LockFn)(), void (*UnlockFn)()>
class ScopedLock {
public:
    ScopedLock()  { LockFn(); }
    ~ScopedLock() { UnlockFn(); }
    ScopedLock(const ScopedLock&) = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;
};
