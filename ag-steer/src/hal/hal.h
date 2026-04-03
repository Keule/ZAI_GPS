/**
 * @file hal.h
 * @brief Hardware Abstraction Layer – abstract interface for all HW access.
 *
 * This header contains ONLY declarations with C linkage.
 * Implementations live in hal_esp32/ or hal_pc/.
 *
 * No Arduino or ESP32 headers may be included here.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// ===================================================================
// System / Timing
// ===================================================================

/// Return milliseconds since boot (like Arduino millis()).
uint32_t hal_millis(void);

/// Return microseconds since boot.
uint32_t hal_micros(void);

/// Blocking delay in milliseconds.
void hal_delay_ms(uint32_t ms);

// ===================================================================
// Logging
// ===================================================================

/// Print a log message (printf-style on PC, Serial on ESP32).
void hal_log(const char* fmt, ...);

// ===================================================================
// Mutex (used by global_state.h)
// ===================================================================

void hal_mutex_init(void);
void hal_mutex_lock(void);
void hal_mutex_unlock(void);

// ===================================================================
// Safety input
// ===================================================================

/// Read safety input – true if OK (HIGH), false if KICK (LOW).
bool hal_safety_ok(void);

// ===================================================================
// GNSS UART
// ===================================================================

/// Initialise both GNSS UARTs.
void hal_gnss_init(void);

/// Read one line from GNSS MAIN UART (newline-terminated).
/// Returns true if a complete line was read into buf.
bool hal_gnss_main_read_line(char* buf, size_t max_len);

/// Read one line from GNSS HEADING UART (newline-terminated).
/// Returns true if a complete line was read into buf.
bool hal_gnss_heading_read_line(char* buf, size_t max_len);

/// Detect if GNSS main has received any data since init.
bool hal_gnss_main_detect(void);

/// Detect if GNSS heading has received any data since init.
bool hal_gnss_heading_detect(void);

/// Reset GNSS detection flags (call after detection check).
void hal_gnss_reset_detection(void);

// ===================================================================
// SPI Sensors / Actuator
// ===================================================================

/// Initialise SPI bus 2 (sensor bus) and all chip selects.
void hal_sensor_spi_init(void);

// --- IMU (BNO085) ---

/// Initialise IMU on SPI.
void hal_imu_begin(void);

/// Read yaw rate and roll from IMU. Returns true on success.
bool hal_imu_read(float* yaw_rate_dps, float* roll_deg);

/// Detect if IMU chip is present on SPI bus (call after hal_imu_begin).
/// Performs a chip ID read to verify hardware responds.
bool hal_imu_detect(void);

// --- Steering Angle Sensor ---

/// Initialise steering angle sensor on SPI.
void hal_steer_angle_begin(void);

/// Read current steering angle in degrees.
float hal_steer_angle_read_deg(void);

/// Detect if steer angle sensor hardware is present (call after hal_steer_angle_begin).
bool hal_steer_angle_detect(void);

// --- Actuator ---

/// Initialise actuator SPI interface.
void hal_actuator_begin(void);

/// Write a command value (0..65535) to the actuator.
void hal_actuator_write(uint16_t cmd);

/// Detect if actuator hardware is present (call after hal_actuator_begin).
bool hal_actuator_detect(void);

// ===================================================================
// Network (W5500 Ethernet)
// ===================================================================

/// Initialise W5500 Ethernet and configure IP.
void hal_net_init(void);

/// Send a UDP datagram to the configured AgIO endpoint.
/// @param data     payload bytes (caller must include AOG header + data + CRC)
/// @param len      total length of the datagram
/// @param port     destination UDP port (host byte order)
void hal_net_send(const uint8_t* data, size_t len, uint16_t port);

/// Poll for a received UDP datagram (non-blocking).
/// @param buf      buffer to receive into
/// @param max_len  buffer size
/// @param out_port receives the source UDP port (host byte order)
/// @return number of bytes received, or 0 if nothing available
int hal_net_receive(uint8_t* buf, size_t max_len, uint16_t* out_port);

/// Check if Ethernet link is up.
bool hal_net_is_connected(void);

/// Check if W5500 chip was detected during init.
bool hal_net_detected(void);

#ifdef __cplusplus
}
#endif
