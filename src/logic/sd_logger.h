/**
 * @file sd_logger.h
 * @brief SD-Card Data Logger – public API.
 *
 * Records navigation/steering data to CSV files on an SD card.
 * Activated/deactivated via a hardware switch (GPIO 47, active LOW).
 *
 * Architecture:
 *   - Control loop calls sdLoggerRecord() to buffer one sample
 *   - A dedicated logger task (lowest priority) periodically drains
 *     the ring buffer and writes CSV records to the SD card
 *   - The SD card shares SPI2_HOST (FSPI) with the sensor bus,
 *     so the sensor SPI is temporarily released during writes
 *   - The hardware switch is checked before each flush cycle
 *
 * Usage:
 *   sdLoggerInit();   // call once in setup() after OTA check
 *   // In control loop:
 *   sdLoggerRecord(); // buffer one sample (subsampled internally)
 */

#pragma once

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// ===================================================================
// Log record – one sample of system state
// ===================================================================

/// A single log record – compact binary representation.
/// Written to ring buffer by the control loop, converted to CSV
/// by the logger task when flushing to SD card.
typedef struct {
    uint32_t timestamp_ms;   ///< milliseconds since boot
    float   heading_deg;     ///< heading [degrees, 0-360]
    float   steer_angle_deg; ///< measured steering angle [degrees]
    float   desired_angle_deg; ///< desired steering angle (setpoint) [degrees]
    float   yaw_rate_dps;    ///< yaw rate from IMU [deg/s]
    float   roll_deg;        ///< roll angle [degrees]
    uint8_t safety_ok;       ///< 1 = safety circuit OK, 0 = KICK
} SdLogRecord;

// ===================================================================
// Public API
// ===================================================================

/// Initialise the SD logger.
///
/// Sets up GPIO 47 as input with pull-up (logging switch),
/// creates the logger FreeRTOS task (lowest priority).
///
/// Must be called AFTER hal_esp32_init_all() but BEFORE
/// creating the control/comm tasks (so the logger can run
/// at a known priority level).
void sdLoggerInit(void);

/// Execute one maintenance logger cycle (called from maintTask).
void sdLoggerMaintTick(void);

/// Buffer one log record.
///
/// Called from the control loop (200 Hz). Internally subsamples
/// to the configured log rate (default: 10 Hz = every 20th call).
/// The function is very fast (~1 µs) – just a memcpy into the
/// ring buffer.
///
/// If the ring buffer is full, the oldest record is silently
/// overwritten (ring buffer wrap-around).
///
/// Safe to call even if logging is disabled – it will be a no-op
/// after checking the switch state (first call in a subsample group).
void sdLoggerRecord(void);

/// Check if logging is currently active (switch is ON).
///
/// Returns true if GPIO 47 is LOW (switch closed to GND).
bool sdLoggerIsActive(void);

/// Get the number of records written to SD card since boot.
/// Useful for status reporting.
uint32_t sdLoggerGetRecordsFlushed(void);

/// Get the number of records currently in the ring buffer
/// (not yet flushed to SD).
uint32_t sdLoggerGetBufferCount(void);

#ifdef __cplusplus
}
#endif
