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
// SPI Sensors / Actuator
// ===================================================================

/// Aggregated telemetry for the shared sensor SPI bus.
typedef struct {
    uint32_t window_ms;
    float bus_utilization_pct;
    uint32_t bus_transactions;
    uint32_t bus_busy_us;
    uint32_t imu_transactions;
    uint32_t was_transactions;
    uint32_t imu_deadline_miss;
    uint32_t was_deadline_miss;
} HalSpiTelemetry;

/// Static IMU SPI wiring/configuration info for diagnostics/bring-up logs.
typedef struct {
    int sck_pin;
    int miso_pin;
    int mosi_pin;
    int cs_pin;
    int int_pin;
    uint32_t freq_hz;
    uint8_t mode;
} HalImuSpiInfo;

/// Boot-time IMU detection statistics (multi-sample qualification).
typedef struct {
    uint16_t samples;
    uint16_t ok_count;
    uint16_t ff_count;
    uint16_t zero_count;
    uint16_t other_count;
    uint8_t last_response;
    bool present;
} HalImuDetectStats;

/// Initialise SPI bus 2 (sensor bus) and all chip selects.
void hal_sensor_spi_init(void);

/// Release SPI bus 2 (sensor bus).
/// Must be called before another peripheral (e.g. SD card) can use SPI2_HOST.
void hal_sensor_spi_deinit(void);

/// Re-initialise SPI bus 2 (sensor bus) after a temporary release.
/// Used to restore the sensor bus after an SD card operation.
void hal_sensor_spi_reinit(void);

/// Copy current SPI telemetry counters into @p out.
void hal_sensor_spi_get_telemetry(HalSpiTelemetry* out);

// --- IMU (BNO085) ---

/// Initialise IMU on SPI.
void hal_imu_begin(void);

/// Read yaw rate and roll from IMU. Returns true on success.
bool hal_imu_read(float* yaw_rate_dps, float* roll_deg);

/// Detect if IMU chip is present on SPI bus (call after hal_imu_begin).
/// Performs a chip ID read to verify hardware responds.
bool hal_imu_detect(void);

/// Boot-time qualified IMU detection.
/// Performs multiple samples and classifies responses.
bool hal_imu_detect_boot_qualified(HalImuDetectStats* out);

/// Return IMU SPI pin mapping + SPI settings used by HAL.
void hal_imu_get_spi_info(HalImuSpiInfo* out);

// --- Steering Angle Sensor ---

/// Initialise steering angle sensor on SPI.
void hal_steer_angle_begin(void);

/// Detect if steer angle sensor hardware is present (call after hal_steer_angle_begin).
bool hal_steer_angle_detect(void);

/// Run interactive steering angle calibration.
/// Prompts via Serial: move to left stop, then right stop.
/// Stores calibrated min/max raw ADC values in NVS (persistent).
/// Must be called AFTER hal_steer_angle_detect() succeeds.
void hal_steer_angle_calibrate(void);

/// Check if steering angle has a valid calibration stored.
/// Returns true if calibration was loaded from NVS.
bool hal_steer_angle_is_calibrated(void);

/// Read current steering angle in degrees.
/// Uses calibrated min/max to map ADC to -22.5°..+22.5°.
/// If not calibrated, returns 0.0.
float hal_steer_angle_read_deg(void);

/// Read raw 16-bit ADC value from ADS1118 (single sample, no median).
/// Returns 0 if not detected or not calibrated.
int16_t hal_steer_angle_read_raw(void);

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

/// Update the destination IP address for outgoing UDP datagrams.
/// Call this after receiving a Subnet Change (PGN 201) from AgIO.
void hal_net_set_dest_ip(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3);

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
