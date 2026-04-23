#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdarg>

#include "logic/modules.h"
#include "hal/hal.h"

extern "C" {

inline void hal_mutex_init(void) {}
inline void hal_mutex_lock(void) {}
inline void hal_mutex_unlock(void) {}

inline uint32_t g_mock_millis = 0;
inline uint32_t g_mock_micros = 0;
inline void hal_mock_set_millis(uint32_t ms) { g_mock_millis = ms; g_mock_micros = ms * 1000U; }
inline uint32_t hal_millis(void) { return g_mock_millis; }
inline uint32_t hal_micros(void) { return g_mock_micros; }
inline void hal_delay_ms(uint32_t ms) { g_mock_millis += ms; g_mock_micros += (ms * 1000U); }

inline void hal_log(const char* /*fmt*/, ...) {}

inline bool g_mock_safety_ok = true;
inline void hal_mock_set_safety(bool ok) { g_mock_safety_ok = ok; }
inline bool hal_safety_ok(void) { return g_mock_safety_ok; }

inline bool hal_sd_card_present(void) { return false; }
inline void hal_sensor_spi_init(void) {}
inline void hal_sensor_spi_deinit(void) {}
inline void hal_sensor_spi_reinit(void) {}
inline void hal_steer_angle_begin(void) {}
inline float g_mock_was_angle = 0.0f;
inline int16_t g_mock_was_raw = 0;
inline void hal_mock_set_was(float angle_deg, int16_t raw) { g_mock_was_angle = angle_deg; g_mock_was_raw = raw; }
inline float hal_steer_angle_read_deg(void) { return g_mock_was_angle; }
inline int16_t hal_steer_angle_read_raw(void) { return g_mock_was_raw; }
inline uint8_t hal_steer_angle_read_sensor_byte(void) { return 0; }
inline bool hal_steer_angle_load_calibration(int16_t* zero, int16_t* left, int16_t* right) {
    if (zero) *zero = 0;
    if (left) *left = -1000;
    if (right) *right = 1000;
    return true;
}
inline bool hal_steer_angle_save_calibration(int16_t, int16_t, int16_t) { return true; }
inline void hal_steer_angle_set_calibration(int16_t, int16_t, int16_t) {}
inline bool hal_steer_angle_is_calibrated(void) { return true; }

inline void hal_actuator_begin(void) {}
inline uint16_t g_mock_actuator_cmd = 0;
inline void hal_actuator_write(uint16_t cmd) { g_mock_actuator_cmd = cmd; }
inline uint16_t hal_mock_get_actuator_cmd(void) { return g_mock_actuator_cmd; }

inline void hal_imu_begin(void) {}
inline float g_mock_imu_yaw = 0.0f;
inline float g_mock_imu_roll = 0.0f;
inline float g_mock_imu_heading = 0.0f;
inline bool g_mock_imu_ok = true;
inline void hal_mock_set_imu(float yaw, float roll, float heading, bool ok) {
    g_mock_imu_yaw = yaw;
    g_mock_imu_roll = roll;
    g_mock_imu_heading = heading;
    g_mock_imu_ok = ok;
}
inline bool hal_imu_read(float* yaw_rate_dps, float* roll_deg, float* heading_deg) {
    if (yaw_rate_dps) *yaw_rate_dps = g_mock_imu_yaw;
    if (roll_deg) *roll_deg = g_mock_imu_roll;
    if (heading_deg) *heading_deg = g_mock_imu_heading;
    return g_mock_imu_ok;
}
inline bool hal_imu_detect(void) { return g_mock_imu_ok; }
inline void hal_imu_reset_pulse(uint32_t, uint32_t) {}
inline bool hal_imu_detect_boot_qualified(HalImuDetectStats* out) { if (out) *out = {}; return g_mock_imu_ok; }
inline void hal_imu_set_spi_config(uint32_t, uint8_t) {}
inline bool hal_imu_probe_once(uint8_t* rsp) { if (rsp) *rsp = 0xA0; return true; }
inline void hal_imu_get_spi_info(HalImuSpiInfo* out) { if (out) *out = {}; }

inline void hal_net_init(void) {}
inline bool hal_net_is_connected(void) { return false; }
inline void hal_net_set_dest_ip(uint8_t, uint8_t, uint8_t, uint8_t) {}
inline void hal_net_send(const uint8_t*, size_t, uint16_t) {}
inline int hal_net_receive(uint8_t*, size_t, uint16_t*) { return 0; }
inline int hal_net_receive_rtcm(uint8_t*, size_t, uint16_t*) { return 0; }

inline bool hal_gnss_rtcm_begin(uint32_t, int8_t, int8_t) { return true; }
inline size_t hal_gnss_rtcm_write(const uint8_t*, size_t len) { return len; }
inline bool hal_gnss_rtcm_is_ready(void) { return true; }
inline uint32_t hal_gnss_rtcm_drop_count(void) { return 0; }

inline bool hal_gnss_uart_begin(uint8_t, uint32_t, int8_t, int8_t) { return true; }
inline size_t hal_gnss_uart_write(uint8_t, const uint8_t*, size_t len) { return len; }
inline bool hal_gnss_uart_is_ready(uint8_t) { return true; }
inline uint32_t hal_gnss_uart_drop_count(uint8_t) { return 0; }

inline bool hal_pin_claim_add(int, const char*) { return true; }
inline int hal_pin_claim_release(const char*) { return 0; }
inline bool hal_pin_claim_check(int) { return false; }
inline const char* hal_pin_claim_owner(int) { return ""; }

} // extern "C"

inline bool moduleIsActive(FirmwareFeatureId /*id*/) {
    return true;
}
