/**
 * @file hal_impl.cpp
 * @brief PC / Linux simulation HAL implementation.
 *
 * Provides dummy sensor values, printf logging, and simulated timing.
 * Used for testing the logic layer without real hardware.
 */

#include "hal_impl.h"
#include "hal/hal.h"
#include "logic/aog_udp_protocol.h"

#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <cmath>
#include <chrono>
#include <mutex>
#include <thread>

// ===================================================================
// Timing – using std::chrono
// ===================================================================
static auto s_start_time = std::chrono::steady_clock::now();

uint32_t hal_millis(void) {
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - s_start_time);
    return static_cast<uint32_t>(ms.count());
}

uint32_t hal_micros(void) {
    auto now = std::chrono::steady_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(now - s_start_time);
    return static_cast<uint32_t>(us.count());
}

void hal_delay_ms(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// ===================================================================
// Logging – printf
// ===================================================================
void hal_log(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    std::printf("[%10lu] %s\n", (unsigned long)hal_millis(), buf);
    std::fflush(stdout);
}

// ===================================================================
// Mutex
// ===================================================================
static std::mutex s_global_mutex;

void hal_mutex_init(void) {
    // std::mutex is already initialised
}

void hal_mutex_lock(void) {
    s_global_mutex.lock();
}

void hal_mutex_unlock(void) {
    s_global_mutex.unlock();
}

// ===================================================================
// Safety input – always OK in simulation
// ===================================================================
bool hal_safety_ok(void) {
    return true;
}

// ===================================================================
// GNSS UART – simulated NMEA output
// ===================================================================
static uint32_t s_gnss_main_tick = 0;
static uint32_t s_gnss_heading_tick = 0;

void hal_gnss_init(void) {
    s_gnss_main_tick = 0;
    s_gnss_heading_tick = 0;
}

bool hal_gnss_main_read_line(char* buf, size_t max_len) {
    // Simulate: emit one NMEA sentence per call cycle.
    // Return true on odd calls (GGA), false on even calls (no line available).
    // This prevents infinite spin in the gnssPollMain while-loop.
    s_gnss_main_tick++;

    if (s_gnss_main_tick % 2 == 1) {
        // GGA: position near Berlin
        std::snprintf(buf, max_len,
            "$GNGGA,120000.00,5231.0000,N,01324.0000,E,4,12,0.9,45.2,M,0.0,M,,*00");
        return true;
    } else if (s_gnss_main_tick % 4 == 0) {
        // RMC: moving at ~5 m/s, heading 90 degrees
        std::snprintf(buf, max_len,
            "$GNRMC,120000.00,A,5231.0000,N,01324.0000,E,9.72,90.0,010125,,,A*00");
        return true;
    }
    return false;
}

bool hal_gnss_heading_read_line(char* buf, size_t max_len) {
    s_gnss_heading_tick++;

    if (s_gnss_heading_tick % 3 == 1) {
        // GGA from heading antenna
        std::snprintf(buf, max_len,
            "$GNGGA,120000.00,5231.0010,N,01324.0010,E,4,10,1.0,45.3,M,0.0,M,,*00");
        return true;
    } else if (s_gnss_heading_tick % 3 == 2) {
        // RMC with heading
        std::snprintf(buf, max_len,
            "$GNRMC,120000.00,A,5231.0010,N,01324.0010,E,9.72,91.5,010125,,,A*00");
        return true;
    }
    return false;
}

// ===================================================================
// SPI Sensors / Actuator – simulated values
// ===================================================================
static float s_sim_steer_angle = 0.0f;
// Simulated state (updated dynamically in hal_imu_read)
static uint16_t s_last_actuator_cmd = 0;

void hal_sensor_spi_init(void) {
    hal_log("PC HAL: sensor SPI initialised (simulated)");
}

void hal_imu_begin(void) {
    hal_log("PC HAL: IMU begun (simulated)");
}

bool hal_imu_read(float* yaw_rate_dps, float* roll_deg) {
    // Simulated: small yaw rate + slight roll
    float t = hal_millis() * 0.001f;
    *yaw_rate_dps = 2.0f * std::sinf(t * 0.3f);  // oscillating yaw rate
    *roll_deg = 1.5f * std::sinf(t * 0.2f);       // oscillating roll
    return true;
}

void hal_steer_angle_begin(void) {
    hal_log("PC HAL: steer angle sensor begun (simulated)");
}

float hal_steer_angle_read_deg(void) {
    // Simulate: slowly move toward the last actuator command direction
    float target = (s_last_actuator_cmd / 65535.0f) * 45.0f - 22.5f;
    s_sim_steer_angle += (target - s_sim_steer_angle) * 0.1f;
    return s_sim_steer_angle;
}

void hal_actuator_begin(void) {
    hal_log("PC HAL: actuator begun (simulated)");
}

void hal_actuator_write(uint16_t cmd) {
    s_last_actuator_cmd = cmd;
    hal_log("PC HAL: actuator cmd = %u", cmd);
}

// ===================================================================
// Network – stub (no real Ethernet on PC, just log + echo)
// ===================================================================
// Net RX buffers (for future simulation of incoming frames)
// static uint8_t s_net_rx_buffer[AOG_MAX_FRAME];
// static size_t s_net_rx_len = 0;

void hal_net_init(void) {
    hal_log("PC HAL: network initialised (simulated – no real Ethernet)");
}

void hal_net_send(const uint8_t* data, size_t len, uint16_t port) {
    // In simulation, just log the send
    // Real implementation would use sendto() or Ethernet library
    (void)port;
    // Already logged via aogHexDump in net.cpp
}

int hal_net_receive(uint8_t* buf, size_t max_len, uint16_t* out_port) {
    // No incoming data in simulation
    (void)buf; (void)max_len; (void)out_port;
    return 0;
}

bool hal_net_is_connected(void) {
    return true;
}

// ===================================================================
// PC init all
// ===================================================================
void hal_pc_init_all(void) {
    hal_mutex_init();
    hal_gnss_init();
    hal_sensor_spi_init();
    hal_net_init();
    hal_log("PC HAL: all subsystems initialised");
}
