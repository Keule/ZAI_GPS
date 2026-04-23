/**
 * @file imu.cpp
 * @brief IMU runtime glue for the BNO085 HAL.
 */

#include "imu.h"
#include "dependency_policy.h"
#include "global_state.h"
#include "hal/hal.h"

#include "log_config.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_IMU
#include "esp_log.h"
#include "log_ext.h"

void imuInit(void) {
    hal_imu_begin();
    LOGI("IMU", "initialised (BNO085 SPI)");
}

bool imuUpdate(void) {
    float yaw_rate = 0.0f;
    float roll = 0.0f;
    float heading = 9999.0f;

    if (!hal_imu_read(&yaw_rate, &roll, &heading)) {
        StateLock lock;
        g_nav.imu.imu_quality_ok = false;
        g_nav.imu.heading_quality_ok = false;
        return false;
    }

    const uint32_t now_ms = hal_millis();
    const bool plausible = dep_policy::isImuPlausible(yaw_rate, roll);
    const bool heading_plausible = dep_policy::isHeadingPlausible(heading);

    {
        StateLock lock;
        if (heading_plausible) {
            g_nav.imu.heading_deg = heading;
            g_nav.imu.heading_timestamp_ms = now_ms;
        }
        g_nav.imu.yaw_rate_dps = yaw_rate;
        g_nav.imu.roll_deg = roll;
        g_nav.imu.imu_timestamp_ms = now_ms;
        g_nav.imu.imu_quality_ok = plausible;
        g_nav.imu.heading_quality_ok = heading_plausible;
    }

    return plausible;
}

bool imuIsHealthy(uint32_t now_ms) {
    StateLock lock;
    if (!g_nav.imu.imu_quality_ok) return false;
    return dep_policy::isFresh(now_ms,
                               g_nav.imu.imu_timestamp_ms,
                               dep_policy::IMU_FRESHNESS_TIMEOUT_MS);
}

void imuBringupInit(void) {
    HalImuSpiInfo spi = {};
    hal_imu_get_spi_info(&spi);

    hal_log("IMU-BRINGUP: mode=ON imu_spi pins[sck=%d miso=%d mosi=%d cs=%d int=%d] params[freq=%luHz mode=%u] intervals[detect=%lums read=%lums]",
            spi.sck_pin, spi.miso_pin, spi.mosi_pin, spi.cs_pin, spi.int_pin,
            (unsigned long)spi.freq_hz, (unsigned)spi.mode,
            (unsigned long)k_detect_interval_ms, (unsigned long)k_read_interval_ms);

    s_last_detect_ms = 0;
    s_last_read_ms = 0;
    s_last_ads_ms = 0;
    s_last_stats_ms = 0;
    s_read_ok = 0;
    s_read_fail = 0;
    s_last_detect_ok = false;
    s_prev_detect_ok = false;
    s_has_prev_detect = false;
    s_phase_index = 0;
    s_matrix_done = false;
    applyCase(s_phase_index);

    hal_log("IMU-BRINGUP: matrix cases=%u phase_ms=%lu detect_ms=%lu read_ms=%lu ads_ms=%lu",
            (unsigned)(sizeof(k_cases) / sizeof(k_cases[0])),
            (unsigned long)k_phase_duration_ms,
            (unsigned long)k_detect_interval_ms,
            (unsigned long)k_read_interval_ms,
            (unsigned long)k_ads_interval_ms);
}

namespace {
bool imu_enabled_check() {
    return feat::imu();
}

bool imu_health_check(uint32_t now_ms) {
    return imuIsHealthy(now_ms);
}

namespace {
bool imu_enabled_check() {
    return feat::imu();
}

bool imu_health_check(uint32_t now_ms) {
    return imuIsHealthy(now_ms);
}
}  // namespace

const ModuleOps imu_ops = {
    "IMU",
    imu_enabled_check,
    imuInit,
    imuUpdate,
    imu_health_check
};
