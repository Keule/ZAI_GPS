/**
 * @file imu.cpp
 * @brief IMU driver implementation (stub for BNO085 SPI).
 *
 * TODO: Implement full BNO085 SH-2 / sensor hub protocol over SPI.
 * Currently reads via HAL stub (PC sim returns dummy values).
 */

#include "imu.h"
#include "dependency_policy.h"
#include "global_state.h"
#include "hal/hal.h"

#include "log_config.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_IMU
#include "esp_log.h"
#include "log_ext.h"

namespace {
#if defined(FEAT_IMU_BRINGUP)
constexpr bool k_imu_bringup_mode = true;
#else
constexpr bool k_imu_bringup_mode = false;
#endif

constexpr uint32_t k_detect_interval_ms = 1000;
constexpr uint32_t k_read_interval_ms = 100;
constexpr uint32_t k_ads_interval_ms = 1000;
constexpr uint32_t k_phase_duration_ms = 6000;
constexpr uint8_t k_spi_mode_0 = 0;
constexpr uint8_t k_spi_mode_3 = 3;
uint32_t s_last_detect_ms = 0;
uint32_t s_last_read_ms = 0;
uint32_t s_last_ads_ms = 0;
uint32_t s_read_ok = 0;
uint32_t s_read_fail = 0;
bool s_last_detect_ok = false;

struct BringupModeCase {
    uint32_t freq_hz;
    uint8_t mode;
    bool ads_ping;
    const char* label;
};

constexpr BringupModeCase k_cases[] = {
    {100000,  k_spi_mode_3, false, "m3_100k_imu_only"},
    {500000,  k_spi_mode_3, false, "m3_500k_imu_only"},
    {1000000, k_spi_mode_3, false, "m3_1m_imu_only"},
    {100000,  k_spi_mode_0, false, "m0_100k_imu_only"},
    {1000000, k_spi_mode_0, false, "m0_1m_imu_only"},
    {1000000, k_spi_mode_3, true,  "m3_1m_with_ads"},
};

struct BringupStats {
    uint32_t probes = 0;
    uint32_t probe_ok = 0;
    uint32_t probe_ff = 0;
    uint32_t probe_zero = 0;
    uint32_t probe_other = 0;
    uint8_t last_rsp = 0x00;
    uint32_t ads_reads = 0;
    int16_t ads_last = 0;
    int16_t ads_min = 0;
    int16_t ads_max = 0;
    bool ads_seen = false;
};

BringupStats s_phase_stats = {};
uint32_t s_phase_start_ms = 0;
uint8_t s_phase_index = 0;
bool s_matrix_done = false;

void resetPhaseStats() {
    s_phase_stats = {};
}

void applyCase(uint8_t idx) {
    if (idx >= (sizeof(k_cases) / sizeof(k_cases[0]))) return;
    const BringupModeCase& c = k_cases[idx];
    hal_imu_set_spi_config(c.freq_hz, c.mode);
    resetPhaseStats();
    s_phase_start_ms = hal_millis();
    hal_log("IMU-BRINGUP: phase_start idx=%u/%u label=%s imu_spi[freq=%lu mode=%u] ads_ping=%s",
            (unsigned)idx + 1u,
            (unsigned)(sizeof(k_cases) / sizeof(k_cases[0])),
            c.label,
            (unsigned long)c.freq_hz,
            (unsigned)c.mode,
            c.ads_ping ? "ON" : "OFF");
}

void logPhaseSummary(uint8_t idx) {
    if (idx >= (sizeof(k_cases) / sizeof(k_cases[0]))) return;
    const BringupModeCase& c = k_cases[idx];
    hal_log("IMU-BRINGUP: phase_done idx=%u label=%s probes=%lu ok=%lu ff=%lu zero=%lu other=%lu last=0x%02X reads_ok=%lu reads_fail=%lu ads_reads=%lu ads_last=%d ads_min=%d ads_max=%d",
            (unsigned)idx + 1u,
            c.label,
            (unsigned long)s_phase_stats.probes,
            (unsigned long)s_phase_stats.probe_ok,
            (unsigned long)s_phase_stats.probe_ff,
            (unsigned long)s_phase_stats.probe_zero,
            (unsigned long)s_phase_stats.probe_other,
            (unsigned)s_phase_stats.last_rsp,
            (unsigned long)s_read_ok,
            (unsigned long)s_read_fail,
            (unsigned long)s_phase_stats.ads_reads,
            (int)s_phase_stats.ads_last,
            s_phase_stats.ads_seen ? (int)s_phase_stats.ads_min : 0,
            s_phase_stats.ads_seen ? (int)s_phase_stats.ads_max : 0);
}
}  // namespace

bool imuBringupModeEnabled(void) {
    return k_imu_bringup_mode;
}

void imuInit(void) {
    hal_imu_begin();
    LOGI("IMU", "initialised (BNO085 SPI stub)");
}

bool imuUpdate(void) {
    float yaw_rate = 0.0f;
    float roll = 0.0f;

    if (!hal_imu_read(&yaw_rate, &roll)) {
        StateLock lock;
        g_nav.imu_quality_ok = false;
        return false;
    }

    const uint32_t now_ms = hal_millis();
    const bool plausible = dep_policy::isImuPlausible(yaw_rate, roll);

    {
        StateLock lock;
        g_nav.yaw_rate_dps = yaw_rate;
        g_nav.roll_deg = roll;
        g_nav.imu_timestamp_ms = now_ms;
        g_nav.imu_quality_ok = plausible;
        g_nav.timestamp_ms = now_ms;
    }

    return plausible;
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
    s_read_ok = 0;
    s_read_fail = 0;
    s_last_detect_ok = false;
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

void imuBringupTick(void) {
    const uint32_t now_ms = hal_millis();
    const BringupModeCase& c = k_cases[s_phase_index];

    if (!s_matrix_done && now_ms - s_phase_start_ms >= k_phase_duration_ms) {
        logPhaseSummary(s_phase_index);
        s_phase_index++;
        if (s_phase_index >= (sizeof(k_cases) / sizeof(k_cases[0]))) {
            s_matrix_done = true;
            s_phase_index = (sizeof(k_cases) / sizeof(k_cases[0])) - 1;
            hal_log("IMU-BRINGUP: matrix_done: holding last phase config for ongoing monitoring");
        } else {
            applyCase(s_phase_index);
        }
    }

    if (now_ms - s_last_detect_ms >= k_detect_interval_ms) {
        s_last_detect_ms = now_ms;
        uint8_t rsp = 0;
        hal_imu_probe_once(&rsp);
        s_phase_stats.probes++;
        s_phase_stats.last_rsp = rsp;
        if (rsp == 0xFF) {
            s_phase_stats.probe_ff++;
            s_last_detect_ok = false;
        } else if (rsp == 0x00) {
            s_phase_stats.probe_zero++;
            s_last_detect_ok = false;
        } else {
            s_phase_stats.probe_ok++;
            s_phase_stats.probe_other++;
            s_last_detect_ok = true;
        }
        hal_log("IMU-BRINGUP: probe=%s rsp=0x%02X phase=%u label=%s",
                s_last_detect_ok ? "OK" : "FAIL",
                (unsigned)rsp,
                (unsigned)s_phase_index + 1u,
                c.label);
    }

    if (now_ms - s_last_read_ms >= k_read_interval_ms) {
        s_last_read_ms = now_ms;
        float yaw_rate = 0.0f;
        float roll = 0.0f;
        const bool ok = hal_imu_read(&yaw_rate, &roll);
        if (ok) {
            s_read_ok++;
        } else {
            s_read_fail++;
        }

        hal_log("IMU-BRINGUP: read=%s yaw=%.3f roll=%.3f cnt_ok=%lu cnt_fail=%lu detect=%s",
                ok ? "OK" : "FAIL",
                yaw_rate,
                roll,
                (unsigned long)s_read_ok,
                (unsigned long)s_read_fail,
                s_last_detect_ok ? "OK" : "FAIL");
    }

    if (c.ads_ping && now_ms - s_last_ads_ms >= k_ads_interval_ms) {
        s_last_ads_ms = now_ms;
        const int16_t raw = hal_steer_angle_read_raw();
        s_phase_stats.ads_reads++;
        s_phase_stats.ads_last = raw;
        if (!s_phase_stats.ads_seen) {
            s_phase_stats.ads_seen = true;
            s_phase_stats.ads_min = raw;
            s_phase_stats.ads_max = raw;
        } else {
            if (raw < s_phase_stats.ads_min) s_phase_stats.ads_min = raw;
            if (raw > s_phase_stats.ads_max) s_phase_stats.ads_max = raw;
        }
        hal_log("IMU-BRINGUP: ads_ping raw=%d min=%d max=%d reads=%lu",
                (int)raw,
                (int)s_phase_stats.ads_min,
                (int)s_phase_stats.ads_max,
                (unsigned long)s_phase_stats.ads_reads);
    }
}
