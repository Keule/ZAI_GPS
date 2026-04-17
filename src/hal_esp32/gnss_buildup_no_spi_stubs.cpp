#include "hal/hal.h"
#include "logic/sd_logger.h"
#include "logic/sd_ota.h"

#if defined(FEAT_GNSS_BUILDUP)

// ---------------------------------------------------------------------------
// IMU stubs (replace hal_bno085.cpp in GNSS buildup profile)
// ---------------------------------------------------------------------------
void hal_imu_begin(void) {}

bool hal_imu_read(float* yaw_rate_dps, float* roll_deg, float* heading_deg) {
    if (yaw_rate_dps) *yaw_rate_dps = 0.0f;
    if (roll_deg) *roll_deg = 0.0f;
    if (heading_deg) *heading_deg = 0.0f;
    return false;
}

bool hal_imu_detect(void) { return false; }

void hal_imu_reset_pulse(uint32_t low_ms, uint32_t settle_ms) {
    (void)low_ms;
    (void)settle_ms;
}

bool hal_imu_detect_boot_qualified(HalImuDetectStats* out) {
    if (out) {
        out->samples_total = 0;
        out->samples_ok = 0;
        out->samples_ff = 0;
        out->samples_zero = 0;
        out->last_response = 0;
    }
    return false;
}

void hal_imu_get_spi_info(HalImuSpiInfo* out) {
    if (out) {
        out->sck_pin = -1;
        out->miso_pin = -1;
        out->mosi_pin = -1;
        out->cs_pin = -1;
        out->int_pin = -1;
        out->freq_hz = 0;
        out->mode = 0;
    }
}

void hal_imu_set_spi_config(uint32_t freq_hz, uint8_t mode) {
    (void)freq_hz;
    (void)mode;
}

bool hal_imu_probe_once(uint8_t* out_response) {
    if (out_response) *out_response = 0;
    return false;
}

void hal_imu_on_sensor_spi_reinit(void) {}

// ---------------------------------------------------------------------------
// SD-OTA stubs (replace sd_ota_esp32.cpp in GNSS buildup profile)
// ---------------------------------------------------------------------------
bool isFirmwareUpdateAvailableOnSD(void) { return false; }

bool updateFirmwareFromSD(void) { return false; }

// ---------------------------------------------------------------------------
// SD logger stubs (replace sd_logger_esp32.cpp in GNSS buildup profile)
// ---------------------------------------------------------------------------
void sdLoggerInit(void) {}

void sdLoggerRecord(void) {}

bool sdLoggerIsActive(void) { return false; }

uint32_t sdLoggerGetRecordsFlushed(void) { return 0; }

uint32_t sdLoggerGetBufferCount(void) { return 0; }

bool hal_spi_busy(void) { return false; }

#endif  // FEAT_GNSS_BUILDUP
