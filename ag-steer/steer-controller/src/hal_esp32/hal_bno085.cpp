/**
 * @file hal_bno085.cpp
 * @brief BNO085 HAL binding using the 7Semi BNO08x SPI example pattern.
 */

#include "hal/hal.h"
#include "hardware_pins.h"

#include <7Semi_BNO08x.h>
#include <Arduino.h>
#include <BnoSPIBus.h>
#include <SPI.h>

SPIClass& hal_esp32_sensor_spi_port(void);

namespace {

constexpr uint32_t kBno085SpiHz = 3000000UL;
constexpr uint16_t kBno085PeriodMs = 20;
constexpr float kRadToDeg = 57.2957795f;

BnoSPIBus* s_bno_bus = nullptr;
BNO08x_7Semi* s_bno08x = nullptr;

bool s_bno08x_begin_attempted = false;
bool s_bno08x_ready = false;
bool s_reports_enabled = false;
bool s_imu_cache_valid = false;
float s_imu_yaw_cache = 0.0f;
float s_imu_roll_cache = 0.0f;
float s_imu_acc_x_cache = 0.0f;
float s_imu_acc_y_cache = 0.0f;
float s_imu_acc_z_cache = 0.0f;
uint32_t s_imu_last_data_us = 0;
uint32_t s_last_diag_log_ms = 0;
uint32_t s_diag_read_ok = 0;
uint32_t s_diag_read_wait = 0;
uint32_t s_spi_recover_count = 0;
uint32_t s_diag_packet_count = 0;

void prepareChipSelects() {
    pinMode(CS_STEER_ANG, OUTPUT);
    digitalWrite(CS_STEER_ANG, HIGH);
    pinMode(CS_ACT, OUTPUT);
    digitalWrite(CS_ACT, HIGH);
    pinMode(CS_IMU, OUTPUT);
    digitalWrite(CS_IMU, HIGH);

    pinMode(IMU_WAKE, OUTPUT);
    digitalWrite(IMU_WAKE, HIGH);
}

bool enableRuntimeReports() {
    if (!s_bno08x) return false;

    for (uint8_t i = 0; i < 5; i++) {
        s_bno08x->processData();
        delay(10);
    }

    const bool acc_ok = s_bno08x->enableAcc(kBno085PeriodMs);
    const bool gyro_ok = s_bno08x->enableGyro(kBno085PeriodMs);
    const bool mag_ok = s_bno08x->enableMag(50);
    const bool rot_ok = s_bno08x->enableRotationVector(kBno085PeriodMs);
    const bool lin_ok = s_bno08x->enableLinearAccel(kBno085PeriodMs);

    s_reports_enabled = true;
    s_imu_cache_valid = false;
    s_imu_last_data_us = 0;
    hal_log("ESP32: 7Semi BNO085 reports requested acc=%s gyro=%s mag=%s rotation=%s linear=%s poll=%s",
            acc_ok ? "OK" : "FAIL",
            gyro_ok ? "OK" : "FAIL",
            mag_ok ? "OK" : "FAIL",
            rot_ok ? "OK" : "FAIL",
            lin_ok ? "OK" : "FAIL",
            s_reports_enabled ? "ON" : "OFF");
    return s_reports_enabled;
}

void recoverAfterSpiReinit() {
    if (!s_bno08x_begin_attempted || !s_bno08x) return;

    prepareChipSelects();
    delay(20);

    s_bno08x_ready = s_bno08x->begin();
    s_reports_enabled = false;
    s_imu_cache_valid = false;
    s_imu_last_data_us = 0;
    s_spi_recover_count++;

    hal_log("ESP32: 7Semi BNO085 SPI recovery #%lu begin %s (levels[int=%d rst=%d wake=%d])",
            (unsigned long)s_spi_recover_count,
            s_bno08x_ready ? "OK" : "FAIL",
            digitalRead(IMU_INT),
            digitalRead(IMU_RST),
            digitalRead(IMU_WAKE));

    if (s_bno08x_ready) {
        enableRuntimeReports();
    }
}

float rollDegFromQuaternion(float i, float j, float k, float real) {
    const float norm = sqrtf((real * real) + (i * i) + (j * j) + (k * k));
    if (norm <= 0.0f) return 0.0f;

    real /= norm;
    i /= norm;
    j /= norm;
    k /= norm;

    const float t0 = 2.0f * ((real * i) + (j * k));
    const float t1 = 1.0f - (2.0f * ((i * i) + (j * j)));
    return atan2f(t0, t1) * kRadToDeg;
}

} // namespace

void hal_imu_begin(void) {
    if (s_bno08x_begin_attempted) {
        return;
    }
    s_bno08x_begin_attempted = true;

    prepareChipSelects();
    hal_imu_set_spi_config(kBno085SpiHz, SPI_MODE3);

    SPIClass& imu_spi = hal_esp32_sensor_spi_port();
    static BnoSPIBus bus(
        imu_spi,
        CS_IMU,
        IMU_INT,
        IMU_RST,
        kBno085SpiHz,
        SPI_MODE3,
        SENS_SPI_SCK,
        SENS_SPI_MISO,
        SENS_SPI_MOSI);
    static BNO08x_7Semi bno(bus);

    s_bno_bus = &bus;
    s_bno08x = &bno;

    s_bno08x_ready = s_bno08x->begin();
    hal_log("ESP32: 7Semi BNO085 example-style begin %s (SCK=%d MISO=%d MOSI=%d CS=%d INT=%d RST=%d WAKE/PS0=%d freq=%luHz mode=3 levels[int=%d rst=%d wake=%d])",
            s_bno08x_ready ? "OK" : "FAIL",
            SENS_SPI_SCK,
            SENS_SPI_MISO,
            SENS_SPI_MOSI,
            CS_IMU,
            IMU_INT,
            IMU_RST,
            IMU_WAKE,
            (unsigned long)kBno085SpiHz,
            digitalRead(IMU_INT),
            digitalRead(IMU_RST),
            digitalRead(IMU_WAKE));

    if (s_bno08x_ready) {
        enableRuntimeReports();
    }
}

void hal_imu_reset_pulse(uint32_t low_ms, uint32_t settle_ms) {
    prepareChipSelects();
    pinMode(IMU_RST, OUTPUT);
    pinMode(IMU_INT, INPUT_PULLUP);

    const int int_before = digitalRead(IMU_INT);
    const int wake_before = digitalRead(IMU_WAKE);
    digitalWrite(IMU_RST, LOW);
    delay(low_ms);
    const int int_during = digitalRead(IMU_INT);
    const int rst_during = digitalRead(IMU_RST);
    digitalWrite(IMU_RST, HIGH);
    delay(settle_ms);
    const int int_after = digitalRead(IMU_INT);
    const int rst_after = digitalRead(IMU_RST);
    const int wake_after = digitalRead(IMU_WAKE);
    hal_log("ESP32: IMU reset pulse low=%lums settle=%lums INT(before=%d during=%d after=%d) RST(during=%d after=%d) WAKE/PS0(before=%d after=%d)",
            (unsigned long)low_ms,
            (unsigned long)settle_ms,
            int_before, int_during, int_after,
            rst_during, rst_after,
            wake_before, wake_after);
}

bool hal_imu_read(float* yaw_rate_dps, float* roll_deg) {
    if (!yaw_rate_dps || !roll_deg) return false;
    if (!s_bno08x_ready || !s_reports_enabled || !s_bno08x) return false;

    bool got_update = false;
    const bool int_asserted = digitalRead(IMU_INT) == LOW;
    if (int_asserted) {
        for (uint8_t i = 0; i < 4 && digitalRead(IMU_INT) == LOW; i++) {
            uint8_t pkt[256] = {};
            const int n = s_bno08x->readPacket(pkt, sizeof(pkt));
            if (n <= 0) {
                break;
            }
            s_diag_packet_count++;
            s_bno08x->processPacket(pkt, static_cast<size_t>(n));
        }
    }

    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    if (s_bno08x->getAccelerometer(ax, ay, az)) {
        s_imu_acc_x_cache = ax;
        s_imu_acc_y_cache = ay;
        s_imu_acc_z_cache = az;
        got_update = true;
    }

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    if (s_bno08x->getGyroscope(x, y, z)) {
        s_imu_yaw_cache = z * kRadToDeg;
        got_update = true;
    }

    float qi = 0.0f;
    float qj = 0.0f;
    float qk = 0.0f;
    float qr = 1.0f;
    if (s_bno08x->getQuaternion(qi, qj, qk, qr)) {
        s_imu_roll_cache = rollDegFromQuaternion(qi, qj, qk, qr);
        got_update = true;
    }

    const uint32_t now_us = micros();
    if (got_update) {
        s_imu_last_data_us = now_us;
        s_imu_cache_valid = true;
        s_diag_read_ok++;
    } else {
        s_diag_read_wait++;
    }

    const uint32_t now_ms = millis();
    if ((now_ms - s_last_diag_log_ms) >= 500UL) {
        s_last_diag_log_ms = now_ms;
        const uint32_t age_ms = s_imu_last_data_us == 0 ? 0 : ((now_us - s_imu_last_data_us) / 1000UL);
        hal_log("IMU-DIAG: 7semi-example poll=%s int=%d data=%s age=%lums pkt=%lu acc=[%.2f %.2f %.2f] mps2 yaw_rate=%.2f dps roll=%.2f deg ok=%lu wait=%lu",
                s_reports_enabled ? "ON" : "OFF",
                digitalRead(IMU_INT),
                got_update ? "NEW" : (s_imu_cache_valid ? "CACHE" : "WAIT"),
                (unsigned long)age_ms,
                (unsigned long)s_diag_packet_count,
                s_imu_acc_x_cache,
                s_imu_acc_y_cache,
                s_imu_acc_z_cache,
                s_imu_yaw_cache,
                s_imu_roll_cache,
                (unsigned long)s_diag_read_ok,
                (unsigned long)s_diag_read_wait);
    }

    if (!s_imu_cache_valid || s_imu_last_data_us == 0 || (now_us - s_imu_last_data_us) > 50000UL) {
        return false;
    }

    *yaw_rate_dps = s_imu_yaw_cache;
    *roll_deg = s_imu_roll_cache;
    return true;
}

bool hal_imu_detect(void) {
    hal_log("ESP32: IMU detect via 7Semi BNO085 example SPI: lib=%s reports=%s",
            s_bno08x_ready ? "OK" : "FAIL",
            s_reports_enabled ? "OK" : "FAIL");
    return s_bno08x_ready;
}

bool hal_imu_probe_once(uint8_t* out_response) {
    if (!out_response) return false;
    *out_response = s_bno08x_ready ? 0x01 : 0x00;
    return s_bno08x_ready;
}

bool hal_imu_detect_boot_qualified(HalImuDetectStats* out) {
    HalImuDetectStats local = {};
    local.samples = 1;
    local.present = s_bno08x_ready;
    local.last_response = s_bno08x_ready ? 0x01 : 0x00;
    if (s_bno08x_ready) {
        local.ok_count = 1;
        local.other_count = 1;
    } else {
        local.zero_count = 1;
    }

    hal_log("ESP32: IMU boot check via 7Semi example SPI: present=%s reports=%s",
            local.present ? "YES" : "NO",
            s_reports_enabled ? "OK" : "FAIL");

    if (out) {
        *out = local;
    }
    return local.present;
}

void hal_imu_on_sensor_spi_reinit(void) {
    recoverAfterSpiReinit();
}
