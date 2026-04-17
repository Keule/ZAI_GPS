/**
 * @file hal_impl.cpp
 * @brief ESP32-S3 HAL implementation for LilyGO T-ETH-Lite-S3.
 *
 * Hardware:
 *   - MCU: ESP32-S3-WROOM-1
 *   - Ethernet: W5500 over SPI3_HOST (GPIO 9/10/11/12/13/14) via ESP-IDF ETH driver
 *   - Sensor SPI (FSPI/SPI2_HOST): SCK=16, MISO=15, MOSI=17
 *     - ADS1118 ADC (steer angle): CS=18
 *     - BNO085 IMU: CS=47
 *     - Actuator: CS=40
 *   - IMU sideband wiring: INT=45, RST=48, WAKE=38
 *   - SD Card (FSPI, OTA only): SCK=7, MISO=5, MOSI=6, CS=42
 *   - Safety: GPIO4 active LOW
 *
 * ADS1118 ADC uses the libdriver/ads1118 library (lib/ads1118/).
 * Interface functions (SPI transmit, init, deinit, delay, debug) are
 * implemented inline in this file to access the shared sensorSPI bus.
 *
 * W5500 Ethernet uses the ESP-IDF ETH driver:
 *   - Arduino ESP32 Core >= 3.0.0: native <ETH.h>
 *   - Arduino ESP32 Core <  3.0.0: LilyGO ETHClass2 library
 * NOT the Arduino Ethernet/Ethernet3 library.
 *
 * This file includes Arduino/ESP32 headers - it must NEVER be included
 * from PC simulation code.
 */

#include "hal_impl.h"
#include "hal/hal.h"
#include "hardware_pins.h"
#include "logic/pgn_types.h"
#include "logic/log_config.h"
#include "logic/log_ext.h"

#define LOG_LOCAL_LEVEL LOG_LEVEL_HAL
#include "esp_log.h"

// ===================================================================
// Arduino / ESP32 includes
// ===================================================================
#include <Arduino.h>
#include <SPI.h>           // SPIClass for sensor bus (FSPI)
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>    // NVS flash storage for calibration
#include "driver_ads1118.h"  // libdriver ADS1118
#include <cstring>

// ===================================================================
// ETH driver selection based on Arduino ESP32 Core version
// ===================================================================
// Arduino ESP32 Core >= 3.0.0:  use native ETH (from <ETH.h>)
// Arduino ESP32 Core <  3.0.0:  use LilyGO ETHClass2 (lib/ETHClass2/)
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    #include <ETH.h>
    // Native ETH provides ETH global instance
#else
    #include "ETHClass2.h"
    // LilyGO ETHClass2 provides ETH2 global instance
    #define ETH ETH2
#endif

// ===================================================================
// W5500 Ethernet - ESP-IDF ETH driver (SPI3_HOST)
// ===================================================================

// UDP sockets for AgIO communication
// Reference uses separate sockets: one for receiving (port 8888),
// one for sending (from port 5126 to AgIO port 9999).
static WiFiUDP ethUDP_recv;  // Listen socket – bound to port 8888 (receives from AgIO)
static WiFiUDP ethUDP_send;  // Send socket – sends FROM port 5126 TO AgIO port 9999

// Static IP configuration
static IPAddress s_local_ip(192, 168, 1, 70);
static IPAddress s_subnet(255, 255, 255, 0);
static IPAddress s_gateway(192, 168, 1, 1);
static IPAddress s_dns(8, 8, 8, 8);
static IPAddress s_dest_ip(192, 168, 1, 255);

// Ethernet state tracking
static bool s_w5500_detected = false;   // true if ETH.begin() succeeded
static bool s_eth_link_up    = false;   // true if ARDUINO_EVENT_ETH_CONNECTED
static bool s_eth_has_ip     = false;   // true if ARDUINO_EVENT_ETH_GOT_IP

// ===================================================================
// Shared SPI bus - FSPI / SPI2_HOST
//
// CRITICAL: Must use FSPI, NOT HSPI!
// On ESP32-S3 (Arduino Core 2.x):  HSPI = SPI3_HOST (occupied by W5500!)
//                                  FSPI = SPI2_HOST (free for sensors)
//
// Sensor devices on this bus: ADS1118 (CS=18), IMU (CS=47), Actuator (CS=40).
// Pins: SCK=16, MISO=15, MOSI=17.
//
// SD card uses the SAME SPI peripheral (FSPI) but DIFFERENT pins (SCK=7, MISO=5, MOSI=6).
// During OTA updates, FSPI is re-initialised with SD pins via
// hal_sensor_spi_deinit() / hal_sensor_spi_reinit().
// ===================================================================
static SPIClass sensorSPI(FSPI);

// ===================================================================
// Shared sensor SPI transaction layer (bus lock + per-device settings)
// ===================================================================
enum class SpiClient : uint8_t {
    ADS1118 = 0,
    BNO085 = 1,
    ACTUATOR = 2,
    NONE = 0xFF,
};

struct SpiClientConfig {
    int cs_pin;
    uint32_t freq_hz;
    uint8_t mode;
    uint32_t period_us;   // polling interval target
};

static const SpiClientConfig k_spi_cfg_ads = {CS_STEER_ANG, 2000000, SPI_MODE1, 10000};  // 100 Hz
static SpiClientConfig k_spi_cfg_imu = {CS_IMU,       1000000, SPI_MODE3,  5000};  // 200 Hz
static const SpiClientConfig k_spi_cfg_act = {CS_ACT,       1000000, SPI_MODE0,     0};  // event-driven

static SemaphoreHandle_t s_spi_bus_mutex = nullptr;

struct SpiPollState {
    uint32_t next_due_us = 0;
    uint32_t deadline_miss = 0;
    uint32_t transactions = 0;
    uint32_t last_us = 0;
    uint32_t max_us = 0;
};

struct SpiBusTelemetry {
    uint32_t window_start_us = 0;
    uint32_t busy_us = 0;
    uint32_t transactions = 0;
};

static SpiPollState s_poll_imu;
static SpiPollState s_poll_was;
static SpiPollState s_poll_act;
static SpiBusTelemetry s_bus_tm;
static SpiClient s_last_spi_client = SpiClient::NONE;
static uint32_t s_last_spi_end_us = 0;
static SpiClient s_last_sensor_spi_client = SpiClient::NONE;
static uint32_t s_last_sensor_spi_end_us = 0;
static uint32_t s_client_switches = 0;
static uint32_t s_was_to_imu_switches = 0;
static uint32_t s_imu_to_was_switches = 0;
static uint32_t s_other_switches = 0;
static uint32_t s_was_to_imu_gap_last_us = 0;
static uint32_t s_was_to_imu_gap_max_us = 0;
static uint32_t s_imu_to_was_gap_last_us = 0;
static uint32_t s_imu_to_was_gap_max_us = 0;
static uint32_t s_sensor_was_to_imu_switches = 0;
static uint32_t s_sensor_imu_to_was_switches = 0;
static uint32_t s_sensor_was_to_imu_gap_last_us = 0;
static uint32_t s_sensor_was_to_imu_gap_max_us = 0;
static uint32_t s_sensor_imu_to_was_gap_last_us = 0;
static uint32_t s_sensor_imu_to_was_gap_max_us = 0;

static int16_t s_was_raw_cache = 0;
static uint32_t s_was_last_poll_us = 0;
static bool s_was_cache_valid = false;

static const SpiClientConfig& spiCfg(SpiClient client) {
    switch (client) {
    case SpiClient::ADS1118: return k_spi_cfg_ads;
    case SpiClient::BNO085: return k_spi_cfg_imu;
    case SpiClient::ACTUATOR: return k_spi_cfg_act;
    case SpiClient::NONE: break;
    }
    return k_spi_cfg_ads;
}

static void spiBeginCritical(void) {
    if (s_spi_bus_mutex) {
        xSemaphoreTake(s_spi_bus_mutex, portMAX_DELAY);
    }
}

static void spiEndCritical(void) {
    if (s_spi_bus_mutex) {
        xSemaphoreGive(s_spi_bus_mutex);
    }
}

static SpiPollState& spiPollForClient(SpiClient client) {
    switch (client) {
    case SpiClient::ADS1118: return s_poll_was;
    case SpiClient::BNO085: return s_poll_imu;
    case SpiClient::ACTUATOR: return s_poll_act;
    case SpiClient::NONE: break;
    }
    return s_poll_was;
}

static void updateLastMax(uint32_t& last_us, uint32_t& max_us, uint32_t value_us) {
    last_us = value_us;
    if (value_us > max_us) {
        max_us = value_us;
    }
}

static void spiRecordTiming(SpiClient client, uint32_t request_us, uint32_t lock_us, uint32_t end_us) {
    const uint32_t dt_us = end_us - request_us;
    SpiPollState& poll = spiPollForClient(client);
    poll.transactions++;
    poll.last_us = dt_us;
    if (dt_us > poll.max_us) {
        poll.max_us = dt_us;
    }

    s_bus_tm.busy_us += dt_us;
    s_bus_tm.transactions++;

    if (s_last_spi_client != SpiClient::NONE && s_last_spi_client != client) {
        s_client_switches++;
        const uint32_t switch_gap_us = s_last_spi_end_us == 0 ? 0 : (lock_us - s_last_spi_end_us);
        if (s_last_spi_client == SpiClient::ADS1118 && client == SpiClient::BNO085) {
            s_was_to_imu_switches++;
            updateLastMax(s_was_to_imu_gap_last_us, s_was_to_imu_gap_max_us, switch_gap_us);
        } else if (s_last_spi_client == SpiClient::BNO085 && client == SpiClient::ADS1118) {
            s_imu_to_was_switches++;
            updateLastMax(s_imu_to_was_gap_last_us, s_imu_to_was_gap_max_us, switch_gap_us);
        } else {
            s_other_switches++;
        }
    }
    s_last_spi_client = client;
    s_last_spi_end_us = end_us;

    if (client == SpiClient::ADS1118 || client == SpiClient::BNO085) {
        if (s_last_sensor_spi_client != SpiClient::NONE && s_last_sensor_spi_client != client) {
            const uint32_t sensor_gap_us = s_last_sensor_spi_end_us == 0 ? 0 : (lock_us - s_last_sensor_spi_end_us);
            if (s_last_sensor_spi_client == SpiClient::ADS1118 && client == SpiClient::BNO085) {
                s_sensor_was_to_imu_switches++;
                updateLastMax(s_sensor_was_to_imu_gap_last_us, s_sensor_was_to_imu_gap_max_us, sensor_gap_us);
            } else if (s_last_sensor_spi_client == SpiClient::BNO085 && client == SpiClient::ADS1118) {
                s_sensor_imu_to_was_switches++;
                updateLastMax(s_sensor_imu_to_was_gap_last_us, s_sensor_imu_to_was_gap_max_us, sensor_gap_us);
            }
        }
        s_last_sensor_spi_client = client;
        s_last_sensor_spi_end_us = end_us;
    }
}

static bool spiTransfer(SpiClient client, const uint8_t* tx, uint8_t* rx, size_t len) {
    const SpiClientConfig& cfg = spiCfg(client);
    if (len == 0) return true;

    const uint32_t request_us = micros();
    spiBeginCritical();
    const uint32_t lock_us = micros();

    digitalWrite(CS_STEER_ANG, HIGH);
    digitalWrite(CS_IMU, HIGH);
    digitalWrite(CS_ACT, HIGH);

    sensorSPI.beginTransaction(SPISettings(cfg.freq_hz, MSBFIRST, cfg.mode));
    digitalWrite(cfg.cs_pin, LOW);
    for (size_t i = 0; i < len; i++) {
        const uint8_t v = sensorSPI.transfer(tx ? tx[i] : 0xFF);
        if (rx) rx[i] = v;
    }
    digitalWrite(cfg.cs_pin, HIGH);
    sensorSPI.endTransaction();

    spiEndCritical();
    const uint32_t end_us = micros();
    spiRecordTiming(client, request_us, lock_us, end_us);
    return true;
}

static void spiCheckDeadline(SpiPollState* poll, uint32_t period_us, uint32_t now_us) {
    if (!poll || period_us == 0) return;
    if (poll->next_due_us == 0) {
        poll->next_due_us = now_us + period_us;
        return;
    }
    if (now_us > poll->next_due_us + period_us) {
        poll->deadline_miss++;
    }
    while (poll->next_due_us <= now_us) {
        poll->next_due_us += period_us;
    }
}

void hal_esp32_imu_spi_check_deadline(uint32_t period_us, uint32_t now_us) {
    spiCheckDeadline(&s_poll_imu, period_us, now_us);
}

bool hal_esp32_imu_raw_transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    return spiTransfer(SpiClient::BNO085, tx, rx, len);
}

uint32_t hal_esp32_sensor_spi_timing_now_us(void) {
    return micros();
}

void hal_esp32_sensor_spi_lock(void) {
    spiBeginCritical();
}

void hal_esp32_sensor_spi_unlock(void) {
    spiEndCritical();
}

void hal_esp32_sensor_spi_record_imu_transfer(uint32_t request_us, uint32_t lock_us, uint32_t end_us) {
    spiRecordTiming(SpiClient::BNO085, request_us, lock_us, end_us);
}

SPIClass& hal_esp32_sensor_spi_port(void) {
    return sensorSPI;
}

void hal_imu_on_sensor_spi_reinit(void);

// ===================================================================
// Mutex (FreeRTOS recursive mutex) — for NavigationState protection
// ===================================================================
#if configSUPPORT_STATIC_ALLOCATION
static StaticSemaphore_t s_mutex_buffer;
static SemaphoreHandle_t s_mutex = nullptr;
#else
static SemaphoreHandle_t s_mutex = nullptr;
#endif

// ===================================================================
// Serial log mutex — protects USB CDC Serial from concurrent access.
// USB CDC (Serial on ESP32-S3) is NOT thread-safe and will crash
// if two tasks call Serial.printf() simultaneously on different cores.
// ===================================================================
static SemaphoreHandle_t s_log_mutex = nullptr;

// ===================================================================
// GNSS RTCM UART (UM980 corrections, 8N1)
// ===================================================================
static HardwareSerial* s_gnss_rtcm_uart = &Serial1;
static uint8_t s_gnss_rtcm_uart_num = 1;
static SemaphoreHandle_t s_gnss_rtcm_mutex = nullptr;
static bool s_gnss_rtcm_ready = false;
static uint32_t s_gnss_rtcm_drop_bytes = 0;

static HardwareSerial* gnssUartForNum(uint8_t uart_num) {
    switch (uart_num) {
    case 1: return &Serial1;
    case 2: return &Serial2;
    default: return nullptr;
    }
}

void hal_esp32_gnss_rtcm_set_uart(uint8_t uart_num) {
    HardwareSerial* uart = gnssUartForNum(uart_num);
    if (!uart) {
        LOGW("HAL", "GNSS RTCM UART%u unsupported (valid: 1 or 2), keeping UART%u",
             static_cast<unsigned>(uart_num),
             static_cast<unsigned>(s_gnss_rtcm_uart_num));
        return;
    }
    if (s_gnss_rtcm_ready) {
        LOGW("HAL", "GNSS RTCM UART already active on UART%u, switch ignored",
             static_cast<unsigned>(s_gnss_rtcm_uart_num));
        return;
    }
    s_gnss_rtcm_uart_num = uart_num;
    s_gnss_rtcm_uart = uart;
    LOGI("HAL", "GNSS RTCM mapped to UART%u", static_cast<unsigned>(s_gnss_rtcm_uart_num));
}

bool hal_gnss_rtcm_begin(uint32_t baud, int8_t rx_pin, int8_t tx_pin) {
    if (!s_gnss_rtcm_uart) {
        LOGE("HAL", "GNSS RTCM begin failed: UART mapping is null");
        return false;
    }
    if (baud == 0) {
        LOGE("HAL", "GNSS RTCM begin failed: baud must be > 0");
        return false;
    }
    if (tx_pin < 0) {
        LOGE("HAL", "GNSS RTCM begin failed: TX pin must be >= 0");
        return false;
    }

    if (!s_gnss_rtcm_mutex) {
        s_gnss_rtcm_mutex = xSemaphoreCreateMutex();
    }
    if (s_gnss_rtcm_mutex) {
        xSemaphoreTake(s_gnss_rtcm_mutex, portMAX_DELAY);
    }

    const int uart_rx = (rx_pin < 0) ? -1 : static_cast<int>(rx_pin);
    const int uart_tx = static_cast<int>(tx_pin);

    s_gnss_rtcm_uart->begin(baud, SERIAL_8N1, uart_rx, uart_tx);
    s_gnss_rtcm_ready = true;
    if (s_gnss_rtcm_mutex) {
        xSemaphoreGive(s_gnss_rtcm_mutex);
    }

    LOGI("HAL", "GNSS RTCM UART%u ready (baud=%lu, mode=8N1, rx=%d, tx=%d)",
         static_cast<unsigned>(s_gnss_rtcm_uart_num),
         static_cast<unsigned long>(baud),
         uart_rx,
         uart_tx);
    return true;
}

size_t hal_gnss_rtcm_write(const uint8_t* data, size_t len) {
    if (!data || len == 0) return 0;
    if (!s_gnss_rtcm_mutex) {
        s_gnss_rtcm_mutex = xSemaphoreCreateMutex();
    }
    if (s_gnss_rtcm_mutex) {
        xSemaphoreTake(s_gnss_rtcm_mutex, portMAX_DELAY);
    }
    if (!s_gnss_rtcm_ready || !s_gnss_rtcm_uart) {
        LOGW("HAL", "GNSS RTCM write dropped (%u bytes): UART not initialised",
             static_cast<unsigned>(len));
        s_gnss_rtcm_drop_bytes += static_cast<uint32_t>(len);
        if (s_gnss_rtcm_mutex) {
            xSemaphoreGive(s_gnss_rtcm_mutex);
        }
        return 0;
    }

    const size_t written = s_gnss_rtcm_uart->write(data, len);
    if (s_gnss_rtcm_mutex) {
        xSemaphoreGive(s_gnss_rtcm_mutex);
    }

    if (written < len) {
        const size_t dropped = len - written;
        s_gnss_rtcm_drop_bytes += static_cast<uint32_t>(dropped);
        LOGW("HAL", "GNSS RTCM short write: %u/%u bytes", static_cast<unsigned>(written), static_cast<unsigned>(len));
    }

    return written;
}

bool hal_gnss_rtcm_is_ready(void) {
    return s_gnss_rtcm_ready;
}

uint32_t hal_gnss_rtcm_drop_count(void) {
    if (!s_gnss_rtcm_mutex) {
        return s_gnss_rtcm_drop_bytes;
    }
    xSemaphoreTake(s_gnss_rtcm_mutex, portMAX_DELAY);
    const uint32_t dropped = s_gnss_rtcm_drop_bytes;
    xSemaphoreGive(s_gnss_rtcm_mutex);
    return dropped;
}

// ===================================================================
// Timing
// ===================================================================
uint32_t hal_millis(void) {
    return ::millis();
}

uint32_t hal_micros(void) {
    return ::micros();
}

void hal_delay_ms(uint32_t ms) {
    ::delay(ms);
}

// ===================================================================
// Logging – prints to USB CDC Serial via Serial.printf.
//
// ESP_LOGI goes to UART0 by default, which does NOT appear on
// USB CDC Serial on ESP32-S3.  Serial.printf goes to USB CDC.
//
// hal_log() is kept for ABI compatibility with logic/ modules.
// New code should use LOGI/LOGD/LOGW/LOGE from log_ext.h directly.
// ===================================================================
void hal_log(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    char category[20] = "LOG";
    char detail[20] = "";
    const char* body = buf;
    const char* colon = std::strchr(buf, ':');
    if (colon && colon != buf && (colon - buf) < 28) {
        bool tag_ok = true;
        for (const char* p = buf; p < colon; ++p) {
            const char c = *p;
            const bool alpha = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
            const bool digit = (c >= '0' && c <= '9');
            if (!alpha && !digit && c != '_' && c != '-') {
                tag_ok = false;
                break;
            }
        }

        if (tag_ok) {
            const char* dash = nullptr;
            for (const char* p = buf; p < colon; ++p) {
                if (*p == '-') {
                    dash = p;
                    break;
                }
            }

            const char* category_end = dash ? dash : colon;
            size_t category_len = static_cast<size_t>(category_end - buf);
            if (category_len >= sizeof(category)) category_len = sizeof(category) - 1;
            for (size_t i = 0; i < category_len; ++i) {
                const char c = buf[i];
                category[i] = (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
            }
            category[category_len] = '\0';

            if (dash && dash + 1 < colon) {
                size_t detail_len = static_cast<size_t>(colon - dash - 1);
                if (detail_len >= sizeof(detail)) detail_len = sizeof(detail) - 1;
                for (size_t i = 0; i < detail_len; ++i) {
                    const char c = dash[1 + i];
                    detail[i] = (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
                }
                detail[detail_len] = '\0';
            }

            body = colon + 1;
            while (*body == ' ') {
                body++;
            }
        }
    }

    // Protect Serial (USB CDC) from concurrent access across cores.
    if (s_log_mutex) {
        xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    }
    if (detail[0] != '\0') {
        Serial.printf("[%s] [%10lu] %s: %s\n", category, millis(), detail, body);
    } else {
        Serial.printf("[%s] [%10lu] %s\n", category, millis(), body);
    }
    if (s_log_mutex) {
        xSemaphoreGive(s_log_mutex);
    }
}

// ===================================================================
// Mutex
// ===================================================================
void hal_mutex_init(void) {
    // State mutex (recursive, for NavigationState)
#if configSUPPORT_STATIC_ALLOCATION
    s_mutex = xSemaphoreCreateRecursiveMutexStatic(&s_mutex_buffer);
#else
    s_mutex = xSemaphoreCreateRecursiveMutex();
#endif

    // Serial log mutex (binary, protects USB CDC from concurrent access)
    s_log_mutex = xSemaphoreCreateMutex();
}

void hal_mutex_lock(void) {
    if (s_mutex) {
        xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY);
    }
}

void hal_mutex_unlock(void) {
    if (s_mutex) {
        xSemaphoreGiveRecursive(s_mutex);
    }
}

// ===================================================================
// Safety input
// ===================================================================
bool hal_safety_ok(void) {
    // SAFETY_IN is active LOW: LOW = KICK, HIGH = OK
    return digitalRead(SAFETY_IN) == HIGH;
}

// ===================================================================
// SPI Sensors / Actuator - SPI Bus 2 (FSPI / SPI2_HOST)
// ===================================================================
void hal_sensor_spi_init(void) {
    sensorSPI.begin(SENS_SPI_SCK, SENS_SPI_MISO, SENS_SPI_MOSI, -1);
    if (!s_spi_bus_mutex) {
        s_spi_bus_mutex = xSemaphoreCreateMutex();
    }
    s_bus_tm.window_start_us = micros();
    s_bus_tm.busy_us = 0;
    s_bus_tm.transactions = 0;
    s_poll_imu = {};
    s_poll_was = {};
    s_poll_act = {};
    s_last_spi_client = SpiClient::NONE;
    s_last_spi_end_us = 0;
    s_last_sensor_spi_client = SpiClient::NONE;
    s_last_sensor_spi_end_us = 0;
    s_client_switches = 0;
    s_was_to_imu_switches = 0;
    s_imu_to_was_switches = 0;
    s_other_switches = 0;
    s_was_to_imu_gap_last_us = 0;
    s_was_to_imu_gap_max_us = 0;
    s_imu_to_was_gap_last_us = 0;
    s_imu_to_was_gap_max_us = 0;
    s_sensor_was_to_imu_switches = 0;
    s_sensor_imu_to_was_switches = 0;
    s_sensor_was_to_imu_gap_last_us = 0;
    s_sensor_was_to_imu_gap_max_us = 0;
    s_sensor_imu_to_was_gap_last_us = 0;
    s_sensor_imu_to_was_gap_max_us = 0;
    s_was_cache_valid = false;
    hal_log("ESP32: sensor SPI initialised on FSPI/SPI2_HOST (SCK=%d MISO=%d MOSI=%d)",
            SENS_SPI_SCK, SENS_SPI_MISO, SENS_SPI_MOSI);
}

void hal_sensor_spi_deinit(void) {
    spiBeginCritical();
    sensorSPI.end();
    spiEndCritical();
    hal_log("ESP32: shared SPI released (FSPI peripheral free)");
}

void hal_sensor_spi_reinit(void) {
    // Ensure the bus is fully released before re-initialising.
    // The OTA code creates a LOCAL SPIClass(FSPI) which can leave
    // the FSPI peripheral in an inconsistent state.  Calling end()
    // again on our sensorSPI forces a clean release, then we re-init
    // with a settling delay.
    spiBeginCritical();
    sensorSPI.end();
    delay(50);   // let SPI peripheral fully settle
    sensorSPI.begin(SENS_SPI_SCK, SENS_SPI_MISO, SENS_SPI_MOSI, -1);
    s_bus_tm.window_start_us = micros();
    s_bus_tm.busy_us = 0;
    s_bus_tm.transactions = 0;
    s_poll_imu = {};
    s_poll_was = {};
    s_poll_act = {};
    s_last_spi_client = SpiClient::NONE;
    s_last_spi_end_us = 0;
    s_last_sensor_spi_client = SpiClient::NONE;
    s_last_sensor_spi_end_us = 0;
    s_client_switches = 0;
    s_was_to_imu_switches = 0;
    s_imu_to_was_switches = 0;
    s_other_switches = 0;
    s_was_to_imu_gap_last_us = 0;
    s_was_to_imu_gap_max_us = 0;
    s_imu_to_was_gap_last_us = 0;
    s_imu_to_was_gap_max_us = 0;
    s_sensor_was_to_imu_switches = 0;
    s_sensor_imu_to_was_switches = 0;
    s_sensor_was_to_imu_gap_last_us = 0;
    s_sensor_was_to_imu_gap_max_us = 0;
    s_sensor_imu_to_was_gap_last_us = 0;
    s_sensor_imu_to_was_gap_max_us = 0;
    s_was_cache_valid = false;
    spiEndCritical();
    delay(10);   // let GPIO matrix reconfigure
    hal_log("ESP32: shared SPI re-initialised on FSPI/SPI2_HOST (SCK=%d MISO=%d MOSI=%d)",
            SENS_SPI_SCK, SENS_SPI_MISO, SENS_SPI_MOSI);
    hal_imu_on_sensor_spi_reinit();
}

void hal_sensor_spi_get_telemetry(HalSpiTelemetry* out) {
    if (!out) return;
    std::memset(out, 0, sizeof(*out));
    const uint32_t now_us = micros();
    const uint32_t win_us = (s_bus_tm.window_start_us == 0 || now_us <= s_bus_tm.window_start_us)
                                ? 0
                                : (now_us - s_bus_tm.window_start_us);
    out->window_ms = win_us / 1000;
    out->bus_busy_us = s_bus_tm.busy_us;
    out->bus_transactions = s_bus_tm.transactions;
    out->imu_transactions = s_poll_imu.transactions;
    out->was_transactions = s_poll_was.transactions;
    out->actuator_transactions = s_poll_act.transactions;
    out->imu_last_us = s_poll_imu.last_us;
    out->imu_max_us = s_poll_imu.max_us;
    out->was_last_us = s_poll_was.last_us;
    out->was_max_us = s_poll_was.max_us;
    out->actuator_last_us = s_poll_act.last_us;
    out->actuator_max_us = s_poll_act.max_us;
    out->client_switches = s_client_switches;
    out->was_to_imu_switches = s_was_to_imu_switches;
    out->imu_to_was_switches = s_imu_to_was_switches;
    out->other_switches = s_other_switches;
    out->was_to_imu_gap_last_us = s_was_to_imu_gap_last_us;
    out->was_to_imu_gap_max_us = s_was_to_imu_gap_max_us;
    out->imu_to_was_gap_last_us = s_imu_to_was_gap_last_us;
    out->imu_to_was_gap_max_us = s_imu_to_was_gap_max_us;
    out->sensor_was_to_imu_switches = s_sensor_was_to_imu_switches;
    out->sensor_imu_to_was_switches = s_sensor_imu_to_was_switches;
    out->sensor_was_to_imu_gap_last_us = s_sensor_was_to_imu_gap_last_us;
    out->sensor_was_to_imu_gap_max_us = s_sensor_was_to_imu_gap_max_us;
    out->sensor_imu_to_was_gap_last_us = s_sensor_imu_to_was_gap_last_us;
    out->sensor_imu_to_was_gap_max_us = s_sensor_imu_to_was_gap_max_us;
    out->imu_deadline_miss = s_poll_imu.deadline_miss;
    out->was_deadline_miss = s_poll_was.deadline_miss;
    if (win_us > 0) {
        out->bus_utilization_pct = (100.0f * static_cast<float>(s_bus_tm.busy_us)) / static_cast<float>(win_us);
    }
}

void hal_imu_get_spi_info(HalImuSpiInfo* out) {
    if (!out) return;
    out->sck_pin = SENS_SPI_SCK;
    out->miso_pin = SENS_SPI_MISO;
    out->mosi_pin = SENS_SPI_MOSI;
    out->cs_pin = CS_IMU;
    out->int_pin = IMU_INT;
    out->freq_hz = k_spi_cfg_imu.freq_hz;
    out->mode = k_spi_cfg_imu.mode;
}

void hal_imu_set_spi_config(uint32_t freq_hz, uint8_t mode) {
    if (freq_hz == 0) return;
    k_spi_cfg_imu.freq_hz = freq_hz;
    k_spi_cfg_imu.mode = mode;
    hal_log("ESP32: IMU SPI config set (freq=%luHz mode=%u)",
            (unsigned long)k_spi_cfg_imu.freq_hz,
            (unsigned)k_spi_cfg_imu.mode);
}

// ===================================================================
// ADS1118 - 16-Bit ADC for steering angle potentiometer
// ===================================================================
// Uses libdriver/ads1118 (lib/ads1118/src/) for initialisation only.
// For runtime reads, we bypass the libdriver and read raw ADC data
// directly via SPI to avoid the "range is invalid" bug in the
// libdriver's continuous_read() (which reads back the config register
// and fails if SPI returns garbage).
//
// The libdriver is used only for:
//   - ads1118_init() — verify interface functions
//   - ads1118_set_channel/range/rate/mode — configure the chip
//   - ads1118_start_continuous_read() — start continuous conversion
//
// All subsequent reads use ads1118_read_raw() which sends 0xFF×2
// and reads back 2 bytes as a raw int16_t ADC value. No config
// register read-back, no range validation.
//
// Wiring:
//   ADS1118 DOUT  -> GPIO 15 (MISO)
//   ADS1118 DIN   -> GPIO 17 (MOSI)
//   ADS1118 SCLK  -> GPIO 16 (SCK)
//   ADS1118 CS    -> GPIO 18
//
// Calibration:
//   Raw ADC min (left stop)  -> -22.5°
//   Raw ADC max (right stop) -> +22.5°
//   Stored in NVS (Preferences) and survives reboots.
// ===================================================================

/// libdriver ADS1118 handle (used for init/config only)
static ads1118_handle_t s_ads1118_handle;

/// ADS1118 detected flag
static bool s_ads1118_detected = false;

/// Calibration state
static bool   s_calibrated = false;
static int16_t s_cal_left_raw  = 0;   // ADC value at left stop  -> -22.5°
static int16_t s_cal_right_raw = 0;   // ADC value at right stop -> +22.5°

/// NVS namespace and keys for calibration persistence
static const char* NVS_NAMESPACE = "agsteer";
static const char* NVS_KEY_CAL_VALID  = "cal_v";
static const char* NVS_KEY_CAL_LEFT   = "cal_l";
static const char* NVS_KEY_CAL_RIGHT  = "cal_r";
static constexpr uint32_t NVS_CAL_MAGIC = 0xA6511C00;  // magic to validate stored data

// --- libdriver interface functions (static, ESP32-specific) ---

static uint8_t ads1118_if_spi_init(void) {
    pinMode(CS_STEER_ANG, OUTPUT);
    digitalWrite(CS_STEER_ANG, HIGH);
    return 0;
}

static uint8_t ads1118_if_spi_deinit(void) {
    return 0;  // Don't deinit shared bus
}

static uint8_t ads1118_if_spi_transmit(uint8_t *tx, uint8_t *rx, uint16_t len) {
    spiTransfer(SpiClient::ADS1118, tx, rx, len);
    return 0;
}

static void ads1118_if_delay_ms(uint32_t ms) {
    delay(ms);
}

static void ads1118_if_debug_print(const char *const fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buf[256];
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
}

// --- Direct raw ADC read (bypasses libdriver) ---

/// Read one raw 16-bit ADC sample directly via SPI.
/// Sends 0xFF×2, receives 2 bytes as big-endian int16_t.
/// This avoids the libdriver's config register read-back which can
/// trigger "range is invalid" if SPI returns garbage.
static int16_t ads1118_read_raw(void) {
    const uint32_t now_us = micros();
    if (s_was_cache_valid && (now_us - s_was_last_poll_us) < k_spi_cfg_ads.period_us) {
        return s_was_raw_cache;
    }
    spiCheckDeadline(&s_poll_was, k_spi_cfg_ads.period_us, now_us);
    uint8_t tx[2] = {0xFF, 0xFF};
    uint8_t rx[2] = {0};
    ads1118_if_spi_transmit(tx, rx, 2);
    s_was_raw_cache = static_cast<int16_t>((static_cast<uint16_t>(rx[0]) << 8) | rx[1]);
    s_was_last_poll_us = now_us;
    s_was_cache_valid = true;
    return s_was_raw_cache;
}

/// Read multiple raw samples and return the median value.
/// More robust than average against outliers.
/// @param samples  number of samples to read (should be odd for true median)
/// @param delay_ms delay between samples in ms (default: 5ms for 250 SPS)
static int16_t ads1118_read_raw_median(int samples, int sample_delay_ms = 5) {
    // Buffer for samples (max 31, enough for median)
    int16_t buf[31];
    if (samples > 31) samples = 31;
    if (samples < 1) samples = 1;

    for (int i = 0; i < samples; i++) {
        buf[i] = ads1118_read_raw();
        if (sample_delay_ms > 0) delay(sample_delay_ms);
    }

    // Simple bubble sort for small array
    for (int i = 0; i < samples - 1; i++) {
        for (int j = i + 1; j < samples; j++) {
            if (buf[j] < buf[i]) {
                int16_t tmp = buf[i];
                buf[i] = buf[j];
                buf[j] = tmp;
            }
        }
    }

    return buf[samples / 2];  // median
}

// --- Calibration persistence (NVS) ---

/// Load calibration from NVS flash.
/// Returns true if valid calibration was loaded.
static bool steer_cal_load(void) {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) {  // read-only mode
        hal_log("SteerCal: NVS open failed");
        return false;
    }

    uint32_t magic = prefs.getUInt(NVS_KEY_CAL_VALID, 0);
    if (magic != NVS_CAL_MAGIC) {
        prefs.end();
        hal_log("SteerCal: no valid calibration in NVS");
        return false;
    }

    s_cal_left_raw  = static_cast<int16_t>(prefs.getInt(NVS_KEY_CAL_LEFT, 0));
    s_cal_right_raw = static_cast<int16_t>(prefs.getInt(NVS_KEY_CAL_RIGHT, 0));
    s_calibrated = true;

    prefs.end();

    hal_log("SteerCal: loaded from NVS (left=%d, right=%d)",
            s_cal_left_raw, s_cal_right_raw);
    return true;
}

/// Save calibration to NVS flash.
static void steer_cal_save(void) {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {  // read-write mode
        hal_log("SteerCal: NVS open failed, cannot save!");
        return;
    }

    prefs.putUInt(NVS_KEY_CAL_VALID, NVS_CAL_MAGIC);
    prefs.putInt(NVS_KEY_CAL_LEFT, static_cast<int32_t>(s_cal_left_raw));
    prefs.putInt(NVS_KEY_CAL_RIGHT, static_cast<int32_t>(s_cal_right_raw));

    prefs.end();

    hal_log("SteerCal: saved to NVS (left=%d, right=%d)",
            s_cal_left_raw, s_cal_right_raw);
}

/// Wait for user to press Enter on Serial monitor.
/// While waiting, shows live ADC voltage at 20 Hz (every 50ms).
static void wait_for_enter_live_adc(void) {
    uint32_t last_print = 0;
    while (true) {
        // Check for Enter key
        if (Serial.available()) {
            int c = Serial.read();
            if (c == '\n' || c == '\r') {
                while (Serial.available()) Serial.read();
                // Print one final value + newline to cleanly end the live line
                int16_t raw = ads1118_read_raw();
                float voltage = raw * 4.096f / 32768.0f;
                Serial.printf("   -> %7.3f V  (raw=%d)\n", voltage, raw);
                delay(50);
                return;
            }
        }
        // Print live ADC value every 50ms using \r to overwrite line
        uint32_t now = millis();
        if (now - last_print >= 50) {
            last_print = now;
            int16_t raw = ads1118_read_raw();
            float voltage = raw * 4.096f / 32768.0f;
            Serial.printf("   -> %7.3f V  (raw=%d)  \r", voltage, raw);
        }
        delay(2);
    }
}

void hal_steer_angle_begin(void) {
    // Ensure all other SPI device CS pins are configured as outputs
    pinMode(CS_IMU, OUTPUT);  digitalWrite(CS_IMU, HIGH);
    pinMode(CS_ACT, OUTPUT);  digitalWrite(CS_ACT, HIGH);

    // Wire up libdriver handle with ESP32 interface functions
    DRIVER_ADS1118_LINK_INIT(&s_ads1118_handle, ads1118_handle_t);
    DRIVER_ADS1118_LINK_SPI_INIT(&s_ads1118_handle, ads1118_if_spi_init);
    DRIVER_ADS1118_LINK_SPI_DEINIT(&s_ads1118_handle, ads1118_if_spi_deinit);
    DRIVER_ADS1118_LINK_SPI_TRANSMIT(&s_ads1118_handle, ads1118_if_spi_transmit);
    DRIVER_ADS1118_LINK_DELAY_MS(&s_ads1118_handle, ads1118_if_delay_ms);
    DRIVER_ADS1118_LINK_DEBUG_PRINT(&s_ads1118_handle, ads1118_if_debug_print);

    // Initialise driver
    uint8_t res = ads1118_init(&s_ads1118_handle);
    if (res != 0) {
        hal_log("ESP32: ADS1118 init failed (err=%u)", res);
        return;
    }

    // Configure: AIN0 single-ended, +/-4.096V, 250 SPS, ADC mode
    ads1118_set_channel(&s_ads1118_handle, ADS1118_CHANNEL_AIN0_GND);
    ads1118_set_range(&s_ads1118_handle, ADS1118_RANGE_4P096V);
    ads1118_set_rate(&s_ads1118_handle, ADS1118_RATE_250SPS);
    ads1118_set_mode(&s_ads1118_handle, ADS1118_MODE_ADC);

    // Try to load calibration from NVS
    steer_cal_load();

    hal_log("ESP32: ADS1118 configured (AIN0, +/-4.096V, 250 SPS, calibrated=%s)",
            s_calibrated ? "YES" : "NO");
}

bool hal_steer_angle_detect(void) {
    // Detection: try a direct raw read. If the value is not stuck at
    // 0x0000 or 0xFFFF (floating MISO), the ADS1118 is present.
    // Also discard 0x7FFF (all 1s in data, possible bus issue).
    //
    // After the OTA SD card check, the FSPI bus was deinitialised and
    // reinitialised by sd_ota_esp32.cpp.  The ADS1118 may need time to
    // recover, or the bus may not be fully functional yet.
    // Strategy: try up to 3 times with increasing delays.

    int16_t raw1 = 0;
    int16_t raw2 = 0;
    bool looks_valid = false;

    for (int attempt = 0; attempt < 3 && !looks_valid; attempt++) {
        if (attempt > 0) {
            hal_log("ESP32: ADS1118 detect retry %d/2...", attempt);
            delay(100);  // give ADS1118 and SPI bus more time
        }

        raw1 = ads1118_read_raw();
        delay(20);
        raw2 = ads1118_read_raw();

        looks_valid = (raw1 != 0 && raw1 != -1 && raw1 != 0x7FFF &&
                       raw2 != 0 && raw2 != -1 && raw2 != 0x7FFF);
    }

    // If still failing, try a full ADS1118 re-init (write config register)
    if (!looks_valid) {
        hal_log("ESP32: ADS1118 still 0x%04X after retries, re-initialising chip...", raw1);
        // Re-init: set CS pin, write config word to start continuous conversion
        // The ADS1118 config word: 0001 MUX PGA RATE MODE TS Reserved
        // MUX=100 (AIN0-GND), PGA=001 (4.096V), RATE=101 (250SPS), MODE=0 (continuous)
        uint8_t tx[4] = {0x84, 0xC3, 0xFF, 0xFF};  // config + 2 dummy bytes for data
        uint8_t rx[4];
        ads1118_if_spi_transmit(tx, rx, 4);
        delay(50);  // wait for first conversion at 250 SPS = 4ms, use 50ms margin

        raw1 = ads1118_read_raw();
        delay(20);
        raw2 = ads1118_read_raw();
        looks_valid = (raw1 != 0 && raw1 != -1 && raw1 != 0x7FFF &&
                       raw2 != 0 && raw2 != -1 && raw2 != 0x7FFF);
        hal_log("ESP32: ADS1118 after re-init: raw1=%d (0x%04X) raw2=%d (0x%04X) %s",
                raw1, (unsigned)raw1 & 0xFFFF, raw2, (unsigned)raw2 & 0xFFFF,
                looks_valid ? "OK" : "STILL FAIL");
    }

    s_ads1118_detected = looks_valid;

    if (s_ads1118_detected) {
        hal_log("ESP32: ADS1118 DETECTED (raw1=%d, raw2=%d)", raw1, raw2);

        // Start continuous conversion mode
        uint8_t res = ads1118_start_continuous_read(&s_ads1118_handle);
        if (res != 0) {
            hal_log("ESP32: ADS1118 continuous start note: err=%u (using direct reads)", res);
            // Don't fail detection — we use direct reads anyway
        } else {
            hal_log("ESP32: ADS1118 continuous mode started");
        }
    } else {
        hal_log("ESP32: ADS1118 DETECT FAILED (raw1=0x%04X, raw2=0x%04X)",
                (unsigned)raw1 & 0xFFFF, (unsigned)raw2 & 0xFFFF);
    }

    return s_ads1118_detected;
}

void hal_steer_angle_calibrate(void) {
    if (!s_ads1118_detected) {
        hal_log("SteerCal: ERROR — ADS1118 not detected, cannot calibrate");
        return;
    }

    hal_log("========================================");
    hal_log("  STEERING ANGLE CALIBRATION");
    hal_log("========================================");
    Serial.println();
    Serial.println("=== Lenkwinkel Kalibrierung ===");
    Serial.println();

    // --- LEFT STOP ---
    Serial.println("1) Lenkung ganz nach LINKS fahren (linker Anschlag)");
    Serial.println("   ENTER druecken zum Speichern des Wertes:");
    Serial.flush();
    wait_for_enter_live_adc();

    // Read 11 samples, take median
    int16_t left_val = ads1118_read_raw_median(11, 8);
    float v_left = left_val * 4.096f / 32768.0f;
    hal_log("SteerCal: left stop  -> raw=%d, %.3f V", left_val, v_left);
    Serial.printf("   Gespeichert: %7.3f V  (raw=%d)\n", v_left, left_val);
    Serial.println();

    // --- RIGHT STOP ---
    Serial.println("2) Lenkung ganz nach RECHTS fahren (rechter Anschlag)");
    Serial.println("   ENTER druecken zum Speichern des Wertes:");
    Serial.flush();
    wait_for_enter_live_adc();

    int16_t right_val = ads1118_read_raw_median(11, 8);
    float v_right = right_val * 4.096f / 32768.0f;
    hal_log("SteerCal: right stop -> raw=%d, %.3f V", right_val, v_right);
    Serial.printf("   Gespeichert: %7.3f V  (raw=%d)\n", v_right, right_val);
    Serial.println();

    // --- Validate ---
    if (left_val == right_val) {
        hal_log("SteerCal: ERROR — left == right (%d), no steering range!", left_val);
        Serial.println("FEHLER: Links und Rechts sind gleich! Nochmal versuchen.");
        Serial.println();
        return;
    }

    // Ensure left < right (swap if poti is wired in reverse)
    if (left_val > right_val) {
        int16_t tmp = left_val;
        left_val = right_val;
        right_val = tmp;
        hal_log("SteerCal: values swapped (left > right), poti wiring reversed");
        Serial.println("   Hinweis: Poti polaritaet automatisch korrigiert");
    }

    s_cal_left_raw = left_val;
    s_cal_right_raw = right_val;
    s_calibrated = true;

    // Save to NVS
    steer_cal_save();

    // Show summary
    int16_t span = right_val - left_val;
    float voltage_left  = left_val  * 4.096f / 32768.0f;
    float voltage_right = right_val * 4.096f / 32768.0f;

    Serial.println("=== Kalibrierung abgeschlossen ===");
    Serial.printf("   Links:  raw=%6d  (%.3f V)  -> -45.0°\n", left_val, voltage_left);
    Serial.printf("   Rechts: raw=%6d  (%.3f V)  -> +45.0°\n", right_val, voltage_right);
    Serial.printf("   Spanne: %d LSB  (%.3f V)\n", span, voltage_right - voltage_left);
    Serial.println();
    Serial.println("Werte im Flash gespeichert. Kalibrierung ueberlebt Neustart.");
    Serial.println("========================================");
    Serial.println();
}

float hal_steer_angle_read_deg(void) {

    if (!s_ads1118_detected || !s_calibrated) {
        return 0.0f;  // Not detected or not calibrated — return neutral
    }

    // Read raw ADC value directly (bypasses libdriver's config read-back)
    int16_t raw = ads1118_read_raw();

    // Map raw ADC to -22.5°..+22.5° using calibrated min/max
    int16_t span = s_cal_right_raw - s_cal_left_raw;
    if (span == 0) return 0.0f;

    // Normalise to 0.0..1.0 within calibrated range
    float normalised = static_cast<float>(raw - s_cal_left_raw) / static_cast<float>(span);

    // Clamp to [0, 1]
    if (normalised < 0.0f) normalised = 0.0f;
    if (normalised > 1.0f) normalised = 1.0f;

    // Map 0..1 -> -22.5°..+22.5°
    float angle = (normalised * 45.0f) - 22.5f;

    return angle;
}

int16_t hal_steer_angle_read_raw(void) {

    if (!s_ads1118_detected || !s_calibrated) return 0;
    return ads1118_read_raw();
}

uint8_t hal_steer_angle_read_sensor_byte(void) {
    if (!s_ads1118_detected) return 0;

    const int16_t raw = ads1118_read_raw();
    const int16_t span = s_cal_right_raw - s_cal_left_raw;
    if (s_calibrated && span != 0) {
        float normalised = static_cast<float>(raw - s_cal_left_raw) / static_cast<float>(span);
        if (normalised < 0.0f) normalised = 0.0f;
        if (normalised > 1.0f) normalised = 1.0f;
        return static_cast<uint8_t>((normalised * 255.0f) + 0.5f);
    }

    return static_cast<uint8_t>(raw & 0xFF);
}

bool hal_steer_angle_is_calibrated(void) {

    return s_calibrated;
}

// ===================================================================
// Actuator - SPI Bus 2 (FSPI / SPI2_HOST)
// ===================================================================
void hal_actuator_begin(void) {
    pinMode(CS_ACT, OUTPUT);
    digitalWrite(CS_ACT, HIGH);
    hal_log("ESP32: Actuator begun on CS=%d (stub)", CS_ACT);
}

bool hal_actuator_detect(void) {
    // Actuator is write-only, hard to verify by reading back.
    // Just verify the SPI bus works by attempting a transfer.
    uint8_t tx = 0x00;
    uint8_t response = 0;
    spiTransfer(SpiClient::ACTUATOR, &tx, &response, 1);

    bool detected = true;  // Stub: assume OK (actuator is write-only)
    hal_log("ESP32: Actuator detect: SPI %s", detected ? "OK" : "FAIL");
    return detected;
}

void hal_actuator_write(uint16_t cmd) {
    uint8_t tx[2] = {
        static_cast<uint8_t>((cmd >> 8) & 0xFF),
        static_cast<uint8_t>(cmd & 0xFF)
    };
    spiTransfer(SpiClient::ACTUATOR, tx, nullptr, sizeof(tx));
}

// ===================================================================
// Network - W5500 Ethernet via ESP-IDF ETH driver
//
// Uses ETH.begin() to initialise the W5500 on SPI3_HOST with the
// pins defined by the LilyGO board design.  The ETH driver handles
// SPI communication internally - no manual SPI setup needed.
//
// Link status and IP assignment are tracked via WiFi.onEvent()
// callbacks (ARDUINO_EVENT_ETH_CONNECTED / ARDUINO_EVENT_ETH_GOT_IP).
// ===================================================================

/// WiFi event handler for Ethernet events
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
static void onEthEvent(WiFiEvent_t event, arduino_event_info_t info) {
#else
// Arduino 2.x uses older event signature
static void onEthEvent(WiFiEvent_t event) {
#endif
    switch (event) {
    case ARDUINO_EVENT_ETH_START:
        hal_log("ETH: driver started");
        ETH.setHostname("agsteer");
        break;

    case ARDUINO_EVENT_ETH_CONNECTED:
        hal_log("ETH: link UP (%s %u Mbps)",
                ETH.fullDuplex() ? "full-duplex" : "half-duplex",
                ETH.linkSpeed());
        s_eth_link_up = true;
        break;

    case ARDUINO_EVENT_ETH_GOT_IP:
        hal_log("ETH: got IP %s  (MAC: %s)",
                ETH.localIP().toString().c_str(),
                ETH.macAddress().c_str());
        s_eth_has_ip = true;

        // Start receive UDP listener on port 8888 (AgIO sends to this port)
        // Reference: ether.udpServerListenOnPort(&udpSteerRecv, 8888)
        ethUDP_recv.begin(aog_port::AGIO_LISTEN);
        hal_log("ETH: UDP listening on port %u (AgIO sends here)", aog_port::AGIO_LISTEN);

        // Start send UDP socket from port 5126 (our source port)
        // Reference: portMy = 5126
        ethUDP_send.begin(aog_port::STEER);
        hal_log("ETH: UDP sending from port %u (to AgIO port %u)", aog_port::STEER, aog_port::AGIO_SEND);
        break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
        hal_log("ETH: link DOWN");
        s_eth_link_up = false;
        s_eth_has_ip = false;
        ethUDP_recv.stop();
        ethUDP_send.stop();
        break;

    case ARDUINO_EVENT_ETH_STOP:
        hal_log("ETH: driver stopped");
        s_eth_link_up = false;
        s_eth_has_ip = false;
        ethUDP_recv.stop();
        ethUDP_send.stop();
        break;

    default:
        break;
    }
}

void hal_net_init(void) {
    // Register Ethernet event handler
    WiFi.onEvent(onEthEvent);

    hal_log("ETH: initialising W5500 on SPI3_HOST (SCK=%d MISO=%d MOSI=%d CS=%d INT=%d RST=%d)...",
            ETH_SCK, ETH_MISO, ETH_MOSI, ETH_CS, ETH_INT, ETH_RST);

    // Initialise W5500 via ESP-IDF ETH driver
    // Parameters: phy_type, phy_addr, cs, irq, rst, spi_host, sck, miso, mosi
    bool init_ok = ETH.begin(
        ETH_PHY_W5500,   // PHY type
        1,                // PHY address (must be 1 for this board)
        ETH_CS,           // Chip Select    = GPIO 9
        ETH_INT,          // Interrupt      = GPIO 13
        ETH_RST,          // Reset          = GPIO 14
        SPI3_HOST,        // SPI peripheral
        ETH_SCK,          // SPI Clock      = GPIO 10
        ETH_MISO,         // SPI MISO       = GPIO 11
        ETH_MOSI          // SPI MOSI       = GPIO 12
    );

    if (!init_ok) {
        hal_log("ETH: FAILED - W5500 not detected! Check SPI connections.");
        s_w5500_detected = false;
        return;
    }

    s_w5500_detected = true;
    hal_log("ETH: W5500 chip detected OK");

    // Configure static IP address
    if (!ETH.config(s_local_ip, s_gateway, s_subnet, s_dns, s_dns)) {
        hal_log("ETH: static IP config failed (may be using DHCP)");
    }

    // Wait up to 5 seconds for link-up and IP assignment
    uint32_t wait_start = millis();
    while (!s_eth_has_ip && (millis() - wait_start < 5000)) {
        delay(100);
        // Feed the event loop so ETH events can be processed
        yield();
    }

    if (s_eth_has_ip) {
        hal_log("ETH: ready - IP=%s", ETH.localIP().toString().c_str());
    } else if (s_eth_link_up) {
        hal_log("ETH: link up but no IP yet (waiting for DHCP...)");
    } else {
        hal_log("ETH: WARNING - no link detected (cable unplugged?)");
    }
}

void hal_net_send(const uint8_t* data, size_t len, uint16_t port) {
    if (!s_eth_has_ip) return;

    // Always send TO AgIO port 9999 (port parameter is legacy, ignored)
    // Reference: portDestination = 9999
    ethUDP_send.beginPacket(s_dest_ip, aog_port::AGIO_SEND);
    ethUDP_send.write(data, static_cast<size_t>(len));
    ethUDP_send.endPacket();
}

void hal_net_set_dest_ip(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3) {
    s_dest_ip = IPAddress(ip0, ip1, ip2, ip3);
    hal_log("HAL: dest IP updated to %u.%u.%u.%u", ip0, ip1, ip2, ip3);
}

int hal_net_receive(uint8_t* buf, size_t max_len, uint16_t* out_port) {
    if (!s_eth_has_ip) return 0;

    int packet_size = ethUDP_recv.parsePacket();
    if (packet_size <= 0) return 0;

    if (static_cast<size_t>(packet_size) > max_len) {
        packet_size = static_cast<int>(max_len);
    }

    int read = ethUDP_recv.read(buf, packet_size);
    if (out_port) {
        *out_port = static_cast<uint16_t>(ethUDP_recv.remotePort());
    }
    return read;
}

bool hal_net_is_connected(void) {
    return s_eth_has_ip;
}

bool hal_net_detected(void) {
    return s_w5500_detected;
}

// ===================================================================
// ESP32 init all
// ===================================================================
static void hal_esp32_common_boot_init(void) {
    // Serial
    Serial.begin(115200);
    uint32_t serial_start = millis();
    while (!Serial && (millis() - serial_start < 3000)) {
        delay(10);
    }

    // Redirect ESP-IDF log (ESP_LOGI etc.) to USB CDC Serial.
    // Without this, esp_log goes to UART0 while Serial.println
    // goes to USB CDC — user would only see half the output.
    Serial.setDebugOutput(true);

    hal_log("ESP32-S3 AgSteer starting...");

    // Mutex
    hal_mutex_init();

    // Safety pin
    pinMode(SAFETY_IN, INPUT_PULLUP);

    // SPI sensor bus (FSPI / SPI2_HOST) - SCK=16, MISO=15, MOSI=17
    hal_sensor_spi_init();
}

void hal_esp32_init_imu_bringup(void) {
    hal_esp32_common_boot_init();

    // IMU + steering-angle ADC for SPI cross-device diagnostics.
    // Keep actuator/network disabled in bring-up mode.
    hal_imu_begin();
    hal_imu_reset_pulse(10, 20);
    hal_steer_angle_begin();
    hal_log("ESP32: IMU bring-up HAL init complete (ADS enabled, actuator/network skipped)");
}

void hal_esp32_init_all(void) {
    hal_esp32_common_boot_init();

    // IMU, steer angle, actuator
    hal_imu_begin();
    hal_steer_angle_begin();
    hal_actuator_begin();

    // Network (W5500 via ETH driver)
    hal_net_init();

    hal_log("ESP32: all subsystems initialised (%s)",
            s_eth_has_ip ? "ETH UP" :
            s_w5500_detected ? "ETH no link" :
            "W5500 not found");
}
