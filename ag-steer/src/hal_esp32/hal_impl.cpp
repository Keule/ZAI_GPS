/**
 * @file hal_impl.cpp
 * @brief ESP32-S3 HAL implementation for LilyGO T-ETH-Lite-S3.
 *
 * Hardware:
 *   - MCU: ESP32-S3-WROOM-1
 *   - Ethernet: W5500 over SPI3_HOST (GPIO10/11/12/9) via ESP-IDF ETH driver
 *   - GNSS: 2x UM980 on UART1/UART2 (460800 baud)
 *   - Sensors: IMU (BNO085), Steer Angle, Actuator on SPI2_HOST (FSPI)
 *   - Safety: GPIO4 active LOW
 *
 * W5500 Ethernet uses the ESP-IDF ETH driver:
 *   - Arduino ESP32 Core >= 3.0.0: native <ETH.h>
 *   - Arduino ESP32 Core <  3.0.0: LilyGO ETHClass2 library
 * NOT the Arduino Ethernet/Ethernet3 library.
 *
 * This file includes Arduino/ESP32 headers – it must NEVER be included
 * from PC simulation code.
 */

#include "hal_impl.h"
#include "hal/hal.h"
#include "hardware_pins.h"
#include "logic/aog_udp_protocol.h"

// ===================================================================
// Arduino / ESP32 includes
// ===================================================================
#include <Arduino.h>
#include <SPI.h>           // SPIClass for sensor bus (FSPI)
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

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
// W5500 Ethernet – ESP-IDF ETH driver (SPI3_HOST)
// ===================================================================

// UDP socket for AgIO communication
static WiFiUDP ethUDP;

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
// GNSS UARTs – HardwareSerial
// ===================================================================
static HardwareSerial gnssMainSerial(1);
static HardwareSerial gnssHeadingSerial(2);

// GNSS line buffers
static char s_gnss_main_buf[256];
static size_t s_gnss_main_pos = 0;
static char s_gnss_heading_buf[256];
static size_t s_gnss_heading_pos = 0;

// GNSS data received flags (for hardware detection)
static volatile bool s_gnss_main_has_data = false;
static volatile bool s_gnss_heading_has_data = false;

// ===================================================================
// Shared SPI bus – FSPI / SPI2_HOST
//
// CRITICAL: Must use FSPI, NOT HSPI!
// On ESP32-S3 (Arduino Core 2.x):  HSPI = SPI3_HOST (occupied by W5500!)
//                                  FSPI = SPI2_HOST (free for sensors + SD)
//
// All SPI devices share this bus: SD card, ADS1118, IMU, actuator.
// Pins: SCK=7, MISO=5, MOSI=6 (same for all devices).
// Each device has its own CS pin (42, 39, 38, 40).
// ===================================================================
static SPIClass sensorSPI(FSPI);

// ===================================================================
// Mutex (FreeRTOS recursive mutex)
// ===================================================================
#if configSUPPORT_STATIC_ALLOCATION
static StaticSemaphore_t s_mutex_buffer;
static SemaphoreHandle_t s_mutex = nullptr;
#else
static SemaphoreHandle_t s_mutex = nullptr;
#endif

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
// Logging
// ===================================================================
void hal_log(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.printf("[%10lu] %s\n", millis(), buf);
}

// ===================================================================
// Mutex
// ===================================================================
void hal_mutex_init(void) {
#if configSUPPORT_STATIC_ALLOCATION
    s_mutex = xSemaphoreCreateRecursiveMutexStatic(&s_mutex_buffer);
#else
    s_mutex = xSemaphoreCreateRecursiveMutex();
#endif
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
// GNSS UART
// ===================================================================
void hal_gnss_init(void) {
    // GNSS MAIN: UART1
    gnssMainSerial.begin(GNSS_BAUD_RATE, SERIAL_8N1, GNSS_MAIN_RX, GNSS_MAIN_TX);
    s_gnss_main_pos = 0;

    // GNSS HEADING: UART2
    gnssHeadingSerial.begin(GNSS_BAUD_RATE, SERIAL_8N1, GNSS_HEADING_RX, GNSS_HEADING_TX);
    s_gnss_heading_pos = 0;

    hal_log("ESP32: GNSS UARTs initialised (%d baud)", GNSS_BAUD_RATE);
}

/// Read available characters from a serial and try to form a complete NMEA line.
static bool readNmeaLine(HardwareSerial& ser, char* buf, size_t max_len, size_t* pos) {
    // Limit reads per call to avoid blocking the scheduler
    int max_reads = 64;
    while (ser.available() > 0 && max_reads-- > 0) {
        int c = ser.read();
        if (c < 0) break;

        if (c == '\r') continue;  // skip CR
        if (c == '\n') {
            if (*pos > 0) {
                buf[*pos] = '\0';
                *pos = 0;
                return true;
            }
            continue;
        }

        if (*pos < max_len - 1) {
            buf[*pos++] = static_cast<char>(c);
        } else {
            // Buffer overflow – reset
            *pos = 0;
        }
    }
    return false;
}

bool hal_gnss_main_read_line(char* buf, size_t max_len) {
    bool result = readNmeaLine(gnssMainSerial, buf, max_len, &s_gnss_main_pos);
    if (result) s_gnss_main_has_data = true;
    return result;
}

bool hal_gnss_heading_read_line(char* buf, size_t max_len) {
    bool result = readNmeaLine(gnssHeadingSerial, buf, max_len, &s_gnss_heading_pos);
    if (result) s_gnss_heading_has_data = true;
    return result;
}

bool hal_gnss_main_detect(void) {
    return s_gnss_main_has_data;
}

bool hal_gnss_heading_detect(void) {
    return s_gnss_heading_has_data;
}

void hal_gnss_reset_detection(void) {
    s_gnss_main_has_data = false;
    s_gnss_heading_has_data = false;
}

// ===================================================================
// SPI Sensors / Actuator – SPI Bus 2 (FSPI / SPI2_HOST)
// ===================================================================
void hal_sensor_spi_init(void) {
    sensorSPI.begin(SENS_SPI_SCK, SENS_SPI_MISO, SENS_SPI_MOSI, -1);
    hal_log("ESP32: shared SPI initialised on FSPI/SPI2_HOST (SCK=%d MISO=%d MOSI=%d)",
            SENS_SPI_SCK, SENS_SPI_MISO, SENS_SPI_MOSI);
}

void hal_sensor_spi_deinit(void) {
    sensorSPI.end();
    hal_log("ESP32: shared SPI released (FSPI peripheral free)");
}

void hal_sensor_spi_reinit(void) {
    sensorSPI.begin(SENS_SPI_SCK, SENS_SPI_MISO, SENS_SPI_MOSI, -1);
    hal_log("ESP32: shared SPI re-initialised on FSPI/SPI2_HOST");
}

void hal_imu_begin(void) {
    pinMode(CS_IMU, OUTPUT);
    digitalWrite(CS_IMU, HIGH);
    // TODO: Send BNO085 reset/initialise sequence over SPI
    hal_log("ESP32: IMU begun on CS=%d (stub)", CS_IMU);
}

bool hal_imu_read(float* yaw_rate_dps, float* roll_deg) {
    // TODO: Implement BNO085 SH-2 SPI read protocol
    *yaw_rate_dps = 0.0f;
    *roll_deg = 0.0f;
    return true;
}

bool hal_imu_detect(void) {
    // BNO085 detection: read chip ID from register 0x00
    // Expected chip ID: 0x00 (BNO085 responds to SPI reset)
    // For now, verify SPI bus is responsive by attempting a transfer
    sensorSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_IMU, LOW);
    uint8_t response = sensorSPI.transfer(0x00);
    digitalWrite(CS_IMU, HIGH);
    sensorSPI.endTransaction();

    // 0xFF = floating MISO (no device pulling it down)
    // 0x00 = MISO stuck LOW (bus fault)
    // A real BNO085 would respond with its chip ID.
    // For now: detect as OK if response is NOT 0xFF (floating) and NOT 0x00 (fault).
    // TODO: When real BNO085 is connected, check actual chip ID.
    bool detected = (response != 0xFF && response != 0x00);
    hal_log("ESP32: IMU detect: SPI response=0x%02X %s",
            response, detected ? "OK" : "FAIL (no device)");
    return detected;
}

// ===================================================================
// ADS1118 – 16-Bit ADC for steering angle potentiometer
// ===================================================================
// Connected via SPI Bus 2 (FSPI / SPI2_HOST), CS = GPIO 39.
//
// ADS1118 SPI protocol (NOT standard SPI slave!):
//   The ADS1118 uses a fixed 16-bit frame per CS cycle.
//   Config and data are transmitted SIMULTANEOUSLY:
//     - While DIN receives config (16 bits), DOUT outputs the
//       previous conversion result (16 bits).
//     - CS HIGH → starts new conversion (if OS bit = 1)
//
//   IMPORTANT: Only 16 bits per CS cycle! Sending more bytes
//   overwrites the config register with garbage.
//
// SPI Mode: TI datasheet says Mode 1 (CPOL=0, CPHA=1) only.
//   We try both Mode 0 and Mode 1 during detection and use
//   whichever returns valid data.
//
// Timing (single-shot mode, 128 SPS):
//   Conversion time = 1/128 = 7.8 ms
//   Must wait >= 8 ms after CS HIGH before next CS LOW.
//   Control loop at 200 Hz (5 ms) → read every other cycle.
//
// Config register (16 bit, MSB first):
//   Bit 15    : OS  = 1 (start single-shot conversion)
//   Bits 14-12: MUX = 000 (AIN0 vs GND)
//   Bits 11-9 : PGA = 001 (±4.096V range)
//   Bit 8     : MODE = 1 (single-shot, not continuous)
//   Bits 7-5  : DR  = 100 (128 SPS)
//   Bit 4     : CM  = 0 (standard mode)
//   Bit 3     : TS  = 0 (ADC mode, not temp sensor)
//   Bit 2     : POL = 0 (MSB first)
//   Bits 1-0  : CQ  = 11 (comparator disabled)
//
// Config = 0x8383  (1000 0011 1000 0011)
//
// ADC result: 16-bit signed, MSB first.
//   0xFFFF = conversion not ready / busy
//   ±4.096V → 1 LSB = 0.125 mV
//   0-3.3V poti → raw 0 to ~26880
// ===================================================================

/// ADS1118 config: AIN0, ±4.096V, single-shot, 128 SPS
static const uint16_t ADS1118_CONFIG =
    (1u << 15) |  // OS: start single-shot
    (0u << 12) |  // MUX: AIN0 vs GND
    (1u <<  9) |  // PGA: ±4.096V
    (1u <<  8) |  // MODE: single-shot
    (4u <<  5) |  // DR: 128 SPS
    (0u <<  4) |  // CM: standard
    (0u <<  3) |  // TS: ADC mode
    (0u <<  2) |  // POL: MSB first
    (3u <<  0);   // CQ: comparator disabled
// = 0x8383

/// 1 LSB in volts (±4.096V over 32768 steps)
static const float ADS1118_LSB_V = 0.000125f;  // 4.096 / 32768 = 125 µV = 0.000125 V

/// Minimum time between CS HIGH and next CS LOW (ms)
/// 128 SPS = 7.8125 ms, use 9 ms for safety margin
static const uint32_t ADS1118_CONV_MS = 9;

/// Timestamp of last CS HIGH (start of conversion)
static uint32_t s_ads_last_conv_start = 0;

/// Last valid raw ADC reading
static int16_t s_ads_last_raw = 0;

/// SPI mode that worked during detection (SPI_MODE0 or SPI_MODE1)
static uint8_t s_ads_spi_mode = SPI_MODE0;

/// Whether ADS1118 was detected successfully
static bool s_ads_detected = false;

/// Whether DOUT data must be bit-inverted (some cheap modules use
/// transistor level-shifters that invert the signal on DOUT).
/// Detected automatically during hal_steer_angle_detect().
static bool s_ads_invert_dout = false;

// ===================================================================
// Optional: Isolated ADS1118 test on separate pins (debug only)
//
// Uncomment the following line to run an isolated ADS1118 test
// on GPIO 44/45/46/48 at boot, BEFORE any other SPI device is touched.
// This tests the ADS1118 module on a completely clean, dedicated bus
// with no SD card, IMU, or actuator to interfere.
//
// Wiring for this test:
//   ADS1118 DOUT  -> GPIO 44
//   ADS1118 DIN   -> GPIO 45
//   ADS1118 SCLK  -> GPIO 46
//   ADS1118 CS    -> GPIO 48
//   ADS1118 VDD   -> 3.3V
//   ADS1118 GND   -> GND
//   ADS1118 AIN0  -> poti wiper
// ===================================================================
// #define ADS1118_ISOLATED_TEST

#ifdef ADS1118_ISOLATED_TEST
static void ads1118IsolatedTest(void) {
    hal_log(\"=== ADS1118 ISOLATED TEST (GPIO 44/45/46/48) ===\");

    // Deselect all shared-bus devices to prevent interference
    pinMode(SD_CS, OUTPUT);        digitalWrite(SD_CS, HIGH);
    pinMode(CS_IMU, OUTPUT);       digitalWrite(CS_IMU, HIGH);
    pinMode(CS_STEER_ANG, OUTPUT); digitalWrite(CS_STEER_ANG, HIGH);
    pinMode(CS_ACT, OUTPUT);       digitalWrite(CS_ACT, HIGH);

    // Set up isolated SPI on test pins
    SPIClass testSPI(FSPI);
    testSPI.begin(ADS_TEST_SCK, ADS_TEST_MISO, ADS_TEST_MOSI, ADS_TEST_CS);
    hal_log(\"Test SPI started (SCK=%d MISO=%d MOSI=%d CS=%d)\",
            ADS_TEST_SCK, ADS_TEST_MISO, ADS_TEST_MOSI, ADS_TEST_CS);

    // DOUT connectivity test (send 0x55, check if MISO echoes it)
    testSPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));
    digitalWrite(ADS_TEST_CS, LOW);
    uint8_t tb1 = testSPI.transfer(0x55);
    uint8_t tb2 = testSPI.transfer(0x55);
    digitalWrite(ADS_TEST_CS, HIGH);
    testSPI.endTransaction();

    if (tb1 == 0x55 && tb2 == 0x55) {
        hal_log(\"Test DOUT: CROSSTALK - DIN/DOUT swapped (read 0x%02X 0x%02X)\", tb1, tb2);
    } else if (tb1 == 0xFF && tb2 == 0xFF) {
        hal_log(\"Test DOUT: FLOATING - no device on GPIO%d\", ADS_TEST_MISO);
    } else {
        hal_log(\"Test DOUT: OK (read 0x%02X 0x%02X)\", tb1, tb2);
    }

    // Full detection cycle on both SPI modes, with and without inversion
    for (int mode_idx = 0; mode_idx < 2; mode_idx++) {
        uint8_t mode = (mode_idx == 0) ? SPI_MODE0 : SPI_MODE1;
        const char* mname = (mode_idx == 0) ? \"Mode0\" : \"Mode1\";

        // Send config, start conversion
        testSPI.beginTransaction(SPISettings(200000, MSBFIRST, mode));
        digitalWrite(ADS_TEST_CS, LOW);
        testSPI.transfer(static_cast<uint8_t>((ADS1118_CONFIG >> 8) & 0xFF));
        testSPI.transfer(static_cast<uint8_t>(ADS1118_CONFIG & 0xFF));
        digitalWrite(ADS_TEST_CS, HIGH);
        testSPI.endTransaction();

        // Wait for conversion
        hal_delay_ms(ADS1118_CONV_MS + 3);

        // Read result
        testSPI.beginTransaction(SPISettings(200000, MSBFIRST, mode));
        digitalWrite(ADS_TEST_CS, LOW);
        uint8_t msb = testSPI.transfer(static_cast<uint8_t>((ADS1118_CONFIG >> 8) & 0xFF));
        uint8_t lsb = testSPI.transfer(static_cast<uint8_t>(ADS1118_CONFIG & 0xFF));
        digitalWrite(ADS_TEST_CS, HIGH);
        testSPI.endTransaction();

        int16_t raw      = static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
        int16_t raw_inv  = static_cast<int16_t>(~raw);

        float v_raw = static_cast<float>(raw) * ADS1118_LSB_V;
        float v_inv = static_cast<float>(raw_inv) * ADS1118_LSB_V;

        hal_log(\"Test %s: raw=%d (0x%04X) = %.4fV  |  inv=%d (0x%04X) = %.4fV\",
                mname, raw, static_cast<uint16_t>(raw), v_raw,
                raw_inv, static_cast<uint16_t>(raw_inv), v_inv);
    }

    // Clean up isolated SPI
    testSPI.end();
    hal_log(\"=== ISOLATED TEST COMPLETE ===\");
}
#endif // ADS1118_ISOLATED_TEST

/**
 * Perform one ADS1118 SPI transaction (16 bits).
 *
 * Sends config to DIN while reading previous conversion result from DOUT.
 * CS HIGH starts a new conversion.
 *
 * If s_ads_invert_dout is true, the received bytes are bit-inverted
 * before interpretation (compensates for modules with inverting level
 * shifters on the DOUT line).
 *
 * @return 16-bit raw ADC value (0xFFFF = conversion not ready)
 */
static int16_t ads1118Transaction(void) {
    sensorSPI.beginTransaction(SPISettings(1000000, MSBFIRST, s_ads_spi_mode));
    digitalWrite(CS_STEER_ANG, LOW);

    // 16-bit simultaneous transfer: config OUT, result IN
    uint8_t msb = sensorSPI.transfer(static_cast<uint8_t>((ADS1118_CONFIG >> 8) & 0xFF));
    uint8_t lsb = sensorSPI.transfer(static_cast<uint8_t>(ADS1118_CONFIG & 0xFF));

    digitalWrite(CS_STEER_ANG, HIGH);
    sensorSPI.endTransaction();

    s_ads_last_conv_start = millis();

    // Apply bit inversion if the module's DOUT line is inverted
    if (s_ads_invert_dout) {
        msb = ~msb;
        lsb = ~lsb;
    }

    return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
}

/**
 * DOUT connectivity test.
 *
 * Sends a distinctive 0x55 0x55 pattern (01010101) on MOSI.
 * If DOUT is correctly on MISO, the ADS1118 drives its own data
 * (previous conversion result) which will NOT be 0x55.
 * If DIN/DOUT are swapped, MISO picks up the MOSI signal and
 * reads back approximately 0x55 0x55 (crosstalk).
 * If DOUT is not connected at all, MISO reads 0xFF 0xFF (floating).
 *
 * @return true if DOUT appears to be correctly connected (device responds)
 */
static bool ads1118CheckDoutConnection(void) {
    // Use slow clock for reliable bit-bang-level test
    sensorSPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_STEER_ANG, LOW);
    uint8_t b1 = sensorSPI.transfer(0x55);  // distinctive pattern
    uint8_t b2 = sensorSPI.transfer(0x55);
    digitalWrite(CS_STEER_ANG, HIGH);
    sensorSPI.endTransaction();

    bool crosstalk  = (b1 == 0x55 || b2 == 0x55);
    bool floating   = (b1 == 0xFF && b2 == 0xFF);

    if (crosstalk) {
        hal_log("ESP32: ADS1118 DOUT test: CROSSTALK (read 0x%02X 0x%02X)", b1, b2);
        hal_log("ESP32:   DIN/DOUT cables are SWAPPED!");
        hal_log("ESP32:   Correct: DOUT -> GPIO%d (MISO)  DIN -> GPIO%d (MOSI)",
                SENS_SPI_MISO, SENS_SPI_MOSI);
        return false;
    } else if (floating) {
        hal_log("ESP32: ADS1118 DOUT test: FLOATING (read 0xFF 0xFF)");
        hal_log("ESP32:   No device driving MISO – check wiring");
        hal_log("ESP32:   DOUT must be connected to GPIO%d (MISO)", SENS_SPI_MISO);
        return false;
    } else {
        hal_log("ESP32: ADS1118 DOUT test: OK (read 0x%02X 0x%02X)", b1, b2);
        return true;
    }
}

/**
 * Check if a raw ADC value looks like MOSI→MISO crosstalk.
 *
 * Crosstalk patterns to detect:
 *   - Low byte matches config low byte  (0x83)
 *   - High byte matches config high byte (0x83)
 *   - Entire value matches config word     (0x8383)
 *   - Value is close to 0x5555 or 0x0000 (test pattern echo)
 */
static bool ads1118IsCrosstalk(int16_t raw) {
    uint16_t uval = static_cast<uint16_t>(raw);
    uint16_t lo = uval & 0x00FF;
    uint16_t hi = (uval >> 8) & 0x00FF;

    // Config word bytes appear in the reading
    if (lo == 0x83 || hi == 0x83) return true;

    // Continuous-mode config echo (0x03 in high byte)
    if (hi == 0x03) return true;

    // Test pattern echo
    if (uval == 0x5555 || uval == 0x0000) return true;

    return false;
}

void hal_steer_angle_begin(void) {
    pinMode(CS_STEER_ANG, OUTPUT);
    digitalWrite(CS_STEER_ANG, HIGH);

    // Deselect SD card slot and other SPI devices to prevent bus interference.
    // The SD library may leave SD_CS floating after SD.end().
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    pinMode(CS_IMU, OUTPUT);
    digitalWrite(CS_IMU, HIGH);
    pinMode(CS_ACT, OUTPUT);
    digitalWrite(CS_ACT, HIGH);

    hal_delay_ms(1);

    hal_log("ESP32: ADS1118 on CS=%d (AIN0, +/-4.096V, 128 SPS)", CS_STEER_ANG);

    // === DOUT CONNECTIVITY TEST ===
    // Before attempting full detection, verify DOUT is on the correct pin.
    // This prevents false-positive detection with crosstalk data.
    ads1118CheckDoutConnection();
}

bool hal_steer_angle_detect(void) {
    // Ensure SD card and other SPI devices are deselected before detection.
    // Prevents residual pull from interfering with MISO.
    digitalWrite(SD_CS, HIGH);
    digitalWrite(CS_IMU, HIGH);
    digitalWrite(CS_ACT, HIGH);

    // === ADS1118 DETECTION ===
    // Strategy: try both SPI modes, pick the one that returns valid data.
    // Use slow clock (200 kHz) for reliable detection.
    // Full cycle per mode: send config → wait → read result.

    struct ModeResult {
        uint8_t mode;
        const char* name;
        int16_t raw;
        bool valid;
        bool inverted;
    };

    ModeResult results[2] = {
        { SPI_MODE0, "Mode0", 0, false, false },
        { SPI_MODE1, "Mode1", 0, false, false },
    };

    for (int i = 0; i < 2; i++) {
        uint8_t mode = results[i].mode;

        // Step 1: send config, start conversion (ignore garbage result)
        sensorSPI.beginTransaction(SPISettings(200000, MSBFIRST, mode));
        digitalWrite(CS_STEER_ANG, LOW);
        sensorSPI.transfer(static_cast<uint8_t>((ADS1118_CONFIG >> 8) & 0xFF));
        sensorSPI.transfer(static_cast<uint8_t>(ADS1118_CONFIG & 0xFF));
        digitalWrite(CS_STEER_ANG, HIGH);
        sensorSPI.endTransaction();

        // Step 2: wait for conversion to complete
        hal_delay_ms(ADS1118_CONV_MS + 3);

        // Step 3: read conversion result
        sensorSPI.beginTransaction(SPISettings(200000, MSBFIRST, mode));
        digitalWrite(CS_STEER_ANG, LOW);
        uint8_t msb = sensorSPI.transfer(static_cast<uint8_t>((ADS1118_CONFIG >> 8) & 0xFF));
        uint8_t lsb = sensorSPI.transfer(static_cast<uint8_t>(ADS1118_CONFIG & 0xFF));
        digitalWrite(CS_STEER_ANG, HIGH);
        sensorSPI.endTransaction();

        int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
        int16_t raw_inv = static_cast<int16_t>(~raw);  // bit-inverted

        // Choose: inverted or not?
        // Some cheap modules have inverting level-shifters on DOUT.
        // Heuristic: prefer the interpretation that gives a value within
        // the ±4.096V range (0..32768 for unipolar poti on 0-3.3V)
        // and does NOT look like crosstalk.
        bool raw_crosstalk = ads1118IsCrosstalk(raw);
        bool inv_crosstalk = ads1118IsCrosstalk(raw_inv);
        bool raw_ok = (raw != static_cast<int16_t>(0xFFFF)) && !raw_crosstalk;
        bool inv_ok = (raw_inv != static_cast<int16_t>(0xFFFF)) && !inv_crosstalk;

        // Prefer inverted if raw is crosstalk but inverted is clean,
        // OR if inverted gives a positive value while raw is negative
        // (assuming poti input is 0-3.3V → positive ADC reading).
        bool use_inverted = false;
        if (inv_ok && !raw_ok) {
            use_inverted = true;
        } else if (raw_ok && inv_ok) {
            // Both pass crosstalk check — prefer the one with
            // a value in the expected 0-3.3V range
            if (raw < 0 && raw_inv >= 0) use_inverted = true;
        }

        results[i].raw = use_inverted ? raw_inv : raw;
        results[i].inverted = use_inverted;
        results[i].valid = (results[i].raw != static_cast<int16_t>(0xFFFF));

        float voltage = static_cast<float>(results[i].raw) * ADS1118_LSB_V;
        hal_log("ESP32: ADS1118 %s%s: raw=%d (0x%04X), voltage=%.4fV %s",
                results[i].name,
                use_inverted ? " (inv)" : "",
                results[i].raw,
                static_cast<uint16_t>(results[i].raw),
                voltage,
                results[i].valid ? "OK" : "BAD");
    }

    // Pick the best result
    int16_t best_raw = 0;
    uint8_t best_mode = SPI_MODE0;
    bool any_valid = false;

    for (int i = 0; i < 2; i++) {
        if (results[i].valid) {
            any_valid = true;
            best_raw = results[i].raw;
            best_mode = results[i].mode;
            break;  // Mode0 preferred if both valid
        }
    }

    if (any_valid) {
        s_ads_spi_mode = best_mode;
        s_ads_last_raw = best_raw;
        s_ads_detected = true;

        // Store the inversion flag from the winning mode
        for (int i = 0; i < 2; i++) {
            if (results[i].mode == best_mode) {
                s_ads_invert_dout = results[i].inverted;
                break;
            }
        }

        float voltage = static_cast<float>(best_raw) * ADS1118_LSB_V;
        hal_log("ESP32: ADS1118 DETECTED OK (%s%s, raw=%d, %.4fV)",
                (best_mode == SPI_MODE0) ? "Mode0" : "Mode1",
                s_ads_invert_dout ? ", invert-DOUT" : "",
                best_raw, voltage);
        return true;
    }

    // === DETECTION FAILED ===
    s_ads_detected = false;

    // For failure diagnostics, re-evaluate raw vs inverted
    bool any_crosstalk = false;
    bool any_floating = false;
    for (int i = 0; i < 2; i++) {
        uint16_t u = static_cast<uint16_t>(results[i].raw);
        if (u == 0xFFFF) any_floating = true;
        if (!results[i].inverted && ads1118IsCrosstalk(results[i].raw)) any_crosstalk = true;
        if (results[i].inverted && ads1118IsCrosstalk(static_cast<int16_t>(~results[i].raw))) any_crosstalk = true;
    }

    if (any_crosstalk) {
        hal_log("ESP32: ADS1118 DETECT FAILED – crosstalk in both modes");
        hal_log("ESP32: ROOT CAUSE: DIN/DOUT cables are SWAPPED");
        hal_log("ESP32: ACTION: Swap the two data cables on the ADS1118 module:");
        hal_log("ESP32:   DOUT -> GPIO%d (MISO)  DIN -> GPIO%d (MOSI)",
                SENS_SPI_MISO, SENS_SPI_MOSI);
    } else if (any_floating) {
        hal_log("ESP32: ADS1118 DETECT FAILED – no response (0xFFFF)");
        hal_log("ESP32: ROOT CAUSE: ADS1118 not communicating");
        hal_log("ESP32: Check: (1) DOUT connected to GPIO%d?", SENS_SPI_MISO);
        hal_log("ESP32:        (2) DIN connected to GPIO%d?", SENS_SPI_MOSI);
        hal_log("ESP32:        (3) ADS1118 powered (VDD=3.3V)?");
        hal_log("ESP32:        (4) CS jumper to GPIO%d?", CS_STEER_ANG);
    } else {
        hal_log("ESP32: ADS1118 DETECT FAILED – unknown response");
    }

    return false;
}

float hal_steer_angle_read_deg(void) {
    if (!s_ads_detected) {
        return 0.0f;  // Not detected – return neutral
    }

    // Check if enough time has passed for the conversion to complete
    uint32_t elapsed = millis() - s_ads_last_conv_start;
    if (elapsed >= ADS1118_CONV_MS) {
        // Conversion ready – read result and start next conversion
        int16_t raw = ads1118Transaction();

        // 0xFFFF means conversion not ready or bus error
        if (raw != static_cast<int16_t>(0xFFFF)) {
            s_ads_last_raw = raw;
        }
        // else: keep previous value
    }
    // else: not enough time, return last known value

    int16_t raw = s_ads_last_raw;

    // Convert to voltage
    float voltage = static_cast<float>(raw) * ADS1118_LSB_V;

    // Normalise to 0.0 .. 1.0 (3.3V poti supply)
    float normalised = voltage / 3.3f;

    // Clamp
    if (normalised < 0.0f) normalised = 0.0f;
    if (normalised > 1.0f) normalised = 1.0f;

    // Map 0..1 → -45°..+45°
    float angle = (normalised * 90.0f) - 45.0f;

    return angle;
}

void hal_actuator_begin(void) {
    pinMode(CS_ACT, OUTPUT);
    digitalWrite(CS_ACT, HIGH);
    hal_log("ESP32: Actuator begun on CS=%d (stub)", CS_ACT);
}

bool hal_actuator_detect(void) {
    // Actuator is write-only, hard to verify by reading back.
    // Just verify the SPI bus works by attempting a transfer.
    sensorSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_ACT, LOW);
    uint8_t response = sensorSPI.transfer(0x00);
    digitalWrite(CS_ACT, HIGH);
    sensorSPI.endTransaction();

    bool detected = true;  // Stub: assume OK (actuator is write-only)
    hal_log("ESP32: Actuator detect: SPI %s", detected ? "OK" : "FAIL");
    return detected;
}

void hal_actuator_write(uint16_t cmd) {
    sensorSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_ACT, LOW);
    sensorSPI.transfer(static_cast<uint8_t>((cmd >> 8) & 0xFF));
    sensorSPI.transfer(static_cast<uint8_t>(cmd & 0xFF));
    digitalWrite(CS_ACT, HIGH);
    sensorSPI.endTransaction();
}

// ===================================================================
// Network – W5500 Ethernet via ESP-IDF ETH driver
//
// Uses ETH.begin() to initialise the W5500 on SPI3_HOST with the
// pins defined by the LilyGO board design.  The ETH driver handles
// SPI communication internally – no manual SPI setup needed.
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
        // Start UDP listener on AgIO port
        ethUDP.begin(AOG_PORT_AGIO);
        hal_log("ETH: UDP listening on port %u", AOG_PORT_AGIO);
        break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
        hal_log("ETH: link DOWN");
        s_eth_link_up = false;
        s_eth_has_ip = false;
        ethUDP.stop();
        break;

    case ARDUINO_EVENT_ETH_STOP:
        hal_log("ETH: driver stopped");
        s_eth_link_up = false;
        s_eth_has_ip = false;
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
        SPI3_HOST,        // SPI peripheral (FSPI on ESP32-S3)
        ETH_SCK,          // SPI Clock      = GPIO 10
        ETH_MISO,         // SPI MISO       = GPIO 11
        ETH_MOSI          // SPI MOSI       = GPIO 12
    );

    if (!init_ok) {
        hal_log("ETH: FAILED – W5500 not detected! Check SPI connections.");
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
        hal_log("ETH: ready – IP=%s", ETH.localIP().toString().c_str());
    } else if (s_eth_link_up) {
        hal_log("ETH: link up but no IP yet (waiting for DHCP...)");
    } else {
        hal_log("ETH: WARNING – no link detected (cable unplugged?)");
    }
}

void hal_net_send(const uint8_t* data, size_t len, uint16_t port) {
    if (!s_eth_has_ip) return;

    ethUDP.beginPacket(s_dest_ip, port);
    ethUDP.write(data, static_cast<size_t>(len));
    ethUDP.endPacket();
}

int hal_net_receive(uint8_t* buf, size_t max_len, uint16_t* out_port) {
    if (!s_eth_has_ip) return 0;

    int packet_size = ethUDP.parsePacket();
    if (packet_size <= 0) return 0;

    if (static_cast<size_t>(packet_size) > max_len) {
        packet_size = static_cast<int>(max_len);
    }

    int read = ethUDP.read(buf, packet_size);
    if (out_port) {
        *out_port = static_cast<uint16_t>(ethUDP.remotePort());
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
void hal_esp32_init_all(void) {
    // Serial – with timeout instead of infinite wait
    Serial.begin(115200);
    uint32_t serial_start = millis();
    while (!Serial && (millis() - serial_start < 3000)) {
        delay(10);
    }
    hal_log("ESP32-S3 AgSteer starting...");

    // Mutex
    hal_mutex_init();

    // Safety pin
    pinMode(SAFETY_IN, INPUT_PULLUP);

#ifdef ADS1118_ISOLATED_TEST
    // Isolated test runs BEFORE shared-bus SPI is initialised.
    // This gives it exclusive use of the FSPI peripheral.
    ads1118IsolatedTest();
#endif

    // SPI sensor bus (FSPI / SPI2_HOST)
    hal_sensor_spi_init();

    // GNSS
    hal_gnss_init();

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
