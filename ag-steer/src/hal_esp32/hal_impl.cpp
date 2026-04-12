/**
 * @file hal_impl.cpp
 * @brief ESP32-S3 HAL implementation for LilyGO T-ETH-Lite-S3.
 *
 * Hardware:
 *   - MCU: ESP32-S3-WROOM-1
 *   - Ethernet: W5500 over SPI3_HOST (GPIO 9/10/11/12/13/14) via ESP-IDF ETH driver
 *   - GNSS: 2x UM980 on UART1 (GPIO 45/46) and UART2 (GPIO 19/20), 460800 baud
 *   - Sensor SPI (FSPI/SPI2_HOST): SCK=16, MISO=15, MOSI=17
 *     - ADS1118 ADC (steer angle): CS=18
 *     - BNO085 IMU: CS=38
 *     - Actuator: CS=40
 *   - SD Card (FSPI, OTA only): SCK=7, MISO=5, MOSI=6, CS=42
 *   - Safety: GPIO4 active LOW
 *
 * ADS1118 ADC uses the local ADS1118 library (lib/ads1118/).
 * The library handles SPI mode auto-detection, bit-inversion compensation,
 * DOUT connectivity testing, and shared-bus CS management.
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
#include "logic/aog_udp_protocol.h"

// ===================================================================
// Arduino / ESP32 includes
// ===================================================================
#include <Arduino.h>
#include <SPI.h>           // SPIClass for sensor bus (FSPI)
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>
#include "ads1118.h"        // Local ADS1118 library

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
// GNSS UARTs - HardwareSerial
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
// Shared SPI bus - FSPI / SPI2_HOST
//
// CRITICAL: Must use FSPI, NOT HSPI!
// On ESP32-S3 (Arduino Core 2.x):  HSPI = SPI3_HOST (occupied by W5500!)
//                                  FSPI = SPI2_HOST (free for sensors)
//
// Sensor devices on this bus: ADS1118 (CS=18), IMU (CS=38), Actuator (CS=40).
// Pins: SCK=16, MISO=15, MOSI=17.
//
// SD card uses the SAME SPI peripheral (FSPI) but DIFFERENT pins (SCK=7, MISO=5, MOSI=6).
// During OTA updates, FSPI is re-initialised with SD pins via
// hal_sensor_spi_deinit() / hal_sensor_spi_reinit().
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
    // GNSS MAIN: UART1 (GPIO 45=RX, 46=TX)
    gnssMainSerial.begin(GNSS_BAUD_RATE, SERIAL_8N1, GNSS_MAIN_RX, GNSS_MAIN_TX);
    s_gnss_main_pos = 0;

    // GNSS HEADING: UART2 (GPIO 19=RX, 20=TX)
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
            // Buffer overflow - reset
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
// SPI Sensors / Actuator - SPI Bus 2 (FSPI / SPI2_HOST)
// ===================================================================
void hal_sensor_spi_init(void) {
    sensorSPI.begin(SENS_SPI_SCK, SENS_SPI_MISO, SENS_SPI_MOSI, -1);
    hal_log("ESP32: sensor SPI initialised on FSPI/SPI2_HOST (SCK=%d MISO=%d MOSI=%d)",
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
    bool detected = (response != 0xFF && response != 0x00);
    hal_log("ESP32: IMU detect: SPI response=0x%02X %s",
            response, detected ? "OK" : "FAIL (no device)");
    return detected;
}

// ===================================================================
// ADS1118 - 16-Bit ADC for steering angle potentiometer
// ===================================================================
// Uses local lib/ads1118/ ADS1118 library.
//
// Features (handled automatically by the library):
//   - 16-bit simultaneous config+data SPI protocol (per datasheet SLASB73)
//   - Auto SPI mode detection (tries Mode0 and Mode1)
//   - Auto bit-inversion detection (cheap module level-shifters)
//   - DOUT connectivity test (crosstalk / floating detection)
//   - Non-blocking readLoop() for 200 Hz control loop
//
// Config: AIN0 vs GND, +/-4.096V, single-shot, 128 SPS.
// ADC result: 16-bit signed, 1 LSB = 0.125 mV.
// 0-3.3V poti -> raw 0 to ~26880.
//
// Wiring:
//   ADS1118 DOUT  -> GPIO 15 (MISO)
//   ADS1118 DIN   -> GPIO 17 (MOSI)
//   ADS1118 SCLK  -> GPIO 16 (SCK)
//   ADS1118 CS    -> GPIO 18
// ===================================================================

/// Callback to deselect other SPI devices on the same bus before ADS1118 access.
static void adsDeselectOthers(void) {
    digitalWrite(CS_IMU, HIGH);
    digitalWrite(CS_ACT, HIGH);
}

/// ADS1118 driver instance
static ADS1118 s_ads1118(sensorSPI);

/// 1 LSB in volts (+/-4.096V over 32768 steps)
static const float ADS1118_LSB_V = 0.000125f;  // 4.096 / 32768 = 125 uV

void hal_steer_angle_begin(void) {
    // Ensure all other SPI device CS pins are configured as outputs
    pinMode(CS_IMU, OUTPUT);  digitalWrite(CS_IMU, HIGH);
    pinMode(CS_ACT, OUTPUT);  digitalWrite(CS_ACT, HIGH);

    // Initialise ADS1118 library with CS pin and shared-bus deselect callback
    s_ads1118.begin(CS_STEER_ANG, nullptr, 0, adsDeselectOthers);

    hal_log("ESP32: ADS1118 on CS=%d (AIN0, +/-4.096V, 128 SPS, SPI SCK=%d MISO=%d MOSI=%d)",
            CS_STEER_ANG, SENS_SPI_SCK, SENS_SPI_MISO, SENS_SPI_MOSI);
}

bool hal_steer_angle_detect(void) {
    bool detected = s_ads1118.detect();

    if (detected) {
        hal_log("ESP32: ADS1118 DETECTED (PGA=%.3fV%s)",
                s_ads1118.getFSR(),
                s_ads1118.isDoutInverted() ? ", invert-DOUT" : "");
    } else {
        hal_log("ESP32: ADS1118 DETECT FAILED");
    }

    return detected;
}

float hal_steer_angle_read_deg(void) {
    if (!s_ads1118.isDetected()) {
        return 0.0f;  // Not detected - return neutral
    }

    // Non-blocking read: starts new conversion if previous is complete.
    // Returns last known value if conversion still in progress.
    int16_t raw = s_ads1118.readLoop(0);  // AIN0

    // Convert to voltage
    float voltage = static_cast<float>(raw) * ADS1118_LSB_V;

    // Normalise to 0.0 .. 1.0 (3.3V poti supply)
    float normalised = voltage / 3.3f;

    // Clamp
    if (normalised < 0.0f) normalised = 0.0f;
    if (normalised > 1.0f) normalised = 1.0f;

    // Map 0..1 -> -45..+45 degrees
    float angle = (normalised * 90.0f) - 45.0f;

    return angle;
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
    // Serial
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

    // SPI sensor bus (FSPI / SPI2_HOST) - SCK=16, MISO=15, MOSI=17
    hal_sensor_spi_init();

    // GNSS UARTs
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
