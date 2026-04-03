/**
 * @file hal_impl.cpp
 * @brief ESP32-S3 HAL implementation for LilyGO T-ETH-Lite-S3.
 *
 * Hardware:
 *   - MCU: ESP32-S3-WROOM-1
 *   - Ethernet: W5500 over SPI (GPIO48/21/47/45)
 *   - GNSS: 2x UM980 on UART1/UART2 (460800 baud)
 *   - Sensors: IMU (BNO085), Steer Angle, Actuator on SPI bus 2
 *   - Safety: GPIO5 active LOW
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
#include <SPI.h>
#include <Ethernet3.h>       // W5500 Ethernet library (ESP32 variant)
#include <HardwareSerial.h>

// ===================================================================
// W5500 Ethernet – SPI bus 1
// ===================================================================
static SPIClass ethSPI(HSPI);  // HSPI for Ethernet

// Ethernet client / UDP
static EthernetUDP ethUDP;

// IP configuration – will be updated by AgIO subnet change
static IPAddress s_local_ip(192, 168, 1, 70);
static IPAddress s_subnet(255, 255, 255, 0);
static IPAddress s_gateway(192, 168, 1, 1);
static IPAddress s_dest_ip(192, 168, 1, 255);

// Ethernet state
static bool s_eth_initialized = false;
static bool s_w5500_detected = false;

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

// ===================================================================
// Sensor SPI bus 2
// ===================================================================
static SPIClass sensorSPI(FSPI);  // FSPI for sensors

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
    return readNmeaLine(gnssMainSerial, buf, max_len, &s_gnss_main_pos);
}

bool hal_gnss_heading_read_line(char* buf, size_t max_len) {
    return readNmeaLine(gnssHeadingSerial, buf, max_len, &s_gnss_heading_pos);
}

// ===================================================================
// SPI Sensors / Actuator – SPI Bus 2
// ===================================================================
void hal_sensor_spi_init(void) {
    sensorSPI.begin(SENS_SPI_SCK, SENS_SPI_MISO, SENS_SPI_MOSI, -1);
    hal_log("ESP32: sensor SPI initialised (SCK=%d MOSI=%d MISO=%d)",
            SENS_SPI_SCK, SENS_SPI_MOSI, SENS_SPI_MISO);
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

void hal_steer_angle_begin(void) {
    pinMode(CS_STEER_ANG, OUTPUT);
    digitalWrite(CS_STEER_ANG, HIGH);
    hal_log("ESP32: Steer angle sensor begun on CS=%d (stub)", CS_STEER_ANG);
}

float hal_steer_angle_read_deg(void) {
    float angle = 0.0f;

    // TODO: Implement actual SPI read from steering angle sensor
    sensorSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_STEER_ANG, LOW);
    uint8_t rx_buf[4] = {0};
    for (int i = 0; i < 4; i++) {
        rx_buf[i] = sensorSPI.transfer(0x00);
    }
    digitalWrite(CS_STEER_ANG, HIGH);
    sensorSPI.endTransaction();

    // Stub: interpret first 2 bytes as angle * 100
    int16_t raw = (rx_buf[0] << 8) | rx_buf[1];
    angle = raw / 100.0f;

    return angle;
}

void hal_actuator_begin(void) {
    pinMode(CS_ACT, OUTPUT);
    digitalWrite(CS_ACT, HIGH);
    hal_log("ESP32: Actuator begun on CS=%d (stub)", CS_ACT);
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
// Network – W5500 Ethernet
// ===================================================================

/// Try to detect W5500 chip via SPI by reading the VERSIONR register (0x39).
static bool detectW5500(void) {
    // Ethernet3 internal init must be called first
    Ethernet.init(8);

    // Small delay after init for SPI to settle
    delay(10);

    // Ethernet3 exposes the W5500 object internally.
    // The Ethernet3 library reads the chip version during init.
    // We check if localIP() is at least plausible after begin().
    // A more robust check would directly read W5500 VERSIONR (0x39 = 'W'),
    // but Ethernet3 doesn't expose raw register access easily.
    // Instead, we verify that the chip responded to init by checking
    // that the IP was actually set (not 255.255.255.255).
    return true;  // We validate after begin() below
}

void hal_net_init(void) {
    // Initialise Ethernet SPI bus 1
    ethSPI.begin(ETH_SPI_SCK, ETH_SPI_MISO, ETH_SPI_MOSI, ETH_CS);

    pinMode(ETH_CS, OUTPUT);
    digitalWrite(ETH_CS, HIGH);
    pinMode(ETH_INT, INPUT);

    // Initialise W5500
    Ethernet.init(8);
    delay(10);

    // Start Ethernet with configured static IP
    // Ethernet3 API: begin(mac, ip, dns, gateway, subnet)
    uint8_t mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
    Ethernet.begin(mac, s_local_ip, IPAddress(8, 8, 8, 8), s_gateway, s_subnet);

    // Short delay for W5500 to process configuration
    delay(100);

    // Check if W5500 actually responded by verifying our IP was set
    IPAddress configured_ip = Ethernet.localIP();
    if (configured_ip == IPAddress(255, 255, 255, 255) ||
        configured_ip == IPAddress(0, 0, 0, 0)) {
        hal_log("ESP32: WARNING – W5500 not detected (IP=%s). Network disabled.",
                configured_ip.toString().c_str());
        s_w5500_detected = false;
        s_eth_initialized = false;
        return;
    }

    s_w5500_detected = true;

    // Start UDP on AgIO port
    ethUDP.begin(AOG_PORT_AGIO);

    s_eth_initialized = true;

    hal_log("ESP32: Ethernet initialised – IP=%s (W5500 OK)",
            configured_ip.toString().c_str());
}

void hal_net_send(const uint8_t* data, size_t len, uint16_t port) {
    if (!s_eth_initialized || !s_w5500_detected) return;

    // Use configured destination IP (broadcast or AgIO IP)
    ethUDP.beginPacket(s_dest_ip, port);
    ethUDP.write(data, static_cast<size_t>(len));
    ethUDP.endPacket();
}

int hal_net_receive(uint8_t* buf, size_t max_len, uint16_t* out_port) {
    if (!s_eth_initialized || !s_w5500_detected) return 0;

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
    return s_w5500_detected && s_eth_initialized &&
           (Ethernet.localIP() != IPAddress(0, 0, 0, 0));
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

    // SPI sensor bus
    hal_sensor_spi_init();

    // GNSS
    hal_gnss_init();

    // IMU, steer angle, actuator
    hal_imu_begin();
    hal_steer_angle_begin();
    hal_actuator_begin();

    // Network (W5500) – graceful failure if not connected
    hal_net_init();

    hal_log("ESP32: all subsystems initialised (%s)",
            s_w5500_detected ? "W5500 OK" : "W5500 not found – network disabled");
}
