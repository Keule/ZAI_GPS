#pragma once

// PIN Profil fuer LilyGO T-ETH-Lite-S3

#define BOARD_PROFILE_NAME "lilygo_t_eth_lite_esp32s3"

#define PIN_GND 100
#define PIN_5V 150
#define PIN_3V3 133
#define PIN_RST 101 
#define PIN_BT 0

#define PIN_L_1 PIN_3V3 //3V3
#define PIN_L_2 4 //IO
#define PIN_L_3 5 //SD_MISO
#define PIN_L_4 6 //SD_MOSI
#define PIN_L_5 7 //SD_SCLK
#define PIN_L_6 15 //IO
#define PIN_L_7 16 //IO
#define PIN_L_8 17 //IO
#define PIN_L_9 18 //IO
#define PIN_L_10 8 //IO
#define PIN_L_11 19 //IO
#define PIN_L_12 20 //IO
#define PIN_L_13 3 //IO
#define PIN_L_14 PIN_GND //GND
#define PIN_L_15 PIN_3V3 //3V3
 
 
#define PIN_R_1 PIN_5V //5V
#define PIN_R_2 PIN_GND //GND
#define PIN_R_3 43 //TXD
#define PIN_R_4 44 //RXD
#define PIN_R_5 PIN_RST //RST
#define PIN_R_6 PIN_BT //BOOT
#define PIN_R_7 1 //IO
#define PIN_R_8 2 //IO
#define PIN_R_9 41 //Out only
#define PIN_R_10 40 //Out only
#define PIN_R_11 39 //Out only
#define PIN_R_12 38 //Out only
#define PIN_R_13 21 //IO
#define PIN_R_14 46 //IO
#define PIN_R_15 PIN_GND //GND
#define PIN_R_16 PIN_5V //5V
 
#define PIN_E_1 48 //IO
#define PIN_E_2 46 //IO
#define PIN_E_3 47 //IO

// ---------------------------------------------------------------------------
// SD Card (FSPI = SPI2_HOST, OTA only)
//
// The SD card uses the SAME SPI peripheral (FSPI) but DIFFERENT pins.
// During normal operation FSPI is initialised with sensor pins (15/16/17).
// For OTA firmware updates, FSPI is re-initialised with SD pins (5/6/7)
// via hal_sensor_spi_deinit() / hal_sensor_spi_reinit().
// ---------------------------------------------------------------------------
#define SD_SPI_SCK     7      // SPI clock for SD card
#define SD_SPI_MISO    5      // SPI MISO for SD card
#define SD_SPI_MOSI    6      // SPI MOSI for SD card
#define SD_CS          42     // SD card slot
 
// ---------------------------------------------------------------------------
// SPI Bus 1: Ethernet - W5500 (managed by ESP-IDF ETH driver on SPI3_HOST)
// These pins are fixed by the board design - DO NOT CHANGE.
// ---------------------------------------------------------------------------

#define ETH_CS_PIN 9 
#define ETH_SCLK_PIN 10 
#define ETH_MISO_PIN 11 
#define ETH_MOSI_PIN 12 
#define ETH_INT 13 
#define ETH_RST 14 
#define ETH_ADDR -1 

#define ETH_SCK        ETH_SCLK_PIN
#define ETH_MISO       ETH_MISO_PIN
#define ETH_MOSI       ETH_MOSI_PIN
#define ETH_CS         ETH_CS_PIN
#define ETH_INT        ETH_INT
#define ETH_RST        ETH_RST

// ---------------------------------------------------------------------------
// SPI Bus 2: Sensor Bus (FSPI = SPI2_HOST)
//
// ADS1118, IMU, and Actuator share this bus with different CS pins.
// SCK/MISO/MOSI are on GPIO 16/15/17 respectively.
// ---------------------------------------------------------------------------
#define SENS_SPI_SCK   47     // SPI clock
#define SENS_SPI_MISO  21     // SPI MISO (data from devices to ESP32)
#define SENS_SPI_MOSI  38     // SPI MOSI (data from ESP32 to devices)

// ---------------------------------------------------------------------------
// Current IMU Wiring (single source of truth)
// ---------------------------------------------------------------------------
#define IMU_INT        46    // BNO085 INT pin (input to ESP32-S3)
#define IMU_RST        41
#define CS_IMU          40    // BNO085 IMU
#define IMU_WAKE       15    // BNO085 PS0/WAKE (set HIGH before reset for SPI mode)

// Chip Selects (active LOW) - GPIOs 38-42 are output-only, which is fine for CS/control.

#define CS_STEER_ANG   18    // ADS1118 ADC (steer angle potentiometer)
#define CS_ACT         16    // Actuator driver



// ---------------------------------------------------------------------------
// Safety input (active LOW)
// ---------------------------------------------------------------------------
#define SAFETY_IN       4

// ---------------------------------------------------------------------------
// GNSS UART bring-up matrix (TASK-019A)
//
// Board constraints (ESP32-S3R8):
//   - GPIO 26..37: reserved by Octal PSRAM (must not be used)
//   - GPIO 38..42: output-only (must not be used as UART RX)
//
// Selected assignment:
//   - UART1 (GNSS/RTCM primary): TX=48, RX=45
//   - UART2 (GNSS/Console mirror): TX=2, RX=1
// ---------------------------------------------------------------------------
#define GNSS_UART1_TX   48
#define GNSS_UART1_RX   45
#define GNSS_UART2_TX    2
#define GNSS_UART2_RX    1

// GNSS console mirror defaults (diagnostic read-only sniffing in gnss_buildup)
inline constexpr uint32_t GNSS_MIRROR_BAUD = 115200;
inline constexpr int8_t GNSS_MIRROR_UART1_RX_PIN = GNSS_UART1_RX;
inline constexpr int8_t GNSS_MIRROR_UART1_TX_PIN = GNSS_UART1_TX;
inline constexpr int8_t GNSS_MIRROR_UART2_RX_PIN = GNSS_UART2_RX;
inline constexpr int8_t GNSS_MIRROR_UART2_TX_PIN = GNSS_UART2_TX;
// ---------------------------------------------------------------------------
// Logging switch (active LOW, internal pull-up)
//
// GPIO 46 is an ESP32-S3 strapping pin, but works for this active-LOW input:
// floating/LOW is safe for serial download mode and normal boot ignores it.
// Connect a toggle switch between GPIO 46 and GND.
//   Switch OFF (open)  -> pin pulled HIGH -> logging disabled
//   Switch ON (closed) -> pin pulled LOW  -> logging enabled
// ---------------------------------------------------------------------------
#define LOG_SWITCH_PIN   46
