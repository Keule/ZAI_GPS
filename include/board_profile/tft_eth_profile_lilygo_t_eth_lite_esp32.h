#pragma once

// PIN-Profil fuer LilyGO T-ETH-Lite-ESP32


#define BOARD_PROFILE_NAME "lilygo_t_eth_lite_esp32"

#define PIN_GND 100
#define PIN_5V 150
#define PIN_3V3 133
#define PIN_RST 101 
#define PIN_BT 0

#define PIN_L_1 PIN_3V3 //3V3
#define PIN_L_2 39 //IO
#define PIN_L_3 34 //SD_MISO
#define PIN_L_4 13 //SD_MOSI
#define PIN_L_5 32 //IO
#define PIN_L_6 14 //SD_SCLK
#define PIN_L_7 34 //SD_MISO
#define PIN_L_8 33 //IO
#define PIN_L_9 14 //SD_SCLK
#define PIN_L_10 13 //SD_MOSI
#define PIN_L_11 4 //IO
#define PIN_L_12 2 //IO
#define PIN_L_13 15 //IO
#define PIN_L_14 PIN_GND //GND
#define PIN_L_15 PIN_3V3 //3V3
 
 
#define PIN_R_1 PIN_5V //5V
#define PIN_R_2 PIN_GND //GND
#define PIN_R_3 1 //TXD
#define PIN_R_4 3 //RXD
#define PIN_R_5 PIN_RST //RST
#define PIN_R_6 PIN_BT //BOOT
#define PIN_R_7 5 //Out only
#define PIN_R_8 35 //IO
#define PIN_R_9 38 //Out only
#define PIN_R_10 PIN_GND //GND
#define PIN_R_11 PIN_5V //5V
#define PIN_R_12 PIN_5V //5V
#define PIN_R_13 PIN_GND //GND
#define PIN_R_14 PIN_5V //5V
#define PIN_R_15 PIN_GND //GND
 
#define ETH_TYPE                        ETH_PHY_RTL8201
#define ETH_ADDR                        0
#define ETH_CLK_MODE                    ETH_CLOCK_GPIO0_IN
#define ETH_RESET_PIN                   -1
#define ETH_MDC_PIN                     23
#define ETH_POWER_PIN                   12
#define ETH_MDIO_PIN                    18

#define SD_MISO_PIN                     34
#define SD_MOSI_PIN                     13
#define SD_SCLK_PIN                     14
#define SD_CS_PIN                       5

#define SD_SPI_SCK     SD_SCLK_PIN      // SPI clock for SD card
#define SD_SPI_MISO    SD_MISO_PIN      // SPI MISO for SD card
#define SD_SPI_MOSI    SD_MOSI_PIN      // SPI MOSI for SD card
#define SD_CS          SD_CS_PIN        // SD card slot
 
// #define SPI_FREQUENCY  27000000
#define SPI_FREQUENCY  40000000

#define SPI_READ_FREQUENCY  20000000

#define SPI_TOUCH_FREQUENCY  2500000

// ---------------------------------------------------------------------------
// SPI Bus 2: Sensor Bus (FSPI = SPI2_HOST)
//
// ADS1118, IMU, and Actuator share this bus with different CS pins.
// SCK/MISO/MOSI are on GPIO 16/15/17 respectively.
// ---------------------------------------------------------------------------
#define SENS_SPI_SCK   15     // SPI clock
#define SENS_SPI_MISO  34     // SPI MISO (data from devices to ESP32)
#define SENS_SPI_MOSI  35     // SPI MOSI (data from ESP32 to devices)

// ---------------------------------------------------------------------------
// Current IMU Wiring (single source of truth)
// ---------------------------------------------------------------------------
#define IMU_INT        39    // BNO085 INT pin (input to ESP32-S3)
#define IMU_RST        38
#define CS_IMU         13    // BNO085 IMU
#define IMU_WAKE       14    // BNO085 PS0/WAKE (set HIGH before reset for SPI mode)

// Chip Selects (active LOW) - GPIOs 38-42 are output-only, which is fine for CS/control.

#define CS_STEER_ANG   34    // ADS1118 ADC (steer angle potentiometer)
#define CS_ACT         05    // Actuator driver



// ---------------------------------------------------------------------------
// Safety input (active LOW)
// ---------------------------------------------------------------------------
#define SAFETY_IN       15

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
#define GNSS_UART1_TX   2
#define GNSS_UART1_RX   4
#define GNSS_UART2_TX    33
#define GNSS_UART2_RX    35

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
#define LOG_SWITCH_PIN   05