/**
 * @file LILYGO_T_ETH_LITE_ESP32_S3_board_pins.h
 * @brief Central pin definitions for LilyGO T-ETH-Lite-S3 (ESP32-S3 + W5500)
 *
 * Target: ESP32-S3-WROOM-1-N8R8 (16 MB Flash, 8 MB Octal PSRAM) + W5500 Ethernet.
 *
 * Pin mapping verified against official LilyGO repository:
 *   https://github.com/Xinyuan-LilyGO/LilyGO-T-ETH-Series
 *   (utilities.h, #ifdef LILYGO_T_ETH_LITE_ESP32S3)
 *
 * ========================================================================
 * SPI BUS ALLOCATION  (CRITICAL - wrong bus = crash!)
 * ========================================================================
 *   SPI3_HOST = W5500 Ethernet  (onboard, GPIO  9/10/11/12/13/14)
 *   SPI2_HOST = Sensor bus      (on FSPI)
 *              -> ADS1118 ADC (steer angle) 
 *              -> BNO085 IMU
 *              -> Actuator driver
 *
 *   SD Card uses SPI2_HOST (FSPI) too, but with DIFFERENT pins (5/6/7).
 *   During normal operation the sensor bus owns FSPI.
 *   For SD card access (OTA at boot), FSPI is re-initialised with SD pins,
 *   then restored to sensor pins after the update.
 *
 * On ESP32-S3 in Arduino Core 2.x:
 *   FSPI = SPI2_HOST   HSPI = SPI3_HOST
 *
 * The sensor bus MUST use FSPI (= SPI2_HOST), NOT HSPI (= SPI3_HOST)!
 * HSPI is the same physical bus as SPI3_HOST which the W5500 ETH driver
 * uses. Sharing the bus between ETH driver and SPIClass causes a hard
 * crash: "assert failed: spi_hal_setup_trans ... spi_ll_get_running_cmd"
 *
 * GPIO NOTE - ESP32-S3R8 (with Octal PSRAM):
 *   GPIOs 26-37 are INTERNALLY occupied by the Octal PSRAM interface.
 *   They are NOT available as general-purpose GPIOs.
 *   GPIOs 38-42 are OUTPUT-ONLY (no input capability).
 *   GPIOs 43/44 are UART0 default pins - AVOID for external UARTs!
 *   GPIOs 45-48 are full bidirectional GPIOs.
 * ========================================================================
 *
 * GPIO assignment - by function:
 *
 *   W5500 Ethernet (SPI3_HOST):  9  10  11  12  13  14  (fixed by board)
 *   SD Card (FSPI, OTA only):    5   6   7  42
 */

#pragma once
#include <cstdint>
#define BOARD_PROFILE_NAME "lilygo_t_eth_lite_esp32_s3"
// ---------------------------------------------------------------------------
// SPI Bus 1: Ethernet - W5500 (managed by ESP-IDF ETH driver on SPI3_HOST)
// These pins are fixed by the board design - DO NOT CHANGE.
// ---------------------------------------------------------------------------



#define ETH_CS         9
#define ETH_SCK        10
#define ETH_MISO       11
#define ETH_MOSI       12
#define ETH_INT        13
#define ETH_RST        14

// ---------------------------------------------------------------------------
// SPI Bus 2: Sensor Bus (FSPI = SPI2_HOST)
//
// ADS1118, IMU, and Actuator share this bus with different CS pins.
// SCK/MISO/MOSI are on GPIO 16/15/17 respectively.
// ---------------------------------------------------------------------------
#define SENS_SPI_SCK   47     // SPI clock
#define SENS_SPI_MISO  21     // SPI MISO (data from devices to ESP32)
#define SENS_SPI_MOSI  38     // SPI MOSI (data from ESP32 to devices)

// Guard against accidental duplicate IMU pin definitions from build flags or
// other headers. This file is the single source of truth for IMU wiring.
#if defined(IMU_RST) || defined(IMU_INT) || defined(CS_IMU) || defined(IMU_WAKE)
  #error "IMU pins already defined elsewhere; keep IMU pin mapping canonical in hardware_pins.h"
#endif

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

// Optional GNSS sideband lines (not wired on current board revision)
#define GNSS1_PPS_PIN   -1
#define GNSS1_EN_PIN    -1
#define GNSS2_PPS_PIN   -1
#define GNSS2_EN_PIN    -1

// GNSS console mirror defaults (diagnostic read-only sniffing in gnss_buildup)
inline constexpr uint32_t GNSS_MIRROR_BAUD = 115200;
inline constexpr int8_t GNSS_MIRROR_UART1_RX_PIN = GNSS_UART1_RX;
inline constexpr int8_t GNSS_MIRROR_UART1_TX_PIN = GNSS_UART1_TX;
inline constexpr int8_t GNSS_MIRROR_UART2_RX_PIN = GNSS_UART2_RX;
inline constexpr int8_t GNSS_MIRROR_UART2_TX_PIN = GNSS_UART2_TX;

// ---------------------------------------------------------------------------
// GNSS Receiver configuration — TASK-025
// Compile-time receiver list (board-specific).
// GNSS_RX_MAX is defined in hal.h (default: 2).
// ---------------------------------------------------------------------------

// NOTE: GNSS_RX_CONFIGS is not constexpr here because it depends on
// GNSS_RX_MAX which may differ per build environment.
// The actual configuration is applied in ntrip.cpp via ntripInit().
// Default configuration for this board (ESP32-S3):
//   inst 0: UART1 (TX48/RX45) = primary GNSS receiver
//   inst 1: UART2 (TX2/RX1)  = secondary GNSS receiver

// Logging switch (active LOW, internal pull-up)
//
// GPIO 46 is an ESP32-S3 strapping pin, but works for this active-LOW input:
// floating/LOW is safe for serial download mode and normal boot ignores it.
// Connect a toggle switch between GPIO 46 and GND.
//   Switch OFF (open)  -> pin pulled HIGH -> logging disabled
//   Switch ON (closed) -> pin pulled LOW  -> logging enabled
// ---------------------------------------------------------------------------
#define LOG_SWITCH_PIN   46
