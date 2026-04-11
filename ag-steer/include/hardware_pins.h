/**
 * @file hardware_pins.h
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
 *   SPI2_HOST = Sensor bus      (GPIO 15/16/17 on FSPI)
 *              -> ADS1118 ADC (steer angle)   CS=18
 *              -> BNO085 IMU                     CS=38
 *              -> Actuator driver                CS=40
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
 *   GPIOs 43-48 are full bidirectional GPIOs.
 * ========================================================================
 *
 * GPIO assignment - by function:
 *
 *   W5500 Ethernet (SPI3_HOST):  9  10  11  12  13  14  (fixed by board)
 *   ADS1118 + Sensors (FSPI):    15  16  17  18  38  40
 *   GNSS UARTs:                 43  44  45  46
 *   SD Card (FSPI, OTA only):    5   6   7  42
 *   Misc:                        4  47  48
 */

#pragma once

// ---------------------------------------------------------------------------
// SPI Bus 1: Ethernet - W5500 (managed by ESP-IDF ETH driver on SPI3_HOST)
// These pins are fixed by the board design - DO NOT CHANGE.
// ---------------------------------------------------------------------------
#define ETH_SCK        10
#define ETH_MISO       11
#define ETH_MOSI       12
#define ETH_CS          9
#define ETH_INT        13
#define ETH_RST        14

// ---------------------------------------------------------------------------
// SPI Bus 2: Sensor Bus (FSPI = SPI2_HOST)
//
// ADS1118, IMU, and Actuator share this bus with different CS pins.
// SCK/MISO/MOSI are on GPIO 16/15/17 respectively.
// ---------------------------------------------------------------------------
#define SENS_SPI_SCK   16     // SPI clock
#define SENS_SPI_MISO  15     // SPI MISO (data from devices to ESP32)
#define SENS_SPI_MOSI  17     // SPI MOSI (data from ESP32 to devices)

// Chip Selects (active LOW) - GPIOs 38-42 are output-only, which is fine for CS.
#define CS_IMU          38    // BNO085 IMU
#define CS_STEER_ANG   18    // ADS1118 ADC (steer angle potentiometer)
#define CS_ACT         40    // Actuator driver

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
// UART: GNSS Heading (second antenna / heading source)
// GPIOs 43/44 - bidirectional, available on T-ETH-Lite-S3 header.
// ---------------------------------------------------------------------------
#define GNSS_HEADING_TX  44
#define GNSS_HEADING_RX  43

// ---------------------------------------------------------------------------
// UART: GNSS Main (primary RTK rover)
// GPIOs 45/46 - bidirectional, available on T-ETH-Lite-S3 header.
// ---------------------------------------------------------------------------
#define GNSS_MAIN_TX  46
#define GNSS_MAIN_RX  45

// GNSS baud rate (both UARTs)
#define GNSS_BAUD_RATE   460800

// ---------------------------------------------------------------------------
// IMU interrupt (BNO085 INT pin) - needs bidirectional GPIO for input!
// ---------------------------------------------------------------------------
#define IMU_INT        48    // GPIO 48 is bidirectional on ESP32-S3

// ---------------------------------------------------------------------------
// Safety input (active LOW)
// ---------------------------------------------------------------------------
#define SAFETY_IN       4

// ---------------------------------------------------------------------------
// Logging switch (active LOW, internal pull-up)
//
// GPIO 47 is a free bidirectional GPIO on the T-ETH-Lite-S3 header.
// Connect a toggle switch between GPIO 47 and GND.
//   Switch OFF (open)  -> pin pulled HIGH -> logging disabled
//   Switch ON (closed) -> pin pulled LOW  -> logging enabled
// ---------------------------------------------------------------------------
#define LOG_SWITCH_PIN   47

// ---------------------------------------------------------------------------
// Firmware OTA files on SD card
// ---------------------------------------------------------------------------
#define SD_FW_FILE_PRIMARY   "/firmware.bin"
#define SD_FW_FILE_ALT       "/update.bin"

// Version file on SD card (optional - contains e.g. "1.2.3")
#define SD_FW_VERSION_FILE   "/firmware.version"
