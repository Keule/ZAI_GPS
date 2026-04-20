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
 *              -> BNO085 IMU                     CS=47
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
 *   GPIOs 43/44 are UART0 default pins - AVOID for external UARTs!
 *   GPIOs 45-48 are full bidirectional GPIOs.
 * ========================================================================
 *
 * GPIO assignment - by function:
 *
 *   W5500 Ethernet (SPI3_HOST):  9  10  11  12  13  14  (fixed by board)
 *   ADS1118 + Sensors (FSPI):    15  16  17  18  38  40  47
 *   SD Card (FSPI, OTA only):    5   6   7  42
 *   Misc:                        4  45  46  48
 */

#pragma once

#include <cstdint>
#include "board_profile/board_profile_select.h"

// ---------------------------------------------------------------------------
// Firmware OTA files on SD card
// ---------------------------------------------------------------------------
#define SD_FW_FILE_PRIMARY   "/firmware.bin"
#define SD_FW_FILE_ALT       "/update.bin"

// Version file on SD card (optional - contains e.g. "1.2.3")
#define SD_FW_VERSION_FILE   "/firmware.version"
