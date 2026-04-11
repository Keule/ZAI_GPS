/**
 * @file hardware_pins.h
 * @brief Central pin definitions for LilyGO T-ETH-Lite-S3 (ESP32-S3 + W5500)
 *
 * NOTE: The canonical version of this file is at include/hardware_pins.h.
 * This copy exists at the project root for backward compatibility with
 * source files that include "hardware_pins.h" via the default search path.
 *
 * If you edit this file, also update include/hardware_pins.h to match.
 */

#pragma once

// SPI Bus 1: Ethernet - W5500 (SPI3_HOST)
#define ETH_SCK        10
#define ETH_MISO       11
#define ETH_MOSI       12
#define ETH_CS          9
#define ETH_INT        13
#define ETH_RST        14

// SPI Bus 2: Sensor Bus (FSPI = SPI2_HOST)
#define SENS_SPI_SCK   16
#define SENS_SPI_MISO  15
#define SENS_SPI_MOSI  17

#define CS_IMU          38
#define CS_STEER_ANG   18
#define CS_ACT         40

// SD Card (FSPI, OTA only)
#define SD_SPI_SCK     7
#define SD_SPI_MISO    5
#define SD_SPI_MOSI    6
#define SD_CS          42

// UART: GNSS Heading
#define GNSS_HEADING_TX  44
#define GNSS_HEADING_RX  43

// UART: GNSS Main
#define GNSS_MAIN_TX  46
#define GNSS_MAIN_RX  45

#define GNSS_BAUD_RATE   460800

#define IMU_INT        48
#define SAFETY_IN       4
#define LOG_SWITCH_PIN   47

#define SD_FW_FILE_PRIMARY   "/firmware.bin"
#define SD_FW_FILE_ALT       "/update.bin"
#define SD_FW_VERSION_FILE   "/firmware.version"
