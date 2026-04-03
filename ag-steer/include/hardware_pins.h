/**
 * @file hardware_pins.h
 * @brief Central pin definitions for LilyGO T-ETH-Lite-S3 (ESP32-S3 + W5500)
 *
 * Target: ESP32-S3-WROOM-1 with W5500 Ethernet over SPI.
 *
 * Pin mapping verified against official LilyGO repository:
 *   https://github.com/Xinyuan-LilyGO/LilyGO-T-ETH-Series
 *   (utilities.h, #ifdef LILYGO_T_ETH_LITE_ESP32S3)
 *
 * SPI buses:
 *   SPI3_HOST (FSPI) = W5500 Ethernet (onboard, GPIO 10/11/12)
 *   SPI2_HOST (HSPI) = Sensors / Actuator (external, configurable)
 *
 * Two UARTs for GNSS modules (UART1, UART2).
 */

#pragma once

// ---------------------------------------------------------------------------
// SPI Bus 1: Ethernet – W5500 (managed by ESP-IDF ETH driver on SPI3_HOST)
// These pins are fixed by the board design – DO NOT CHANGE.
// ---------------------------------------------------------------------------
#define ETH_SCK        10
#define ETH_MISO       11
#define ETH_MOSI       12
#define ETH_CS          9
#define ETH_INT        13
#define ETH_RST        14

// ---------------------------------------------------------------------------
// UART: GNSS Main (primary RTK rover)
// ---------------------------------------------------------------------------
#define GNSS_MAIN_TX  17
#define GNSS_MAIN_RX  18

// ---------------------------------------------------------------------------
// UART: GNSS Heading (second antenna / heading source)
// ---------------------------------------------------------------------------
#define GNSS_HEADING_TX  15
#define GNSS_HEADING_RX  16

// GNSS baud rate (both UARTs)
#define GNSS_BAUD_RATE   460800

// ---------------------------------------------------------------------------
// SPI Bus 2: Sensors / Actuator (HSPI / SPI2_HOST)
//
// NOTE: The W5500 uses SPI3_HOST (GPIO 10/11/12).  Sensors MUST use
//       SPI2_HOST (HSPI) on different pins to avoid bus contention.
//
// TODO: Adjust these pins to match your actual external wiring!
//       GPIOs 8, 21, 45, 46, 47, 48 are candidates (free on this board).
//       GPIOs 38, 39, 40, 41 are also available.
// ---------------------------------------------------------------------------
#define SENS_SPI_SCK   47
#define SENS_SPI_MOSI  48
#define SENS_SPI_MISO  21

// ---------------------------------------------------------------------------
// Chip Selects (SPI Bus 2 devices) – active LOW
// TODO: Adjust to match your actual wiring!
// ---------------------------------------------------------------------------
#define CS_IMU          8
#define CS_STEER_ANG   45
#define CS_ACT         46

// ---------------------------------------------------------------------------
// Interrupt / Status lines
// TODO: Adjust to match your actual wiring!
// ---------------------------------------------------------------------------
#define IMU_INT        38
#define STEER_ANG_INT  39

// ---------------------------------------------------------------------------
// Safety input (active LOW)
// TODO: Adjust to match your actual wiring!
// ---------------------------------------------------------------------------
#define SAFETY_IN       4
