/**
 * @file hardware_pins.h
 * @brief Central pin definitions for LilyGO T-ETH-Lite-S3 (ESP32-S3 + W5500)
 *
 * Target: ESP32-S3-WROOM-1 with W5500 Ethernet over SPI.
 * No RMII variant, no SC16IS752 UART bridge.
 *
 * Two SPI buses:
 *   SPI-1 = Ethernet (W5500)
 *   SPI-2 = Sensors / Actuator (IMU, Steer Angle, Actuator)
 *
 * Two UARTs for GNSS modules.
 */

#pragma once

// ---------------------------------------------------------------------------
// SPI Bus 1: Ethernet (W5500 only)
// ---------------------------------------------------------------------------
#define ETH_SPI_SCK   48
#define ETH_SPI_MOSI  21
#define ETH_SPI_MISO  47
#define ETH_CS        45
#define ETH_INT       14

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
// SPI Bus 2: Sensors / Actuator
// ---------------------------------------------------------------------------
#define SENS_SPI_SCK   12
#define SENS_SPI_MOSI  11
#define SENS_SPI_MISO  13

// ---------------------------------------------------------------------------
// Chip Selects (SPI Bus 2 devices)
// ---------------------------------------------------------------------------
#define CS_IMU          10
#define CS_STEER_ANG    9
#define CS_ACT          8

// ---------------------------------------------------------------------------
// Interrupt / Status lines
// ---------------------------------------------------------------------------
#define IMU_INT         6
#define STEER_ANG_INT   7

// ---------------------------------------------------------------------------
// Safety input (active LOW)
// ---------------------------------------------------------------------------
#define SAFETY_IN       5
