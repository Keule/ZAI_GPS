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
 * ========================================================================
 * SPI BUS ALLOCATION  (CRITICAL – wrong bus = crash!)
 * ========================================================================
 *   SPI3_HOST = W5500 Ethernet  (onboard, GPIO  9/10/11/12/13/14)
 *   SPI2_HOST = Sensor bus      (external, GPIO 35/36/37/38/39/40/41)
 *
 * On ESP32-S3 in Arduino Core 2.x:
 *   FSPI = SPI2_HOST   HSPI = SPI3_HOST
 *
 * The sensor bus MUST use FSPI (= SPI2_HOST), NOT HSPI (= SPI3_HOST)!
 * HSPI is the same physical bus as SPI3_HOST which the W5500 ETH driver
 * uses. Sharing the bus between ETH driver and SPIClass causes a hard
 * crash: "assert failed: spi_hal_setup_trans ... spi_ll_get_running_cmd"
 * ========================================================================
 *
 * GPIO assignment – grouped for physical adjacency on the header:
 *
 *   Header row 1 (onboard):  9 10 11 12 13 14 = W5500 (fixed)
 *   Header row 2 (GNSS):    15 16 17 18 = UART2 RX/TX, UART1 RX/TX
 *   Header row 3 (sensors): 35 36 37 38 39 40 41 = SPI2 + CS + INT + Safety
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
// UART: GNSS Heading (second antenna / heading source)
// Physical grouping: GPIO 15/16 are adjacent on the header
// ---------------------------------------------------------------------------
#define GNSS_HEADING_TX  15
#define GNSS_HEADING_RX  16

// ---------------------------------------------------------------------------
// UART: GNSS Main (primary RTK rover)
// Physical grouping: GPIO 17/18 are adjacent on the header
// ---------------------------------------------------------------------------
#define GNSS_MAIN_TX  17
#define GNSS_MAIN_RX  18

// GNSS baud rate (both UARTs)
#define GNSS_BAUD_RATE   460800

// ---------------------------------------------------------------------------
// SPI Bus 2: Sensors / Actuator (FSPI = SPI2_HOST)
//
// MUST use FSPI on ESP32-S3 (Arduino Core 2.x) because HSPI = SPI3_HOST
// which is occupied by the W5500 ETH driver.
//
// GPIOs 35-41 are available on the header and physically adjacent:
//   35 36 37 38 39 40 41
//
// Pin assignment (physically grouped):
//   SPI bus:   35=SCK  36=MISO  37=MOSI
//   Chip Sel:  38=IMU_CS  39=WAS_CS  40=ACT_CS
//   INT/Safe:  41=IMU_INT
//   Safety:    4  (standalone, pull-up input)
// ---------------------------------------------------------------------------
#define SENS_SPI_SCK   35
#define SENS_SPI_MISO  36
#define SENS_SPI_MOSI  37

// Chip Selects (SPI Bus 2 devices) – active LOW
#define CS_IMU          38
#define CS_STEER_ANG   39
#define CS_ACT         40

// IMU interrupt (BNO085 INT pin)
#define IMU_INT        41

// ---------------------------------------------------------------------------
// Safety input (active LOW)
// TODO: Verify this matches your wiring. GPIO 4 is a common choice.
// ---------------------------------------------------------------------------
#define SAFETY_IN       4
