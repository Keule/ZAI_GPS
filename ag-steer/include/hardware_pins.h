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
 * SPI BUS ALLOCATION  (CRITICAL – wrong bus = crash!)
 * ========================================================================
 *   SPI3_HOST = W5500 Ethernet  (onboard, GPIO  9/10/11/12/13/14)
 *   SPI2_HOST = Shared bus      (GPIO  5/6/7/38/39/40/42/43)
 *              → SD card (OTA + logging)
 *              → ADS1118 ADC (steer angle)
 *              → BNO085 IMU
 *              → Actuator driver
 *
 * On ESP32-S3 in Arduino Core 2.x:
 *   FSPI = SPI2_HOST   HSPI = SPI3_HOST
 *
 * The shared bus MUST use FSPI (= SPI2_HOST), NOT HSPI (= SPI3_HOST)!
 * HSPI is the same physical bus as SPI3_HOST which the W5500 ETH driver
 * uses. Sharing the bus between ETH driver and SPIClass causes a hard
 * crash: "assert failed: spi_hal_setup_trans ... spi_ll_get_running_cmd"
 *
 * GPIO NOTE – ESP32-S3R8 (with Octal PSRAM):
 *   GPIOs 26-37 are INTERNALLY occupied by the Octal PSRAM interface.
 *   They are NOT available as general-purpose GPIOs and are not connected
 *   to the header pins on the T-ETH-Lite-S3 board.
 *   GPIOs 38-42 are OUTPUT-ONLY (no input capability).
 *   GPIOs 43-48 are full bidirectional GPIOs.
 * ========================================================================
 *
 * GPIO assignment – by header row:
 *
 *   Row 1 (onboard ETH):  9  10  11  12  13  14  = W5500 (fixed)
 *   Row 2 (GNSS UART):   15  16  17  18           = UART2, UART1
 *   Row 3 (SPI + CS):     5   6   7              = SPI2 bus (MISO/MOSI/SCK)
 *   Row 4 (CS + misc):   38  39  40  42           = CS pins (output-only OK)
 *   Misc:                 4  43  47               = Safety, IMU_INT, LogSwitch
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
// SPI Bus 2: Shared Sensor + SD Card Bus (FSPI = SPI2_HOST)
//
// All SPI devices (SD card, ADS1118, IMU, actuator) share this bus.
// Each device has its own CS pin.  The Arduino SD library and our sensor
// code both use beginTransaction()/endTransaction() for exclusive access.
//
// GPIOs 5/6/7 are bidirectional general-purpose GPIOs, verified free on
// the T-ETH-Lite-S3 (not connected to any onboard peripheral).
//
// During normal operation the sensor code owns the bus.
// For SD card access (OTA at boot, or logging), the logger temporarily
// releases the sensor SPI so the SD library can use the same bus.
// ---------------------------------------------------------------------------
#define SENS_SPI_SCK   7      // SPI clock
#define SENS_SPI_MISO  5      // SPI MISO (data from devices to ESP32)
#define SENS_SPI_MOSI  6      // SPI MOSI (data from ESP32 to devices)

// Chip Selects (active LOW) – GPIOs 38-42 are output-only, which is fine.
#define CS_IMU          38    // BNO085 IMU
#define CS_STEER_ANG   39    // ADS1118 ADC (steer angle potentiometer)
#define CS_ACT         40    // Actuator driver

// SD Card – same bus, different CS
#define SD_CS          42    // SD card slot

// IMU interrupt (BNO085 INT pin) – needs bidirectional GPIO for input!
#define IMU_INT        43    // GPIO 43 is bidirectional on ESP32-S3

// ---------------------------------------------------------------------------
// Safety input (active LOW)
// ---------------------------------------------------------------------------
#define SAFETY_IN       4

// Firmware file name on SD card (for OTA)
#define SD_FW_FILE_PRIMARY   "/firmware.bin"
#define SD_FW_FILE_ALT       "/update.bin"

// Version file on SD card (optional – contains e.g. "1.2.3")
#define SD_FW_VERSION_FILE   "/firmware.version"

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
// Isolated ADS1118 test pins (debug only)
//
// GPIOs 44, 45, 46, 48 are free bidirectional GPIOs.
// Enable with #define ADS1118_ISOLATED_TEST in hal_impl.cpp.
// Then connect the ADS1118 module directly to these pins:
//   ADS1118 DOUT  -> GPIO 44 (MISO)
//   ADS1118 DIN   -> GPIO 45 (MOSI)
//   ADS1118 SCLK  -> GPIO 46 (SCK)
//   ADS1118 CS    -> GPIO 48 (CS)
// This uses a separate SPIClass on FSPI to test the ADS1118 without
// any interference from the shared bus (SD card, IMU, actuator).
// ---------------------------------------------------------------------------
#define ADS_TEST_MISO  44
#define ADS_TEST_MOSI  45
#define ADS_TEST_SCK   46
#define ADS_TEST_CS    48
