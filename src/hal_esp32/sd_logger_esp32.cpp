/**
 * @file sd_logger_esp32.cpp
 * @brief ESP32-S3 implementation of the SD-Card Data Logger.
 *
 * Platform-specific code for the logger:
 *   - GPIO 47 switch reading (with debounce)
 *   - FreeRTOS logger task (lowest priority)
 *   - SD card init/deinit on shared SPI2_HOST bus
 *   - CSV file creation and writing
 *
 * SPI bus strategy:
 *   The SD card uses SPI2_HOST (FSPI) with pins SCK=7, MISO=5, MOSI=6.
 *   The sensor bus also uses FSPI but with DIFFERENT pins (SCK=16, MISO=15, MOSI=17).
 *   During SD card access, the sensor SPI is released via hal_sensor_spi_deinit(),
 *   then FSPI is re-initialised with SD pins. After SD access, the sensor SPI
 *   is restored via hal_sensor_spi_reinit().
 *
 *   To prevent the control loop from accessing the bus while SD is active,
 *   a s_spi_busy flag is used:
 *     - Logger sets flag before SD access, clears after
 *     - Control loop should check hal_spi_busy() and skip sensor reads
 *       when the flag is set (use last known values)
 *
 *   The sensor SPI is unavailable for ~50-200 ms per flush cycle.
 */

#include "hal/hal.h"
#include "hardware_pins.h"
#include "logic/sd_logger.h"
#include "logic/global_state.h"

#include "logic/log_config.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_SDL
#include "esp_log.h"
#include "logic/log_ext.h"

// ESP32 / Arduino includes
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <freertos/FreeRTOS.h>

// ===================================================================
// Configuration
// ===================================================================

/// Logger task interval (ms). The task wakes every N ms to check
/// the switch and flush the ring buffer to SD.
static const uint32_t LOGGER_INTERVAL_MS = 2000;

/// Switch debounce time (ms). After a state change, wait this long
/// before accepting the new state.
static const uint32_t SWITCH_DEBOUNCE_MS = 200;

/// Maximum CSV line length (one record formatted as CSV).
static const size_t CSV_LINE_MAX = 160;

// ===================================================================
// State
// ===================================================================

/// FreeRTOS task handle
static TaskHandle_t s_logger_task_handle = nullptr;

/// Current switch state (true = logging active)
static volatile bool s_logging_active = false;

/// SD card file handle (valid only while s_logging_active is true)
static fs::File s_log_file;

/// Current log file number (auto-incremented for each new session)
static uint32_t s_file_counter = 0;

/// SPI busy flag – set to true while SD card is accessing SPI2_HOST.
/// Sensor reads should be skipped when this is true.
static volatile bool s_spi_busy = false;

/// Statistics
static uint32_t s_last_flush_count = 0;
static uint32_t s_last_overflow = 0;
static uint32_t s_session_flush_total = 0;

// ===================================================================
// Platform hooks – called from sd_logger.cpp
// ===================================================================

/// Global declarations from sd_logger.cpp
extern "C" bool sdLoggerHasRecords(void);
extern "C" bool sdLoggerReadRecord(SdLogRecord* out);
extern "C" void sdLoggerIncrementFlushed(uint32_t count);
extern "C" uint32_t sdLoggerGetOverflowCount(void);

extern "C" bool sdLoggerReadSwitch(void) {
    // Active LOW: LOW = logging ON, HIGH = logging OFF
    return digitalRead(LOG_SWITCH_PIN) == LOW;
}

// ===================================================================
// SPI bus coordination
// ===================================================================

bool hal_spi_busy(void) {
    return s_spi_busy;
}

// ===================================================================
// CSV formatting
// ===================================================================

/**
 * Format a log record as a CSV line.
 *
 * CSV columns:
 *   timestamp_ms, heading_deg, steer_angle_deg, desired_angle_deg,
 *   yaw_rate_dps, roll_deg, safety_ok
 *
 * @param rec   The log record to format.
 * @param buf   Output buffer (at least CSV_LINE_MAX bytes).
 * @return      Number of characters written (excluding null terminator).
 */
static size_t formatCsvLine(const SdLogRecord& rec, char* buf) {
    int n = snprintf(buf, CSV_LINE_MAX,
        "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%u",
        (unsigned long)rec.timestamp_ms,
        (double)rec.heading_deg,
        (double)rec.steer_angle_deg,
        (double)rec.desired_angle_deg,
        (double)rec.yaw_rate_dps,
        (double)rec.roll_deg,
        (unsigned)rec.safety_ok);

    return (n > 0) ? static_cast<size_t>(n) : 0;
}

// ===================================================================
// SD card operations
// ===================================================================

/**
 * Open a new CSV log file on the SD card.
 *
 * Filename format: /log_001.csv, /log_002.csv, etc.
 * Finds the next available file number.
 *
 * @return true if the file was opened successfully.
 */
static bool openLogFile(void) {
    s_file_counter++;

    // Scan for next available file number
    char path[32];
    for (uint32_t i = s_file_counter; i < 10000; i++) {
        snprintf(path, sizeof(path), "/log_%03lu.csv", (unsigned long)i);
        if (!SD.exists(path)) {
            s_file_counter = i;
            break;
        }
    }

    snprintf(path, sizeof(path), "/log_%03lu.csv", (unsigned long)s_file_counter);
    s_log_file = SD.open(path, FILE_WRITE);

    if (!s_log_file) {
        LOGE("SDL", "ERROR – cannot create %s", path);
        return false;
    }

    // Write CSV header
    s_log_file.println("timestamp_ms,heading_deg,steer_angle_deg,desired_angle_deg,"
                       "yaw_rate_dps,roll_deg,safety_ok");
    s_log_file.flush();

    LOGI("SDL", "opened %s", path);
    return true;
}

/**
 * Flush the ring buffer to the SD card.
 *
 * @return Number of records written to SD.
 */
static uint32_t flushBufferToSD(void) {
    if (!s_log_file) return 0;

    char csv_buf[CSV_LINE_MAX];
    uint32_t count = 0;
    const uint32_t batch_limit = 512;

    while (sdLoggerHasRecords() && count < batch_limit) {
        SdLogRecord rec;
        if (!sdLoggerReadRecord(&rec)) break;

        size_t len = formatCsvLine(rec, csv_buf);
        if (len > 0) {
            s_log_file.write(reinterpret_cast<uint8_t*>(csv_buf), len);
            s_log_file.write('\n');
            count++;
        }
    }

    if (count > 0) {
        s_log_file.flush();
    }

    return count;
}

// ===================================================================
// SD card init / deinit on shared SPI bus
// ===================================================================

/**
 * Claim the SPI bus for SD card access.
 *
 * The SD card uses FSPI with pins SCK=7, MISO=5, MOSI=6 (CS=42).
 * The sensor bus uses FSPI with pins SCK=16, MISO=15, MOSI=17.
 * We must release the sensor SPIClass before the SD library can
 * claim the FSPI peripheral with its own pins.
 */
static void sdBusClaim(void) {
    s_spi_busy = true;
    hal_sensor_spi_deinit();   // release FSPI peripheral
    hal_delay_ms(2);
}

/**
 * Release the SPI bus back to sensor code.
 */
static void sdBusRelease(void) {
    hal_sensor_spi_reinit();   // reclaim FSPI peripheral for sensors
    s_spi_busy = false;
}

// ===================================================================
// Logger task
// ===================================================================

/**
 * Main function for the logger FreeRTOS task.
 *
 * Runs at LOWEST priority on Core 0.
 * Wakes every LOGGER_INTERVAL_MS to:
 *   1. Check the hardware switch
 *   2. If switch is ON: flush ring buffer to SD
 *   3. If switch is OFF: close log file, release SD
 *
 * For each flush cycle:
 *   1. Claim SPI bus (set s_spi_busy flag)
 *   2. Init SD card on FSPI with SD pins
 *   3. Open/append log file, write records
 *   4. Close file, release SD
 *   5. Release SPI bus (clear s_spi_busy flag)
 *
 * We open/close the file on each flush to minimise bus hold time.
 * The overhead is minimal (~100 ms total) and guarantees the sensor
 * SPI is available between flushes.
 */
static void loggerTaskFunc(void* param) {
    (void)param;
    LOGI("SDL", "task started on core %d (priority=lowest)", xPortGetCoreID());

    bool was_active = false;
    uint32_t last_switch_change_ms = 0;
    bool last_switch_raw = sdLoggerReadSwitch();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(LOGGER_INTERVAL_MS));

        // Read switch with simple debounce
        bool switch_raw = sdLoggerReadSwitch();
        uint32_t now = millis();
        if (switch_raw != last_switch_raw) {
            last_switch_change_ms = now;
            last_switch_raw = switch_raw;
        }
        bool switch_debounced = switch_raw;

        // State machine: switch ON -> start logging, switch OFF -> stop
        if (switch_debounced && !was_active) {
            // --- TRANSITION: OFF -> ON ---
            LOGI("SDL", "switch ON – starting logging session");

            sdBusClaim();

            // Init SD card on shared SPI2_HOST
            SPIClass sdSPI(FSPI);
            sdSPI.begin(SD_SPI_SCK, SD_SPI_MISO, SD_SPI_MOSI, SD_CS);

            if (SD.begin(SD_CS, sdSPI, 4000000, "/sd", 5)) {
                LOGI("SDL", "SD card mounted OK");
                openLogFile();
                s_logging_active = true;
                s_session_flush_total = 0;
                s_last_flush_count = 0;
                s_last_overflow = 0;
            } else {
                LOGE("SDL", "ERROR – SD card init failed");
                s_logging_active = false;
            }

            SD.end();
            sdSPI.end();

            sdBusRelease();
            was_active = true;

        } else if (!switch_debounced && was_active) {
            // --- TRANSITION: ON -> OFF ---
            LOGI("SDL", "switch OFF – stopping logging session");
            LOGI("SDL", "session stats: %lu records flushed, %lu buffer overflows",
                    (unsigned long)s_session_flush_total,
                    (unsigned long)sdLoggerGetOverflowCount() - s_last_overflow);

            s_logging_active = false;
            was_active = false;

        } else if (switch_debounced && was_active) {
            // --- ACTIVE: flush ring buffer to SD ---
            if (!sdLoggerHasRecords()) continue;

            sdBusClaim();

            // Re-init SD (we close it after each flush)
            SPIClass sdSPI(FSPI);
            sdSPI.begin(SD_SPI_SCK, SD_SPI_MISO, SD_SPI_MOSI, SD_CS);

            if (SD.begin(SD_CS, sdSPI, 4000000, "/sd", 5)) {
                // Re-open the log file in append mode
                char path[32];
                snprintf(path, sizeof(path), "/log_%03lu.csv", (unsigned long)s_file_counter);
                s_log_file = SD.open(path, FILE_APPEND);

                if (s_log_file) {
                    uint32_t flushed = flushBufferToSD();
                    sdLoggerIncrementFlushed(flushed);
                    s_session_flush_total += flushed;

                    if (flushed > 0) {
                        uint32_t buf_remaining = sdLoggerGetBufferCount();
                        uint32_t overflows = sdLoggerGetOverflowCount();
                        LOGI("SDL", "flushed %lu records (buf=%lu, overflow=%lu)",
                                (unsigned long)flushed,
                                (unsigned long)buf_remaining,
                                (unsigned long)overflows);
                    }

                    s_log_file.close();
                }
            } else {
                LOGE("SDL", "ERROR – SD card re-init failed during flush");
            }

            SD.end();
            sdSPI.end();

            sdBusRelease();
        }
        // else: switch OFF and was_active=false -> nothing to do (idle)
    }
}

// ===================================================================
// Public API – ESP32 implementation
// ===================================================================

void sdLoggerInit(void) {
    // Configure logging switch GPIO
    pinMode(LOG_SWITCH_PIN, INPUT_PULLUP);
    LOGI("SDL", "switch on GPIO %d (active LOW)", LOG_SWITCH_PIN);

    s_logging_active = false;

    // Create the logger task on Core 0 with LOWEST priority.
    xTaskCreatePinnedToCore(
        loggerTaskFunc,
        "logger",
        4096,
        nullptr,
        1,              // LOWEST priority
        &s_logger_task_handle,
        0               // Core 0
    );

    LOGI("SDL", "initialised (flush interval = %lu ms, log rate = 10 Hz)",
            (unsigned long)LOGGER_INTERVAL_MS);
}
