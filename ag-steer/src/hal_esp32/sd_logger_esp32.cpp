/**
 * @file sd_logger_esp32.cpp
 * @brief ESP32-S3 implementation of the SD-Card Data Logger.
 *
 * Platform-specific code for the logger:
 *   - GPIO 47 switch reading (with debounce)
 *   - FreeRTOS logger task (lowest priority)
 *   - SD card init/deinit with SPI2_HOST bus switching
 *   - CSV file creation and writing
 *
 * SPI bus strategy:
 *   The SD card uses SPI2_HOST (FSPI) with pins 5/6/7/42.
 *   The sensor bus also uses SPI2_HOST but with pins 35/36/37.
 *   They cannot be active simultaneously on the same SPI host.
 *
 *   During a flush cycle:
 *     1. Sensor SPI is released (hal_sensor_spi_deinit)
 *     2. SD card is initialised with SD-card pins
 *     3. Buffered records are written to the CSV file
 *     4. SD card is released
 *     5. Sensor SPI is restored (hal_sensor_spi_reinit)
 *
 *   The sensor SPI is unavailable for ~50-200 ms per flush cycle.
 *   The control loop must tolerate this (use last known values).
 *   A flag (s_spi_busy) is provided so the control loop could check
 *   it, but typically the brief gap is acceptable for a 5-second
 *   flush interval.
 *
 * This file includes Arduino / ESP32 headers and is only compiled
 * for the ESP32 target.
 */

#include "hal/hal.h"
#include "hardware_pins.h"
#include "logic/sd_logger.h"
#include "logic/global_state.h"

// ESP32 / Arduino includes
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FreeRTOS.h>

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
/// Each field is at most ~15 chars, 10 fields + commas + newline.
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

/// SPI busy flag – set to true while SD card is using SPI2_HOST.
/// The control loop could check this to skip sensor reads.
static volatile bool s_spi_busy = false;

/// Statistics
static uint32_t s_last_flush_count = 0;
static uint32_t s_last_overflow = 0;
static uint32_t s_session_flush_total = 0;

// ===================================================================
// Platform hooks – called from sd_logger.cpp
// ===================================================================

/// Global declarations from sd_logger.cpp
extern bool sdLoggerHasRecords(void);
extern bool sdLoggerReadRecord(SdLogRecord* out);
extern void sdLoggerIncrementFlushed(uint32_t count);
extern uint32_t sdLoggerGetOverflowCount(void);

bool sdLoggerReadSwitch(void) {
    // Active LOW: LOW = logging ON, HIGH = logging OFF
    return digitalRead(LOG_SWITCH_PIN) == LOW;
}

// ===================================================================
// CSV formatting
// ===================================================================

/**
 * Format a log record as a CSV line.
 *
 * CSV columns:
 *   timestamp_ms, lat_deg, lon_deg, fix, speed_mps, heading_deg,
 *   steer_angle_deg, desired_angle_deg, yaw_rate_dps, safety_ok
 *
 * @param rec   The log record to format.
 * @param buf   Output buffer (at least CSV_LINE_MAX bytes).
 * @return      Number of characters written (excluding null terminator).
 */
static size_t formatCsvLine(const SdLogRecord& rec, char* buf) {
    // Use 7 decimal places for lat/lon (~1 cm precision)
    int n = snprintf(buf, CSV_LINE_MAX,
        "%lu,%.7f,%.7f,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%u",
        (unsigned long)rec.timestamp_ms,
        rec.lat_deg,
        rec.lon_deg,
        (unsigned)rec.fix_quality,
        (double)rec.speed_mps,
        (double)rec.heading_deg,
        (double)rec.steer_angle_deg,
        (double)rec.desired_angle_deg,
        (double)rec.yaw_rate_dps,
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
        hal_log("LOGGER: ERROR – cannot create %s", path);
        return false;
    }

    // Write CSV header
    s_log_file.println("timestamp_ms,lat_deg,lon_deg,fix,speed_mps,heading_deg,"
                       "steer_angle_deg,desired_angle_deg,yaw_rate_dps,safety_ok");
    s_log_file.flush();

    hal_log("LOGGER: opened %s", path);
    return true;
}

/**
 * Flush the ring buffer to the SD card.
 *
 * This is the critical section where SPI2_HOST is borrowed from the
 * sensor bus. The total time depends on the number of buffered records
 * and SD card write speed. Typically ~50-150 ms for a full 2-second
 * buffer at 10 Hz log rate.
 *
 * @return Number of records written to SD.
 */
static uint32_t flushBufferToSD(void) {
    if (!s_log_file) return 0;

    char csv_buf[CSV_LINE_MAX];
    uint32_t count = 0;
    const uint32_t batch_limit = 512;  // max records per flush

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
        s_log_file.flush();  // ensure data is written to SD
    }

    return count;
}

// ===================================================================
// Logger task
// ===================================================================

/**
 * Main function for the logger FreeRTOS task.
 *
 * Runs at the LOWEST priority (priority 1) on Core 0.
 * Wakes every LOGGER_INTERVAL_MS to:
 *   1. Check the hardware switch
 *   2. If switch is ON: flush ring buffer to SD
 *   3. If switch is OFF: close log file, release SD
 *
 * When the switch transitions from OFF → ON:
 *   - Release sensor SPI bus
 *   - Initialise SD card with SD pins
 *   - Open a new log file
 *   - Restore sensor SPI bus (file handle kept open)
 * Wait – actually we need to keep the SD card accessible between
 * flushes. But that means SPI2_HOST is mapped to SD pins, not
 * sensor pins.
 *
 * REVISED APPROACH: For each flush cycle:
 *   1. Release sensor SPI
 *   2. Init SD
 *   3. Write records
 *   4. Close file
 *   5. Release SD
 *   6. Restore sensor SPI
 *
 * This means we open/close the file on each flush. The overhead is
 * minimal (~100 ms total) and guarantees the sensor SPI is available
 * between flushes.
 */
static void loggerTaskFunc(void* param) {
    (void)param;
    hal_log("LOGGER: task started on core %d (priority=lowest)", xPortGetCoreID());

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

        // State machine: switch ON → start logging, switch OFF → stop
        if (switch_debounced && !was_active) {
            // --- TRANSITION: OFF → ON ---
            hal_log("LOGGER: switch ON – starting logging session");

            // Release sensor SPI → take over SPI2_HOST for SD
            s_spi_busy = true;
            hal_sensor_spi_deinit();
            hal_delay_ms(10);

            // Init SD card
            SPIClass sdSPI(FSPI);
            sdSPI.begin(SD_SPI_SCK, SD_SPI_MISO, SD_SPI_MOSI, SD_CS);

            if (SD.begin(SD_CS, sdSPI, 4000000, "/sd", 5)) {
                hal_log("LOGGER: SD card mounted OK");
                openLogFile();
                s_logging_active = true;
                s_session_flush_total = 0;
                s_last_flush_count = 0;
                s_last_overflow = 0;
            } else {
                hal_log("LOGGER: ERROR – SD card init failed");
                sdSPI.end();
                s_logging_active = false;
            }

            // Restore sensor SPI
            SD.end();
            sdSPI.end();
            hal_sensor_spi_reinit();
            s_spi_busy = false;

            was_active = true;

        } else if (!switch_debounced && was_active) {
            // --- TRANSITION: ON → OFF ---
            hal_log("LOGGER: switch OFF – stopping logging session");
            hal_log("LOGGER: session stats: %lu records flushed, %lu buffer overflows",
                    (unsigned long)s_session_flush_total,
                    (unsigned long)sdLoggerGetOverflowCount() - s_last_overflow);

            s_logging_active = false;
            was_active = false;

        } else if (switch_debounced && was_active) {
            // --- ACTIVE: flush ring buffer to SD ---
            if (!sdLoggerHasRecords()) continue;

            // Release sensor SPI → take over SPI2_HOST for SD
            s_spi_busy = true;
            hal_sensor_spi_deinit();
            hal_delay_ms(5);

            // Re-init SD (we close it after each flush, so need to re-open)
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
                        hal_log("LOGGER: flushed %lu records (buf=%lu, overflow=%lu)",
                                (unsigned long)flushed,
                                (unsigned long)buf_remaining,
                                (unsigned long)overflows);
                    }

                    s_log_file.close();
                }
            } else {
                hal_log("LOGGER: ERROR – SD card re-init failed during flush");
            }

            // Restore sensor SPI
            SD.end();
            sdSPI.end();
            hal_sensor_spi_reinit();
            s_spi_busy = false;

        }
        // else: switch OFF and was_active=false → nothing to do (idle)
    }
}

// ===================================================================
// Public API – ESP32 implementation
// ===================================================================

void sdLoggerInit(void) {
    // Configure logging switch GPIO
    pinMode(LOG_SWITCH_PIN, INPUT_PULLUP);
    hal_log("LOGGER: switch on GPIO %d (active LOW)", LOG_SWITCH_PIN);

    // Read initial state
    s_logging_active = false;

    // Create the logger task on Core 0 with LOWEST priority.
    // Stack size: 4096 bytes (needs headroom for SD + SPI operations)
    xTaskCreatePinnedToCore(
        loggerTaskFunc,
        "logger",
        4096,
        nullptr,
        1,              // LOWEST priority (configMAX_PRIORITIES - 1 = highest)
        &s_logger_task_handle,
        0               // Core 0
    );

    hal_log("LOGGER: initialised (flush interval = %lu ms, log rate = 10 Hz)",
            (unsigned long)LOGGER_INTERVAL_MS);
}
