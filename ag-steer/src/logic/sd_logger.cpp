/**
 * @file sd_logger.cpp
 * @brief SD-Card Data Logger – ring buffer and platform-independent logic.
 *
 * The ring buffer is a lock-free single-producer / single-consumer queue:
 *   - Producer: control loop (Core 1) calls sdLoggerRecord()
 *   - Consumer: logger task (Core 0) calls sdLoggerDrainBuffer()
 *
 * The ring buffer uses power-of-2 size so wrap-around can be done
 * with a simple mask instead of modulo.
 *
 * This file is platform-independent – no Arduino/ESP32 headers.
 * Platform-specific code lives in sd_logger_esp32.cpp.
 */

#include "logic/sd_logger.h"
#include <cstring>

// ===================================================================
// Configuration
// ===================================================================

/// Ring buffer size (must be power of 2).
/// 512 records × 48 bytes = 24 KB.
/// At 10 Hz log rate: 51 seconds of buffer before overflow.
#define LOG_RING_SIZE       512
#define LOG_RING_MASK       (LOG_RING_SIZE - 1)

/// Log rate divider. The control loop runs at 200 Hz.
/// A divider of 20 gives a 10 Hz log rate (one record every 100 ms).
#define LOG_RATE_DIVIDER    20

// ===================================================================
// Ring buffer – single-producer / single-consumer
// ===================================================================

/// The ring buffer itself (static allocation).
static SdLogRecord s_ring_buf[LOG_RING_SIZE];

/// Write index – only modified by the producer (control loop).
/// After writing to s_ring_buf[s_write_idx], the producer does:
///   s_write_idx = (s_write_idx + 1) & LOG_RING_MASK;
static volatile uint32_t s_write_idx = 0;

/// Read index – only modified by the consumer (logger task).
/// After reading from s_ring_buf[s_read_idx], the consumer does:
///   s_read_idx = (s_read_idx + 1) & LOG_RING_MASK;
static volatile uint32_t s_read_idx = 0;

/// Overflow counter – incremented when the producer overwrites
/// data that the consumer hasn't read yet.
static volatile uint32_t s_overflow_count = 0;

/// Total records flushed to SD (incremented by consumer).
static volatile uint32_t s_records_flushed = 0;

/// Call counter for subsampling – incremented each time
/// sdLoggerRecord() is called. Only records when the counter
/// is a multiple of LOG_RATE_DIVIDER.
static volatile uint32_t s_call_counter = 0;

// ===================================================================
// Platform hooks – implemented in sd_logger_esp32.cpp
// ===================================================================

/// Read the hardware logging switch.
/// Returns true if logging should be active (pin LOW).
extern "C" bool sdLoggerReadSwitch(void);

/// Drain the ring buffer: read all pending records and write to SD.
/// Called by the logger task. Returns the number of records flushed.
extern "C" uint32_t sdLoggerDrainBuffer(void);

/// Close the current log file and release the SD card.
/// Called when the switch is turned OFF.
extern "C" void sdLoggerCloseFile(void);

// ===================================================================
// Public API – platform-independent
// ===================================================================

void sdLoggerRecord(void) {
    // Only record every N-th call (subsample from 200 Hz to 10 Hz)
    uint32_t count = s_call_counter;
    s_call_counter = count + 1;

    if ((count % LOG_RATE_DIVIDER) != 0) {
        return;  // skip – not our turn
    }

    // Only buffer if logging is active (fast GPIO check)
    if (!sdLoggerReadSwitch()) {
        return;
    }

    // Capture current state into a temporary record.
    // We read the state directly from g_nav to avoid
    // holding the mutex in the control loop.
    extern NavigationState g_nav;
    extern volatile float desiredSteerAngleDeg;

    SdLogRecord rec;
    rec.timestamp_ms     = g_nav.timestamp_ms;
    rec.lat_deg          = g_nav.lat_deg;
    rec.lon_deg          = g_nav.lon_deg;
    rec.fix_quality      = g_nav.fix_quality;
    rec.speed_mps        = g_nav.sog_mps;
    rec.heading_deg      = g_nav.heading_deg;
    rec.steer_angle_deg  = g_nav.steer_angle_deg;
    rec.desired_angle_deg = desiredSteerAngleDeg;
    rec.yaw_rate_dps     = g_nav.yaw_rate_dps;
    rec.safety_ok        = g_nav.safety_ok ? 1 : 0;

    // Write to ring buffer (no mutex needed – SPSC pattern)
    uint32_t widx = s_write_idx;
    uint32_t next_widx = (widx + 1) & LOG_RING_MASK;

    // Check for overflow (consumer hasn't caught up)
    if (next_widx == s_read_idx) {
        // Buffer full – advance read pointer (drop oldest)
        s_read_idx = (s_read_idx + 1) & LOG_RING_MASK;
        s_overflow_count++;
    }

    s_ring_buf[widx] = rec;
    s_write_idx = next_widx;
}

bool sdLoggerIsActive(void) {
    return sdLoggerReadSwitch();
}

uint32_t sdLoggerGetRecordsFlushed(void) {
    return s_records_flushed;
}

uint32_t sdLoggerGetBufferCount(void) {
    uint32_t w = s_write_idx;
    uint32_t r = s_read_idx;
    if (w >= r) return w - r;
    return LOG_RING_SIZE - r + w;
}

// ===================================================================
// Buffer access – called from the logger task (consumer side)
// ===================================================================

/// Check if there are records waiting in the ring buffer.
bool sdLoggerHasRecords(void) {
    return s_write_idx != s_read_idx;
}

/// Read one record from the ring buffer (consumer side).
/// Returns true if a record was read, false if buffer is empty.
bool sdLoggerReadRecord(SdLogRecord* out) {
    uint32_t r = s_read_idx;
    if (r == s_write_idx) {
        return false;  // buffer empty
    }

    *out = s_ring_buf[r];
    s_read_idx = (r + 1) & LOG_RING_MASK;
    return true;
}

/// Increment the flushed counter (called by the consumer after writing to SD).
void sdLoggerIncrementFlushed(uint32_t count) {
    s_records_flushed += count;
}

/// Get the overflow counter (for diagnostics).
uint32_t sdLoggerGetOverflowCount(void) {
    return s_overflow_count;
}
