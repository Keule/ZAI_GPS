/**
 * @file hw_status.cpp
 * @brief Hardware status monitoring implementation.
 *
 * Monitors all subsystems and sends PGN 0xDD hardware messages to AgIO.
 * Messages are rate-limited to avoid flooding the network.
 *
 * Pure C++ logic – uses HAL only for timing and network send.
 */

#include "hw_status.h"
#include "aog_udp_protocol.h"
#include "hal/hal.h"

#include <cstdarg>
#include <cstdio>

// ===================================================================
// Configuration
// ===================================================================

/// How often to re-send a persistent error message (milliseconds)
static const uint32_t RESEND_INTERVAL_MS = 10000;  // 10 seconds

/// Minimum time between any two hardware messages (milliseconds)
static const uint32_t MIN_MSG_INTERVAL_MS = 2000;  // 2 seconds

/// Delay before reporting an error after first detection (debounce)
static const uint32_t ERROR_DEBOUNCE_MS = 3000;  // 3 seconds

// ===================================================================
// State
// ===================================================================

/// Per-subsystem status
static HwSubsysStatus s_subsys[HW_COUNT];

/// Timestamp of last hardware message sent (rate limiting)
static uint32_t s_last_msg_sent_ms = 0;

// ===================================================================
// Human-readable subsystem names (for HW messages)
// ===================================================================
static const char* const s_subsys_names[HW_COUNT] = {
    "Ethernet",     // HW_ETH
    "GNSS Main",    // HW_GNSS_MAIN
    "GNSS Heading", // HW_GNSS_HEAD
    "IMU",          // HW_IMU
    "Steer Angle",  // HW_WAS
    "Actuator",     // HW_ACTUATOR
    "Safety"        // HW_SAFETY
};

// ===================================================================
// Init
// ===================================================================
void hwStatusInit(void) {
    for (uint8_t i = 0; i < HW_COUNT; i++) {
        s_subsys[i].error      = false;
        s_subsys[i].severity   = HW_SEV_OK;
        s_subsys[i].first_seen = 0;
        s_subsys[i].last_sent  = 0;
    }
    s_last_msg_sent_ms = 0;
    hal_log("HWSTATUS: monitoring initialised (%u subsystems)", (unsigned)HW_COUNT);
}

// ===================================================================
// Set / Clear / Query flags
// ===================================================================
void hwStatusSetFlag(HwSubsystem id, HwSeverity severity) {
    if (id >= HW_COUNT) return;
    if (!s_subsys[id].error) {
        s_subsys[id].error = true;
        s_subsys[id].severity = severity;
        s_subsys[id].first_seen = hal_millis();
        hal_log("HWSTATUS: ERROR flagged: %s (sev=%u)",
                s_subsys_names[id], (unsigned)severity);
    }
}

void hwStatusClearFlag(HwSubsystem id) {
    if (id >= HW_COUNT) return;
    if (s_subsys[id].error) {
        hal_log("HWSTATUS: CLEARED: %s", s_subsys_names[id]);
    }
    s_subsys[id].error = false;
    s_subsys[id].severity = HW_SEV_OK;
}

bool hwStatusHasError(HwSubsystem id) {
    if (id >= HW_COUNT) return false;
    return s_subsys[id].error;
}

uint8_t hwStatusErrorCount(void) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < HW_COUNT; i++) {
        if (s_subsys[i].error) count++;
    }
    return count;
}

// ===================================================================
// Internal: send a formatted hardware message
// ===================================================================
static void sendHwMessage(uint8_t src, uint8_t color, uint8_t duration,
                           const char* text) {
    uint32_t now = hal_millis();

    // Rate limit: minimum interval between messages
    if (now - s_last_msg_sent_ms < MIN_MSG_INTERVAL_MS) return;

    uint8_t tx_buf[AOG_MAX_FRAME];
    size_t len = encodeAogHardwareMessage(tx_buf, sizeof(tx_buf),
                                           src, duration, color, text);
    if (len > 0) {
        hal_net_send(tx_buf, len, AOG_PORT_AGIO);
        s_last_msg_sent_ms = now;
        hal_log("HWSTATUS: sent msg color=%u dur=%u \"%s\"",
                (unsigned)color, (unsigned)duration, text);
    }
}

// ===================================================================
// Send a one-off hardware message (varargs)
// ===================================================================
void hwStatusSendMessage(uint8_t src, HwSeverity severity, uint8_t duration,
                          const char* fmt, ...) {
    char text[AOG_HWMSG_MAX_TEXT + 1];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    sendHwMessage(src, severity, duration, text);
}

// ===================================================================
// Periodic update – check all subsystems, send messages
// ===================================================================
uint8_t hwStatusUpdate(bool connected,
                        bool gnss_fix,
                        bool gnss_head_fix,
                        bool safety_ok,
                        bool steer_angle_valid,
                        bool imu_valid) {
    uint32_t now = hal_millis();

    // --- Update subsystem flags based on actual status ---

    // Ethernet
    if (!connected) {
        hwStatusSetFlag(HW_ETH, HW_SEV_ERROR);
    } else {
        hwStatusClearFlag(HW_ETH);
    }

    // GNSS Main
    if (!gnss_fix) {
        hwStatusSetFlag(HW_GNSS_MAIN, HW_SEV_WARNING);
    } else {
        hwStatusClearFlag(HW_GNSS_MAIN);
    }

    // GNSS Heading
    if (!gnss_head_fix) {
        hwStatusSetFlag(HW_GNSS_HEAD, HW_SEV_WARNING);
    } else {
        hwStatusClearFlag(HW_GNSS_HEAD);
    }

    // IMU
    if (!imu_valid) {
        hwStatusSetFlag(HW_IMU, HW_SEV_WARNING);
    } else {
        hwStatusClearFlag(HW_IMU);
    }

    // Steer Angle Sensor
    if (!steer_angle_valid) {
        hwStatusSetFlag(HW_WAS, HW_SEV_WARNING);
    } else {
        hwStatusClearFlag(HW_WAS);
    }

    // Safety circuit
    if (!safety_ok) {
        hwStatusSetFlag(HW_SAFETY, HW_SEV_ERROR);
    } else {
        hwStatusClearFlag(HW_SAFETY);
    }

    // --- Send hardware messages for active errors ---

    for (uint8_t i = 0; i < HW_COUNT; i++) {
        if (!s_subsys[i].error) continue;

        // Debounce: wait before first message
        if (now - s_subsys[i].first_seen < ERROR_DEBOUNCE_MS) continue;

        // Resend interval for persistent errors
        if (s_subsys[i].last_sent > 0 &&
            now - s_subsys[i].last_sent < RESEND_INTERVAL_MS) {
            continue;
        }

        // Build message text
        char text[AOG_HWMSG_MAX_TEXT + 1];
        const char* name = s_subsys_names[i];
        const char* sev_text = (s_subsys[i].severity == HW_SEV_ERROR) ? "ERR" : "WARN";

        std::snprintf(text, sizeof(text), "%s %s: %s", sev_text, name,
                      (s_subsys[i].severity == HW_SEV_ERROR) ? "Failure" : "Not Available");

        // Send from steer module source
        sendHwMessage(AOG_SRC_STEER, s_subsys[i].severity,
                      AOG_HWMSG_DURATION_PERSIST, text);

        if (s_subsys[i].last_sent == 0) {
            // First time sending for this error
            s_subsys[i].last_sent = now;
        }
    }

    // --- If all errors cleared and we had errors before, send OK message ---
    // (Only once, when transitioning from error state to OK)
    static bool s_had_errors = false;
    uint8_t err_count = hwStatusErrorCount();

    if (err_count > 0) {
        s_had_errors = true;
    } else if (s_had_errors) {
        // All errors cleared
        s_had_errors = false;
        // Send a single "all OK" message (green)
        sendHwMessage(AOG_SRC_STEER, HW_SEV_OK, 3, "All systems OK");
    }

    return err_count;
}
