/**
 * @file modules.cpp
 * @brief AgOpenGPS module registry – hardware detection, hello replies, error reporting.
 *
 * This file bridges the low-level hardware detection (HAL) with the
 * AgOpenGPS protocol layer (PGN encoding, UDP sending).
 *
 * Module → Subsystem mapping:
 *   Steer Module requires: WAS (steer angle sensor), Actuator, Safety
 *   GPS Module requires:   GNSS Main, (optional) GNSS Heading, IMU
 *   Both require:          Ethernet (W5500)
 *
 * Detection strategy:
 *   - SPI sensors: attempt chip ID / register read at init
 *   - GNSS: flag set when first NMEA sentence received (late detection)
 *   - Ethernet: tracked by ETH.begin() result
 *   - Safety: GPIO read at init
 */

#include "modules.h"
#include "aog_udp_protocol.h"
#include "global_state.h"
#include "hal/hal.h"

#include <cstdio>
#include <cstring>

// ===================================================================
// Internal state
// ===================================================================

/// Module table – all modules this firmware implements
static AogModuleInfo s_modules[AOG_MOD_COUNT] = {
    { AOG_SRC_STEER,     AOG_PORT_STEER, "Steer", true, false },
    { AOG_SRC_GPS_REPLY, AOG_PORT_GPS,   "GPS",   true, false },
};

/// Hardware detection results (filled by modulesInit)
static ModuleHwStatus s_hw = {};

/// Module IP (for subnet replies) – matches hal_impl.cpp default
static uint8_t s_module_ip[4] = {192, 168, 1, 70};
static uint8_t s_module_subnet[3] = {255, 255, 255};

/// Track if startup errors have been sent
static bool s_startup_errors_sent = false;

/// Track if GPS module was late-detected (GNSS got first data after init)
static bool s_gps_late_detected = false;

// ===================================================================
// Init – detect all hardware
// ===================================================================
void modulesInit(void) {
    hal_log("MODULES: === Hardware Detection ===");

    // --- Detect individual subsystems ---

    // Ethernet
    s_hw.eth_detected = hal_net_detected();
    hal_log("MODULES: Ethernet (W5500)    : %s", s_hw.eth_detected ? "OK" : "FAIL");

    // IMU (BNO085)
    s_hw.imu_detected = hal_imu_detect();

    // Steer Angle Sensor (WAS)
    s_hw.was_detected = hal_steer_angle_detect();

    // Actuator
    s_hw.actuator_detected = hal_actuator_detect();

    // GNSS (late detection – check after a brief poll period)
    // At this point, the UART is initialized but may not have received data yet.
    // We poll briefly to give the GNSS a chance to send its first sentence.
    s_hw.gnss_main_detected = hal_gnss_main_detect();
    s_hw.gnss_head_detected = hal_gnss_heading_detect();
    hal_log("MODULES: GNSS Main            : %s", s_hw.gnss_main_detected ? "OK" : "PENDING (no data yet)");
    hal_log("MODULES: GNSS Heading         : %s", s_hw.gnss_head_detected ? "OK" : "PENDING (no data yet)");

    // Safety circuit
    s_hw.safety_ok = hal_safety_ok();
    hal_log("MODULES: Safety Circuit       : %s", s_hw.safety_ok ? "OK" : "KICK");

    // --- Derive module hw_detected from subsystems ---

    // Steer Module: needs Ethernet + WAS + Actuator + Safety
    // (IMU is part of GPS module data path)
    s_modules[AOG_MOD_STEER].hw_detected =
        s_hw.eth_detected &&
        s_hw.was_detected &&
        s_hw.actuator_detected &&
        s_hw.safety_ok;

    // GPS Module: needs Ethernet + GNSS Main
    // IMU and GNSS Heading are optional enhancements
    s_modules[AOG_MOD_GPS].hw_detected =
        s_hw.eth_detected &&
        s_hw.gnss_main_detected;

    // --- Log module summary ---
    hal_log("MODULES: === Module Summary ===");
    for (uint8_t i = 0; i < AOG_MOD_COUNT; i++) {
        hal_log("MODULES:   [%u] %s (Src=0x%02X, Port=%u) enabled=%s hw=%s",
                (unsigned)i,
                s_modules[i].name,
                s_modules[i].src_id,
                (unsigned)s_modules[i].port,
                s_modules[i].enabled ? "Y" : "N",
                s_modules[i].hw_detected ? "OK" : "FAIL");
    }
    hal_log("MODULES: === Detection Complete ===");
}

// ===================================================================
// Accessors
// ===================================================================
uint8_t modulesCount(void) {
    return AOG_MOD_COUNT;
}

const AogModuleInfo* modulesGet(AogModuleId id) {
    if (id >= AOG_MOD_COUNT) return nullptr;
    return &s_modules[id];
}

const ModuleHwStatus* modulesGetHwStatus(void) {
    return &s_hw;
}

bool modulesHwOk(AogModuleId id) {
    if (id >= AOG_MOD_COUNT) return false;
    return s_modules[id].hw_detected;
}

// ===================================================================
// Send hello replies for ALL enabled modules
// ===================================================================
void modulesSendHellos(void) {
    uint8_t buf[64];

    for (uint8_t i = 0; i < AOG_MOD_COUNT; i++) {
        if (!s_modules[i].enabled) continue;

        const AogModuleInfo& mod = s_modules[i];
        size_t len = 0;
        const char* label = nullptr;

        if (mod.src_id == AOG_SRC_STEER) {
            // Steer hello: PGN=0x7E, Len=5
            // Payload: steerAngle(2) + sensorCounts(2) + switchByte(1)
            StateLock lock;
            int16_t angle = static_cast<int16_t>(g_nav.steer_angle_deg);
            uint16_t counts = 0;  // TODO: from actual sensor
            uint8_t sw = g_nav.safety_ok ? 0x00 : 0x80;
            len = encodeAogHelloReplySteer(buf, sizeof(buf), angle, counts, sw);
            label = "SteerHello";

        } else if (mod.src_id == AOG_SRC_GPS_REPLY) {
            // GPS hello: PGN=0x78, Len=5
            // Payload: 5 reserved bytes (all zero)
            len = encodeAogHelloReplyGps(buf, sizeof(buf));
            label = "GpsHello";
        }

        if (len > 0) {
            hal_net_send(buf, len, mod.port);
            aogHexDump(label, buf, len);
            hal_log("MODULE: sent %s hello reply (%zu bytes, src=0x%02X, port=%u)",
                    mod.name, len, mod.src_id, (unsigned)mod.port);
        }
    }
}

// ===================================================================
// Send subnet replies for ALL enabled modules
// ===================================================================
void modulesSendSubnetReplies(void) {
    uint8_t buf[64];

    for (uint8_t i = 0; i < AOG_MOD_COUNT; i++) {
        if (!s_modules[i].enabled) continue;

        const AogModuleInfo& mod = s_modules[i];
        size_t len = encodeAogSubnetReply(buf, sizeof(buf),
                                           mod.src_id,
                                           s_module_ip, s_module_subnet);
        if (len > 0) {
            hal_net_send(buf, len, mod.port);
            hal_log("MODULE: sent %s subnet reply (src=0x%02X, port=%u)",
                    mod.name, mod.src_id, (unsigned)mod.port);
        }
    }
}

// ===================================================================
// Internal helper: send an error message via UDP if network is up,
// otherwise log to Serial only.
// ===================================================================
static void reportError(const char* subsystem, const char* message,
                         uint8_t src, uint8_t color) {
    if (hal_net_is_connected()) {
        uint8_t tx_buf[AOG_MAX_FRAME];
        size_t len = encodeAogHardwareMessage(tx_buf, sizeof(tx_buf),
                                               src,
                                               AOG_HWMSG_DURATION_PERSIST,
                                               color,
                                               message);
        if (len > 0) {
            hal_net_send(tx_buf, len, AOG_PORT_AGIO);
            hal_log("MODULES: UDP error sent – %s: %s", subsystem, message);
            return;
        }
    }
    // Fallback: Serial only
    hal_log("MODULES: [SERIAL] %s: %s  (network not available)", subsystem, message);
}

// ===================================================================
// Send startup error messages for failed hardware
// ===================================================================
void modulesSendStartupErrors(void) {
    if (s_startup_errors_sent) return;
    s_startup_errors_sent = true;

    hal_log("MODULES: === Startup Error Report ===");

    // --- Check each subsystem and report if failed ---
    // If network is up, errors go via UDP (PGN 0xDD) to AgIO.
    // If network is down, errors go to Serial only.

    // Ethernet
    if (!s_hw.eth_detected) {
        reportError("Ethernet", "ERR Ethernet: W5500 Not Detected",
                    AOG_SRC_STEER, AOG_HWMSG_COLOR_RED);
    }

    // GNSS Main
    if (!s_hw.gnss_main_detected) {
        reportError("GNSS Main", "ERR GNSS Main: No Data Received",
                    AOG_SRC_GPS, AOG_HWMSG_COLOR_RED);
    }

    // GNSS Heading (warning, not error – it's optional)
    if (!s_hw.gnss_head_detected) {
        reportError("GNSS Heading", "WARN GNSS Heading: No Data",
                    AOG_SRC_GPS, AOG_HWMSG_COLOR_YELLOW);
    }

    // IMU
    if (!s_hw.imu_detected) {
        reportError("IMU", "ERR IMU (BNO085): Not Detected",
                    AOG_SRC_STEER, AOG_HWMSG_COLOR_RED);
    }

    // Steer Angle Sensor
    if (!s_hw.was_detected) {
        reportError("SteerAngle", "ERR Steer Angle Sensor: Not Detected",
                    AOG_SRC_STEER, AOG_HWMSG_COLOR_RED);
    }

    // Actuator
    if (!s_hw.actuator_detected) {
        reportError("Actuator", "ERR Actuator: Not Detected",
                    AOG_SRC_STEER, AOG_HWMSG_COLOR_RED);
    }

    // Safety Circuit
    if (!s_hw.safety_ok) {
        reportError("Safety", "ERR Safety Circuit: KICK Engaged",
                    AOG_SRC_STEER, AOG_HWMSG_COLOR_RED);
    }

    // --- Module-level summaries ---
    for (uint8_t i = 0; i < AOG_MOD_COUNT; i++) {
        if (!s_modules[i].enabled) continue;
        if (s_modules[i].hw_detected) continue;

        char msg[64];
        std::snprintf(msg, sizeof(msg), "ERR Module %s: Not Available", s_modules[i].name);
        reportError(s_modules[i].name, msg,
                    s_modules[i].src_id, AOG_HWMSG_COLOR_RED);
    }

    hal_log("MODULES: === Startup Error Report Complete (%s) ===",
            hal_net_is_connected() ? "UDP" : "Serial only");
}

// ===================================================================
// Update dynamic hardware status (late detection)
// ===================================================================
void modulesUpdateStatus(void) {
    // GNSS Main: late detection – if first data arrives after init
    if (!s_hw.gnss_main_detected && hal_gnss_main_detect()) {
        s_hw.gnss_main_detected = true;
        s_gps_late_detected = true;
        hal_log("MODULES: GNSS Main LATE DETECTED – now reporting data");

        // If GPS module was previously failed, it's now OK
        if (!s_modules[AOG_MOD_GPS].hw_detected && s_hw.eth_detected) {
            s_modules[AOG_MOD_GPS].hw_detected = true;
            hal_log("MODULES: GPS module now ACTIVE");

            // Send green "OK" message to clear the error in AgIO
            reportError("GPS", "GPS Module: Now Active",
                        AOG_SRC_GPS, AOG_HWMSG_COLOR_GREEN);
        }
    }

    // GNSS Heading: late detection
    if (!s_hw.gnss_head_detected && hal_gnss_heading_detect()) {
        s_hw.gnss_head_detected = true;
        hal_log("MODULES: GNSS Heading LATE DETECTED");
    }

    // Safety circuit: dynamic monitoring
    bool current_safety = hal_safety_ok();
    if (current_safety != s_hw.safety_ok) {
        s_hw.safety_ok = current_safety;
        if (!current_safety) {
            hal_log("MODULES: Safety circuit KICK detected!");
        } else {
            hal_log("MODULES: Safety circuit restored");
        }
    }
}
