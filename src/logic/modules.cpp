/**
 * @file modules.cpp
 * @brief AgOpenGPS module registry – hardware detection, hello replies, error reporting.
 *
 * This file bridges the low-level hardware detection (HAL) with the
 * AgOpenGPS protocol layer (PGN encoding, UDP sending).
 *
 * Module → Subsystem mapping:
 *   Steer Module   requires: Ethernet + WAS + IMU + Actuator + Safety
 *   GPS Module     requires: Ethernet + WAS + IMU
 *   Machine Module requires: Ethernet + Actuator
 *
 * Detection strategy:
 *   - SPI sensors: attempt chip ID / register read at init
 *   - Ethernet: tracked by ETH.begin() result
 *   - Safety: GPIO read at init
 */

#include "modules.h"
#include "features.h"
#include "pgn_codec.h"
#include "pgn_types.h"
#include "global_state.h"
#include "hw_status.h"
#include "hal/hal.h"

#include "log_config.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_MOD
#include "esp_log.h"
#include "log_ext.h"

#include <cstdio>
#include <cstring>

// ===================================================================
// Internal state
// ===================================================================

/// Module table – all modules this firmware implements
static AogModuleInfo s_modules[AOG_MOD_COUNT] = {
    { aog_src::STEER,   aog_pgn::HELLO_REPLY_STEER, aog_port::STEER,   "Steer",   false, false },
    { aog_src::GPS,     aog_pgn::HELLO_REPLY_GPS,   aog_port::GPS,     "GPS",     false, false },
    { aog_src::MACHINE, aog_src::MACHINE,           aog_port::MACHINE, "Machine", false, false },
};

/// Hardware detection results (filled by modulesInit)
static ModuleHwStatus s_hw = {};

/// Module IP (for subnet replies) – matches hal_impl.cpp default
static uint8_t s_module_ip[4] = {192, 168, 1, 70};
static uint8_t s_module_subnet[3] = {255, 255, 255};

/// Track if startup errors still need to be sent to AgIO.
static bool s_startup_errors_pending = true;

static bool isModuleActive(const AogModuleInfo& mod) {
    return mod.enabled && mod.hw_detected;
}

// ===================================================================
// Init – detect all hardware
// ===================================================================
void modulesInit(void) {
    hal_log("MODULES: === Hardware Detection ===");

    // --- Detect individual subsystems ---

    // Ethernet
    s_hw.eth_detected = feat::comm() ? hal_net_detected() : true;
    hal_log("MODULES: Ethernet (W5500)    : %s", s_hw.eth_detected ? "OK" : "FAIL");

    // Steer Angle Sensor (ADS1118) – detect BEFORE IMU!
    // The ADS1118 holds DOUT LOW while converting, which would
    // cause the IMU detect to read 0x00 (false negative).
    // Detecting ADS1118 first ensures DOUT is HIGH after detection.
    s_hw.was_detected = feat::sensor() ? hal_steer_angle_detect() : true;

    // IMU (BNO085)
    if (feat::imu()) {
        HalImuDetectStats imu_ds = {};
        s_hw.imu_detected = hal_imu_detect_boot_qualified(&imu_ds);
        hal_log("MODULES: IMU boot detect      : %s (ok=%u/%u ff=%u zero=%u last=0x%02X)",
                s_hw.imu_detected ? "OK" : "FAIL",
                (unsigned)imu_ds.ok_count,
                (unsigned)imu_ds.samples,
                (unsigned)imu_ds.ff_count,
                (unsigned)imu_ds.zero_count,
                (unsigned)imu_ds.last_response);
    } else {
        s_hw.imu_detected = true;
    }

    // Actuator
    s_hw.actuator_detected = feat::actor() ? hal_actuator_detect() : true;

    // Safety circuit
    s_hw.safety_ok = feat::control() ? hal_safety_ok() : true;
    hal_log("MODULES: Safety Circuit       : %s", s_hw.safety_ok ? "OK" : "KICK");

    // --- Derive module enablement + hw_detected from feature profiles ---
    s_modules[AOG_MOD_STEER].enabled   = feat::control();
    s_modules[AOG_MOD_GPS].enabled     = feat::sensor();
    s_modules[AOG_MOD_MACHINE].enabled = feat::actor();

    // Steer Module: needs Ethernet + WAS + IMU + Actuator + Safety
    s_modules[AOG_MOD_STEER].hw_detected =
        s_hw.eth_detected && s_hw.was_detected && s_hw.imu_detected &&
        s_hw.actuator_detected && s_hw.safety_ok;

    // GPS Module: needs Ethernet + WAS + IMU
    s_modules[AOG_MOD_GPS].hw_detected =
        s_hw.eth_detected && s_hw.was_detected && s_hw.imu_detected;

    // Machine Module: needs Ethernet + actuator path
    s_modules[AOG_MOD_MACHINE].hw_detected =
        s_hw.eth_detected && s_hw.actuator_detected;

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
        if (!isModuleActive(s_modules[i])) continue;

        const AogModuleInfo& mod = s_modules[i];
        size_t len = 0;
        const char* label = nullptr;

        if (mod.hello_pgn == aog_pgn::HELLO_REPLY_STEER) {
            // Steer hello: PGN=0x7E, Len=5
            // Payload: steerAngle×100(2) + sensorCounts(2) + switchByte(1)
            StateLock lock;
            int16_t angle = static_cast<int16_t>(g_nav.steer_angle_deg * 100.0f);
            uint16_t counts = static_cast<uint16_t>(g_nav.steer_angle_raw);
            uint8_t sw = 0;
            if (!g_nav.safety_ok)   sw |= 0x80;  // bit 7 = safety
            if (g_nav.work_switch) sw |= 0x01;  // bit 0 = work switch
            if (g_nav.steer_switch) sw |= 0x02; // bit 1 = steer switch
            len = pgnEncodeHelloReplySteer(buf, sizeof(buf), angle, counts, sw);
            label = "SteerHello";
        } else if (mod.hello_pgn == aog_pgn::HELLO_REPLY_GPS) {
            len = pgnEncodeHelloReplyGps(buf, sizeof(buf));
            label = "GpsHello";
        } else {
            // Generic hello reply: 5-byte zero payload, PGN = module hello PGN.
            // Used for modules without dedicated hello payload codec.
            uint8_t payload[5] = {0, 0, 0, 0, 0};
            len = pgnBuildFrame(buf, sizeof(buf), mod.src_id, mod.hello_pgn,
                                payload, sizeof(payload));
            label = "GenericHello";
        }

        if (len > 0) {
            hal_net_send(buf, len, mod.port);
            pgnHexDump(label, buf, len);
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
        if (!isModuleActive(s_modules[i])) continue;

        const AogModuleInfo& mod = s_modules[i];
        size_t len = pgnEncodeSubnetReply(buf, sizeof(buf),
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
// Send startup error messages for failed hardware
// ===================================================================
void modulesSendStartupErrors(void) {
    if (!s_startup_errors_pending) return;
    const bool net_connected = hal_net_is_connected();
    if (!net_connected) {
        hal_log("MODULES: startup errors pending (network down, defer UDP report)");
    }

    hal_log("MODULES: === Startup Error Report ===");

    // --- Module-level subsystem checks (active profile only) ---
    for (uint8_t i = 0; i < AOG_MOD_COUNT; i++) {
        if (!s_modules[i].enabled) continue;
        const AogModuleInfo& mod = s_modules[i];

        if (!s_hw.eth_detected) {
            hwStatusSendClassifiedMessage(mod.src_id, HW_ERR_CLASS_STARTUP, HW_ERR_PRIO_P1,
                                          aog_hwmsg::DURATION_PERSIST,
                                          "ERR Ethernet: W5500 Not Detected");
        }

        if (i == AOG_MOD_STEER || i == AOG_MOD_GPS) {
            if (!s_hw.was_detected) {
                hwStatusSendClassifiedMessage(mod.src_id, HW_ERR_CLASS_STARTUP, HW_ERR_PRIO_P1,
                                              aog_hwmsg::DURATION_PERSIST,
                                              "ERR Steer Angle Sensor: Not Detected");
            }
            if (!s_hw.imu_detected) {
                hwStatusSendClassifiedMessage(mod.src_id, HW_ERR_CLASS_STARTUP, HW_ERR_PRIO_P1,
                                              aog_hwmsg::DURATION_PERSIST,
                                              "ERR IMU (BNO085): Not Detected");
            }
        }

        if (i == AOG_MOD_STEER || i == AOG_MOD_MACHINE) {
            if (!s_hw.actuator_detected) {
                hwStatusSendClassifiedMessage(mod.src_id, HW_ERR_CLASS_STARTUP, HW_ERR_PRIO_P1,
                                              aog_hwmsg::DURATION_PERSIST,
                                              "ERR Actuator: Not Detected");
            }
        }

        if (i == AOG_MOD_STEER && !s_hw.safety_ok) {
            hwStatusSendClassifiedMessage(mod.src_id, HW_ERR_CLASS_STARTUP, HW_ERR_PRIO_P1,
                                          aog_hwmsg::DURATION_PERSIST,
                                          "ERR Safety Circuit: KICK Engaged");
        }

        char msg[64];
        std::snprintf(msg, sizeof(msg), "ERR Module %s: Not Available", mod.name);
        if (!mod.hw_detected) {
            hwStatusSendClassifiedMessage(mod.src_id, HW_ERR_CLASS_STARTUP, HW_ERR_PRIO_P1,
                                          aog_hwmsg::DURATION_PERSIST, "%s", msg);
        }
    }

    hal_log("MODULES: === Startup Error Report Complete (%s) ===",
            hal_net_is_connected() ? "UDP" : "Serial only");

    // Mark complete only when UDP path was available so AgIO can see boot faults.
    if (net_connected) {
        s_startup_errors_pending = false;
    }
}

// ===================================================================
// Update dynamic hardware status (late detection)
// ===================================================================
void modulesUpdateStatus(void) {
    if (!feat::control()) return;

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


ModState moduleGetState(const char* module_name) {
    if (!module_name) return MOD_UNAVAILABLE;
    for (uint8_t i = 0; i < AOG_MOD_COUNT; i++) {
        const AogModuleInfo& mod = s_modules[i];
        if (std::strcmp(mod.name, module_name) != 0) continue;
        if (!mod.enabled || !mod.hw_detected) return MOD_UNAVAILABLE;
        return MOD_ON;
    }
    if (std::strcmp(module_name, "NTRIP") == 0) {
#if FEAT_ENABLED(FEAT_NTRIP)
        return MOD_ON;
#else
        return MOD_UNAVAILABLE;
#endif
    }
    return MOD_UNAVAILABLE;
}

bool moduleActivate(const char* module_name) {
    return moduleGetState(module_name) != MOD_UNAVAILABLE;
}

bool moduleDeactivate(const char* module_name) {
    (void)module_name;
    return true;
}
