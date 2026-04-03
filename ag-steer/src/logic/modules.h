/**
 * @file modules.h
 * @brief AgOpenGPS module registry and management.
 *
 * Defines all AgOpenGPS network modules implemented by this firmware,
 * tracks their hardware detection status, and provides functions to:
 *   - Send hello replies for all enabled modules
 *   - Send subnet replies for all enabled modules
 *   - Report hardware errors via PGN 0xDD for failed modules
 *
 * AgOpenGPS module concept:
 *   Each "module" is a logical network entity with a unique Source ID
 *   and UDP port. AgIO discovers modules via Hello (PGN 200) and Scan
 *   (PGN 202) requests, and modules respond with hello/subnet replies.
 *
 * Modules implemented by this firmware:
 *   1. Steer Module (Src=0x7E, Port=5126) – steering control
 *   2. GPS Module   (Src=0x78, Port=5124) – GNSS position data
 *
 * Reference: https://github.com/AgOpenGPS-Official/Boards/blob/main/PGN.md
 */

#pragma once

#include <cstddef>
#include <cstdint>

// ===================================================================
// AgOpenGPS Module IDs
// ===================================================================
enum AogModuleId : uint8_t {
    AOG_MOD_STEER = 0,   ///< Steering module (Src=0x7E, Port=5126)
    AOG_MOD_GPS,         ///< GPS module       (Src=0x78, Port=5124)
    AOG_MOD_COUNT        ///< Number of defined modules
};

// ===================================================================
// Per-module hardware subsystem detection results
// ===================================================================
struct ModuleHwStatus {
    bool eth_detected;       ///< W5500 Ethernet chip detected
    bool gnss_main_detected; ///< GNSS main receiver has sent data
    bool gnss_head_detected; ///< GNSS heading receiver has sent data
    bool imu_detected;       ///< IMU (BNO085) chip detected via SPI
    bool was_detected;       ///< Wheel Angle Sensor (steer angle) detected
    bool actuator_detected;  ///< Actuator detected via SPI
    bool safety_ok;          ///< Safety circuit reads OK
};

// ===================================================================
// Module descriptor
// ===================================================================
struct AogModuleInfo {
    uint8_t  src_id;       ///< AgOpenGPS Source ID (0x7E, 0x78, etc.)
    uint16_t port;         ///< UDP port for this module's data
    const char* name;      ///< Human-readable module name
    bool     enabled;      ///< Software enabled (module is compiled in)
    bool     hw_detected;  ///< Hardware detected and operational
};

// ===================================================================
// Initialise module system.
// Runs hardware detection for all subsystems, logs results,
// and sets hw_detected for each module based on its requirements.
// Call AFTER hal_esp32_init_all() and BEFORE netInit().
// ===================================================================
void modulesInit(void);

// ===================================================================
// Get number of defined modules.
// ===================================================================
uint8_t modulesCount(void);

// ===================================================================
// Get module descriptor by index.
// Returns nullptr if index out of range.
// ===================================================================
const AogModuleInfo* modulesGet(AogModuleId id);

// ===================================================================
// Get full hardware detection results.
// ===================================================================
const ModuleHwStatus* modulesGetHwStatus(void);

// ===================================================================
// Send hello reply for ALL enabled modules.
// Each module sends its specific hello PGN on its port:
//   - Steer: PGN=0x7E, Src=0x7E, Len=5 (angle+counts+switch)
//   - GPS:   PGN=0x78, Src=0x78, Len=5 (reserved)
// Call when AgIO Hello (PGN 200) is received.
// ===================================================================
void modulesSendHellos(void);

// ===================================================================
// Send subnet reply for ALL enabled modules.
// Each module sends: PGN=0xCB, Len=7 (IP + Subnet)
// Call when AgIO Scan (PGN 202) is received.
// ===================================================================
void modulesSendSubnetReplies(void);

// ===================================================================
// Send hardware error messages (PGN 0xDD) for modules and subsystems
// where hardware was NOT detected during init.
// Call once after modulesInit() when network is up.
// Only sends messages for failures – no message if all OK.
// ===================================================================
void modulesSendStartupErrors(void);

// ===================================================================
// Check if a specific module has hardware detected.
// ===================================================================
bool modulesHwOk(AogModuleId id);

// ===================================================================
// Update dynamic hardware status (call periodically from commTask).
// Handles late-detecting hardware (e.g., GNSS getting first fix
// after satellites are acquired).
// ===================================================================
void modulesUpdateStatus(void);
