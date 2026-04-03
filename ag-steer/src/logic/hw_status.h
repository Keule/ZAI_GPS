/**
 * @file hw_status.h
 * @brief Hardware status monitoring and AgOpenGPS hardware message reporting.
 *
 * Monitors all subsystems (Ethernet, GNSS, IMU, Steer Angle, Actuator, Safety)
 * and sends PGN 0xDD hardware messages to AgIO when errors are detected.
 *
 * Usage:
 *   hwStatusInit()         – call once during setup
 *   hwStatusSetFlag(id)    – set a subsystem error flag
 *   hwStatusClearFlag(id)  – clear a subsystem error flag
 *   hwStatusUpdate()       – call periodically from commTask to check & send messages
 */

#pragma once

#include <cstddef>
#include <cstdint>

// ===================================================================
// Hardware Subsystem IDs – one bit per monitored subsystem
// ===================================================================
enum HwSubsystem : uint8_t {
    HW_ETH       = 0,   ///< W5500 Ethernet not detected / link down
    HW_GNSS_MAIN = 1,   ///< GNSS main receiver – no fix / no data
    HW_GNSS_HEAD = 2,   ///< GNSS heading receiver – no fix / no data
    HW_IMU       = 3,   ///< BNO085 IMU communication failure
    HW_WAS       = 4,   ///< Wheel Angle Sensor (steer angle) failure
    HW_ACTUATOR  = 5,   ///< Actuator communication failure
    HW_SAFETY    = 6,   ///< Safety circuit triggered (KICK)
    HW_COUNT     = 7    ///< Number of monitored subsystems
};

// ===================================================================
// Hardware Status Message Severity (maps to AOG color codes)
// ===================================================================
enum HwSeverity : uint8_t {
    HW_SEV_OK      = 0,  ///< Green – subsystem working
    HW_SEV_ERROR   = 1,  ///< Red – critical failure
    HW_SEV_WARNING = 2,  ///< Yellow – degraded operation
    HW_SEV_INFO    = 3,  ///< Blue – informational
};

// ===================================================================
// Per-subsystem status entry
// ===================================================================
struct HwSubsysStatus {
    bool     error;        ///< true = error detected
    uint8_t  severity;     ///< HwSeverity color code
    uint32_t first_seen;   ///< millis() when error was first detected
    uint32_t last_sent;    ///< millis() when last HW message was sent
};

// ===================================================================
// Initialise hardware status tracking.
// Call once during setup (after all HAL init).
// ===================================================================
void hwStatusInit(void);

// ===================================================================
// Set an error flag for a subsystem.
// ===================================================================
void hwStatusSetFlag(HwSubsystem id, HwSeverity severity = HW_SEV_ERROR);

// ===================================================================
// Clear an error flag for a subsystem.
// ===================================================================
void hwStatusClearFlag(HwSubsystem id);

// ===================================================================
// Check if a subsystem has an error.
// ===================================================================
bool hwStatusHasError(HwSubsystem id);

// ===================================================================
// Get the number of active errors.
// ===================================================================
uint8_t hwStatusErrorCount(void);

// ===================================================================
// Periodic update: check subsystems, send HW messages for active errors.
/// Should be called from commTask at ~1 Hz (lower rate than data frames).
/// @param connected  true if Ethernet/network is up
/// @param gnss_fix   true if GNSS main has a valid fix
/// @param gnss_head_fix  true if GNSS heading has a valid fix
/// @param safety_ok  true if safety circuit is OK
/// @param steer_angle_valid  true if steer angle sensor reading is plausible
/// @param imu_valid  true if IMU data is being received
/// @return number of active errors
/// ===================================================================
uint8_t hwStatusUpdate(bool connected,
                        bool gnss_fix,
                        bool gnss_head_fix,
                        bool safety_ok,
                        bool steer_angle_valid,
                        bool imu_valid);

// ===================================================================
// Send a single hardware message immediately (for one-off notifications).
/// @param src      module source ID (0x7E for steer, 0x7C for GPS)
/// @param severity error severity / color
/// @param duration display duration in seconds (0 = persistent)
/// @param fmt      printf-style format string
/// @param ...      format arguments
// ===================================================================
void hwStatusSendMessage(uint8_t src, HwSeverity severity, uint8_t duration,
                          const char* fmt, ...);
