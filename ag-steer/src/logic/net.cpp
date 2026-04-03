/**
 * @file net.cpp
 * @brief Network / UDP communication implementation.
 *
 * Sends:
 *   - GPS Main Out (PGN 0xD6) on port 5124  @ ~10 Hz
 *   - Steer Status Out (PGN 0xFD) on port 5126 @ ~10 Hz
 *   - From Autosteer 2 (PGN 0xFA) on port 5126 @ ~10 Hz
 *
 * Receives:
 *   - Hello From AgIO (PGN 200)
 *   - Scan Request (PGN 202)
 *   - Subnet Change (PGN 201)
 *   - Steer Data In (PGN 254)
 *
 * All frames use the AOG Ethernet protocol format.
 * Reference: https://github.com/AgOpenGPS-Official/Boards/blob/main/PGN.md
 */

#include "net.h"
#include "modules.h"
#include "control.h"
#include "global_state.h"
#include "hal/hal.h"

#include <cstring>

// ===================================================================
// Send interval tracking
// ===================================================================
static uint32_t s_last_send_ms = 0;
static const uint32_t SEND_INTERVAL_MS = 100;  // 10 Hz

// ===================================================================
// Initialise network
// ===================================================================
void netInit(void) {
    hal_net_init();
    hal_log("NET: initialised (W5500 Ethernet)");
    hal_log("NET: dest IP = %u.%u.%u.%u",
            g_net_cfg.dest_ip[0], g_net_cfg.dest_ip[1],
            g_net_cfg.dest_ip[2], g_net_cfg.dest_ip[3]);
}

// ===================================================================
// Process a single validated frame
// ===================================================================
void netProcessFrame(uint8_t src, uint8_t pgn,
                     const uint8_t* payload, size_t payload_len) {
    switch (pgn) {
        case PGN_HELLO_FROM_AGIO: {
            AogHelloFromAgio msg;
            if (tryDecodeAogHelloFromAgio(payload, payload_len, &msg)) {
                hal_log("NET: Hello from AgIO (module=0x%02X, ver=%u) -> sending ALL module hellos",
                        (unsigned)msg.moduleId, (unsigned)msg.agioVersion);
                // Send hello reply for ALL enabled modules (Steer + GPS)
                modulesSendHellos();
            }
            break;
        }

        case PGN_SCAN_REQUEST: {
            if (tryDecodeAogScanRequest(payload, payload_len)) {
                hal_log("NET: Scan request -> sending ALL module subnet replies");
                // Send subnet reply for ALL enabled modules
                modulesSendSubnetReplies();
            }
            break;
        }

        case PGN_SUBNET_CHANGE: {
            AogSubnetChange msg;
            if (tryDecodeAogSubnetChange(payload, payload_len, &msg)) {
                // Update destination IP: set first 3 octets from subnet change
                g_net_cfg.dest_ip[0] = msg.ip_one;
                g_net_cfg.dest_ip[1] = msg.ip_two;
                g_net_cfg.dest_ip[2] = msg.ip_three;
                g_net_cfg.dest_ip[3] = 255;  // broadcast

                hal_log("NET: subnet changed, dest=%u.%u.%u.%u",
                        g_net_cfg.dest_ip[0], g_net_cfg.dest_ip[1],
                        g_net_cfg.dest_ip[2], g_net_cfg.dest_ip[3]);
            }
            break;
        }

        case PGN_STEER_DATA_IN: {
            AogSteerDataIn msg;
            if (tryDecodeAogSteerDataIn(payload, payload_len, &msg)) {
                // Update desired steer angle from AgIO
                desiredSteerAngleDeg = msg.steerAngle / 100.0f;

                // TODO: extract speed for feedforward, section control, etc.
                {
                    StateLock lock;
                    g_nav.sog_mps = msg.speed / 100.0f;  // cm/s -> m/s
                }
            }
            break;
        }

        case PGN_HARDWARE_MESSAGE: {
            // Incoming hardware message from AgIO (display or command)
            uint8_t dur = 0, color = 0;
            char msg_text[128];
            if (tryDecodeAogHardwareMessage(payload, payload_len,
                                             &dur, &color,
                                             msg_text, sizeof(msg_text))) {
                hal_log("NET: HW message from AgIO: [%u] (col=%u) \"%s\"",
                        (unsigned)dur, (unsigned)color, msg_text);
                // TODO: display on connected LCD, or process commands
            }
            break;
        }

        case PGN_STEER_SETTINGS_IN:
            hal_log("NET: SteerSettings received (len=%zu) – TODO: implement", payload_len);
            break;

        case PGN_STEER_CONFIG_IN:
            hal_log("NET: SteerConfig received (len=%zu) – TODO: implement", payload_len);
            break;

        default:
            hal_log("NET: unhandled PGN 0x%02X from Src 0x%02X (len=%zu)",
                    pgn, src, payload_len);
            break;
    }
}

// ===================================================================
// Poll for received UDP frames
// ===================================================================
void netPollReceive(void) {
    uint8_t rx_buf[AOG_MAX_FRAME];

    while (true) {
        uint16_t src_port = 0;
        int rx_len = hal_net_receive(rx_buf, sizeof(rx_buf), &src_port);

        if (rx_len <= 0) break;

        // Validate frame
        uint8_t frame_src = 0;
        uint8_t frame_pgn = 0;
        const uint8_t* payload = nullptr;
        size_t payload_len = 0;

        if (aogValidateFrame(rx_buf, static_cast<size_t>(rx_len),
                              &frame_src, &frame_pgn,
                              &payload, &payload_len)) {
            aogHexDump("NET: rx frame", rx_buf, static_cast<size_t>(rx_len));
            netProcessFrame(frame_src, frame_pgn, payload, payload_len);
        } else {
            hal_log("NET: invalid frame received (%d bytes from port %u)",
                    rx_len, src_port);
        }
    }
}

// ===================================================================
// Send periodic AOG frames
// ===================================================================
void netSendAogFrames(void) {
    // Skip if network not available
    if (!hal_net_is_connected()) return;

    uint32_t now = hal_millis();
    if (now - s_last_send_ms < SEND_INTERVAL_MS) return;
    s_last_send_ms = now;

    uint8_t tx_buf[AOG_MAX_FRAME];
    size_t tx_len = 0;

    // ----------------------------------------------------------
    // 1. GPS Main Out (PGN 0xD6) -> AgIO port 5124
    // ----------------------------------------------------------
    {
        StateLock lock;
        AogGpsMainOut gps{};
        gps.longitude    = static_cast<int32_t>(g_nav.lon_deg * 1e7);
        gps.latitude     = static_cast<int32_t>(g_nav.lat_deg * 1e7);
        gps.heading      = static_cast<int16_t>(g_nav.heading_deg * 16.0f);
        gps.dualHeading  = static_cast<int16_t>(g_nav.heading_deg * 16.0f);
        gps.speed        = static_cast<uint16_t>(g_nav.sog_mps * 1000.0f);  // m/s -> mm/s
        gps.roll         = static_cast<int16_t>(g_nav.roll_deg * 16.0f);
        gps.altitude     = static_cast<int32_t>(g_nav.alt_m * 1000.0f);     // m -> mm
        gps.satCount     = 12;   // TODO: from GGA
        gps.fixQuality   = g_nav.fix_quality;
        gps.hdop         = 100;  // TODO: from GGA
        gps.age          = 0;
        gps.imuHeading   = static_cast<int16_t>(g_nav.heading_deg * 16.0f);
        gps.imuRoll      = static_cast<int16_t>(g_nav.roll_deg * 16.0f);
        gps.imuPitch     = 0;
        gps.imuYawRate   = static_cast<int16_t>(g_nav.yaw_rate_dps * 16.0f);
        gps.reserved1    = 0;
        gps.relay        = 0;
        gps.section1_8   = 0;
        gps.section9_16  = 0;
        gps.year         = 25;
        gps.month        = 1;
        gps.day          = 1;
        gps.hour         = 0;
        gps.minute       = 0;
        gps.second       = 0;
        gps.ms_hi        = 0;
        gps.ms_lo        = 0;

        tx_len = encodeAogGpsMainOut(tx_buf, sizeof(tx_buf), gps);
        if (tx_len > 0) {
            hal_net_send(tx_buf, tx_len, AOG_PORT_GPS);
        }
    }

    // ----------------------------------------------------------
    // 2. Steer Status Out (PGN 0xFD) -> AgIO port 5126
    //    Note: heading and roll are degrees * 16 in this PGN
    // ----------------------------------------------------------
    {
        StateLock lock;
        int16_t angle_x100 = static_cast<int16_t>(g_nav.steer_angle_deg * 100.0f);
        int16_t heading_x16 = static_cast<int16_t>(g_nav.heading_deg * 16.0f);
        int16_t roll_x16    = static_cast<int16_t>(g_nav.roll_deg * 16.0f);
        uint8_t switch_st   = g_nav.safety_ok ? 0x00 : 0x80;  // bit7 = safety
        uint8_t pwm_disp    = 0;  // TODO: from PID output

        tx_len = encodeAogSteerStatusOut(tx_buf, sizeof(tx_buf),
                                          angle_x100, heading_x16, roll_x16,
                                          switch_st, pwm_disp);
        if (tx_len > 0) {
            hal_net_send(tx_buf, tx_len, AOG_PORT_STEER);
        }
    }

    // ----------------------------------------------------------
    // 3. From Autosteer 2 (PGN 0xFA) -> AgIO port 5126
    //    Sensor value byte for steer angle sensor
    // ----------------------------------------------------------
    {
        StateLock lock;
        // Send raw sensor value (just low byte of steer angle for now)
        uint8_t sensor_val = static_cast<uint8_t>(
            static_cast<int16_t>(g_nav.steer_angle_deg) & 0xFF);
        tx_len = encodeAogFromAutosteer2(tx_buf, sizeof(tx_buf), sensor_val);
        if (tx_len > 0) {
            hal_net_send(tx_buf, tx_len, AOG_PORT_STEER);
        }
    }
}
