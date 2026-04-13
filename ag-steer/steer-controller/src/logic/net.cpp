/**
 * @file net.cpp
 * @brief Network / UDP communication implementation.
 *
 * Sends:
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
// AgIO status byte bitfield (from PGN 254 steer data)
// ===================================================================
constexpr uint8_t STATUS_BIT_WORK_SWITCH   = 0x01;  // bit 0
constexpr uint8_t STATUS_BIT_STEER_SWITCH  = 0x02;  // bit 1
constexpr uint8_t STATUS_BIT_STEER_ON      = 0x04;  // bit 2

// ===================================================================
// Send interval tracking
// ===================================================================
static uint32_t s_last_send_ms = 0;
static const uint32_t SEND_INTERVAL_MS = 100;  // 10 Hz

// Rate-limit log messages to avoid serial spam from broadcast echo
static uint32_t s_last_invalid_log_ms = 0;
static uint32_t s_last_unhandled_log_ms = 0;

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
    // Only process frames from AgIO (0x7F).
    // Ignore our own frames (0x7E) echoed back via broadcast,
    // and any other module-to-module traffic.
    if (src != AOG_SRC_AGIO) return;

    switch (pgn) {
        case PGN_HELLO_FROM_AGIO: {
            AogHelloFromAgio msg;
            if (tryDecodeAogHelloFromAgio(payload, payload_len, &msg)) {
                hal_log("NET: Hello from AgIO (module=0x%02X, ver=%u) -> sending ALL module hellos",
                        (unsigned)msg.moduleId, (unsigned)msg.agioVersion);
                // Send hello reply for ALL enabled modules (Steer)
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

                // Also update the HAL's destination IP (used by hal_net_send)
                hal_net_set_dest_ip(msg.ip_one, msg.ip_two, msg.ip_three, 255);

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

                // Decode status byte: work switch, steer switch, steer on
                {
                    StateLock lock;
                    g_nav.work_switch      = (msg.status & STATUS_BIT_WORK_SWITCH) != 0;
                    g_nav.steer_switch     = (msg.status & STATUS_BIT_STEER_SWITCH) != 0;
                    g_nav.last_status_byte = msg.status;

                    // Store GPS speed for safety check [km/h]
                    g_nav.gps_speed_kmh = msg.speed / 10.0f;

                    // Reset watchdog – AgIO is alive
                    g_nav.watchdog_timer_ms = hal_millis();
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

        case PGN_STEER_SETTINGS_IN: {
            AogSteerSettingsIn settings;
            if (tryDecodeAogSteerSettingsIn(payload, payload_len, &settings)) {
                // Apply settings to PID controller
                controlUpdateSettings(settings.kp, settings.highPWM, settings.lowPWM,
                                     settings.minPWM, settings.countsPerDegree,
                                     settings.wasOffset, settings.ackerman);
                hal_log("NET: SteerSettings applied (Kp=%u hiPWM=%u loPWM=%u minPWM=%u cnt=%u off=%d ack=%u)",
                        (unsigned)settings.kp, (unsigned)settings.highPWM,
                        (unsigned)settings.lowPWM, (unsigned)settings.minPWM,
                        (unsigned)settings.countsPerDegree, (int)settings.wasOffset,
                        (unsigned)settings.ackerman);
            }
            break;
        }

        case PGN_STEER_CONFIG_IN: {
            AogSteerConfigIn config;
            if (tryDecodeAogSteerConfigIn(payload, payload_len, &config)) {
                hal_log("NET: SteerConfig received (set0=0x%02X pulse=%u speed=%u ackFix=%u)",
                        (unsigned)config.set0, (unsigned)config.maxPulse,
                        (unsigned)config.minSpeed, (unsigned)config.ackermanFix);

                // Store config in global state for future use
                {
                    StateLock lock;
                    g_nav.config_set0      = config.set0;
                    g_nav.config_max_pulse = config.maxPulse;
                    g_nav.config_min_speed = config.minSpeed;
                    g_nav.config_received  = true;
                }

                // TODO: apply hardware config bits (invert WAS, relay polarity,
                // motor direction, Cytron driver mode, etc.)
                // Reference does a hard reset after config – we log only for now.
            }
            break;
        }

        default: {
            // Rate-limit unhandled PGN logs (max once per 10s)
            uint32_t now = hal_millis();
            if (now - s_last_unhandled_log_ms >= 10000) {
                s_last_unhandled_log_ms = now;
                hal_log("NET: unhandled PGN 0x%02X from Src 0x%02X (len=%zu)",
                        pgn, src, payload_len);
            }
            break;
        }
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

        // Quick filter: skip non-AOG frames early (NMEA starts with '$' = 0x24)
        if (rx_buf[0] != AOG_ID_1) continue;

        // Validate frame
        uint8_t frame_src = 0;
        uint8_t frame_pgn = 0;
        const uint8_t* payload = nullptr;
        size_t payload_len = 0;

        if (aogValidateFrame(rx_buf, static_cast<size_t>(rx_len),
                              &frame_src, &frame_pgn,
                              &payload, &payload_len)) {
            netProcessFrame(frame_src, frame_pgn, payload, payload_len);
        } else {
            // Rate-limit invalid frame logs (max once per 10s)
            uint32_t now = hal_millis();
            if (now - s_last_invalid_log_ms >= 10000) {
                s_last_invalid_log_ms = now;
                hal_log("NET: invalid frame (%d bytes from port %u, first=0x%02X)",
                        rx_len, src_port, rx_buf[0]);
            }
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
    // 1. Steer Status Out (PGN 0xFD) -> AgIO port 5126
    // ----------------------------------------------------------
    {
        StateLock lock;
        int16_t angle_x100 = static_cast<int16_t>(g_nav.steer_angle_deg * 100.0f);
        int16_t heading_x10 = static_cast<int16_t>(g_nav.heading_deg * 10.0f);
        int16_t roll_x10    = static_cast<int16_t>(g_nav.roll_deg * 10.0f);

        // Switch byte: bit 7 = safety (0=OK, 1=KICK), bit 0 = steer switch relay
        // Also include work switch state
        uint8_t switch_st = 0;
        if (!g_nav.safety_ok)   switch_st |= 0x80;
        if (g_nav.work_switch) switch_st |= 0x01;
        if (g_nav.steer_switch) switch_st |= 0x02;

        // PWM display: map PID output (0-65535) to 0-255
        uint8_t pwm_disp = static_cast<uint8_t>(
            (g_nav.pid_output * 255.0f) / 65535.0f);

        tx_len = encodeAogSteerStatusOut(tx_buf, sizeof(tx_buf),
                                          angle_x100, heading_x10, roll_x10,
                                          switch_st, pwm_disp);
        if (tx_len > 0) {
            hal_net_send(tx_buf, tx_len, AOG_PORT_STEER);
        }
    }

    // ----------------------------------------------------------
    // 2. From Autosteer 2 (PGN 0xFA) -> AgIO port 5126
    //    Sensor value byte for steer angle sensor (raw ADC low byte)
    // ----------------------------------------------------------
    {
        StateLock lock;
        // Send raw sensor value (low byte of raw ADC)
        uint8_t sensor_val = static_cast<uint8_t>(g_nav.steer_angle_raw & 0xFF);
        tx_len = encodeAogFromAutosteer2(tx_buf, sizeof(tx_buf), sensor_val);
        if (tx_len > 0) {
            hal_net_send(tx_buf, tx_len, AOG_PORT_STEER);
        }
    }
}
