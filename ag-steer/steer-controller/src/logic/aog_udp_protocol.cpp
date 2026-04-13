/**
 * @file aog_udp_protocol.cpp
 * @brief AgOpenGPS / AgIO UDP protocol – encoder, decoder, checksum.
 *
 * Pure C++ – no Arduino / ESP32 headers.
 * Reference: https://github.com/AgOpenGPS-Official/Boards/blob/main/PGN.md
 */

#include "aog_udp_protocol.h"
#include "global_state.h"
#include "hal/hal.h"

#include <cstdio>

// ===================================================================
// Network config instance
// ===================================================================
AogNetworkConfig g_net_cfg;

// ===================================================================
// Checksum (NOT a CRC – it is a simple additive 8-bit checksum).
//
// Algorithm: sum all bytes from index 2 (Src) to index (frame_len-2),
//   then take the low 8 bits.  The preamble (bytes 0,1 = 0x80,0x81)
//   and the checksum byte itself (last byte) are excluded.
//
// This is identical to the AgOpenGPS reference implementation
// (Autosteer_UDP_v5.ino, UDPComm.Designer.cs).
// ===================================================================
uint8_t aogChecksum(const uint8_t* frame, size_t frame_len) {
    if (frame_len < AOG_HEADER_SIZE + AOG_CRC_SIZE) return 0;

    // Minimum valid frame: 5 header + 1 CRC = 6 bytes.
    // Sum bytes[2 .. frame_len-2]  (Src + PGN + Len + payload)
    size_t last_included = frame_len - 1;  // index of the CRC byte itself
    uint16_t sum = 0;
    for (size_t i = 2; i < last_included; i++) {
        sum += frame[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

// ===================================================================
// Build a complete AOG frame: 0x80 0x81 | Src | PGN | Len | payload | CRC
// ===================================================================
size_t aogBuildFrame(uint8_t* buf, size_t buf_size,
                     uint8_t src, uint8_t pgn,
                     const void* payload, size_t payload_len) {
    size_t total = AOG_HEADER_SIZE + payload_len + AOG_CRC_SIZE;
    if (buf_size < total) {
        hal_log("AOG: frame buffer too small: need %zu, have %zu", total, buf_size);
        return 0;
    }

    // Preamble
    buf[0] = AOG_ID_1;
    buf[1] = AOG_ID_2;

    // Header
    buf[2] = src;
    buf[3] = pgn;
    buf[4] = static_cast<uint8_t>(payload_len);

    // Payload
    if (payload_len > 0 && payload != nullptr) {
        std::memcpy(buf + AOG_HEADER_SIZE, payload, payload_len);
    }

    // CRC over bytes 2..(total-2)
    uint16_t sum = 0;
    for (size_t i = 2; i < total - 1; i++) {
        sum += buf[i];
    }
    buf[total - 1] = static_cast<uint8_t>(sum & 0xFF);

    return total;
}

// ===================================================================
// Validate a received frame: check preamble, bounds, CRC.
// ===================================================================
bool aogValidateFrame(const uint8_t* frame, size_t frame_len,
                       uint8_t* out_src, uint8_t* out_pgn,
                       const uint8_t** out_payload, size_t* out_payload_len) {
    // Minimum: preamble(2) + src(1) + pgn(1) + len(1) + crc(1) = 6
    if (frame_len < 6) return false;

    // Check preamble
    if (frame[0] != AOG_ID_1 || frame[1] != AOG_ID_2) return false;

    // Extract header fields
    uint8_t src = frame[2];
    uint8_t pgn = frame[3];
    uint8_t len = frame[4];

    // Bounds check: header(5) + payload(len) + crc(1)
    if (frame_len < (size_t)(AOG_HEADER_SIZE + len + AOG_CRC_SIZE)) return false;
    if (len > AOG_MAX_PAYLOAD) return false;

    // CRC check: sum of bytes[2..(5+len-1)] must match byte[5+len]
    uint16_t sum = 0;
    size_t crc_idx = AOG_HEADER_SIZE + len;
    for (size_t i = 2; i < crc_idx; i++) {
        sum += frame[i];
    }
    uint8_t expected_crc = static_cast<uint8_t>(sum & 0xFF);
    if (frame[crc_idx] != expected_crc) {
        // Rate-limit CRC mismatch logs (max once per 10s)
        static uint32_t s_last_crc_log_ms = 0;
        extern uint32_t hal_millis(void);
        uint32_t now = hal_millis();
        if (now - s_last_crc_log_ms >= 10000) {
            s_last_crc_log_ms = now;
            // Include hex dump for diagnosis
            char hexbuf[128];
            char* p = hexbuf;
            for (size_t i = 0; i < frame_len && (p - hexbuf) < (int)sizeof(hexbuf) - 4; i++) {
                p += snprintf(p, 4, "%02X ", frame[i]);
            }
            hal_log("AOG: CRC mismatch: got 0x%02X, exp 0x%02X (Src=%u PGN=0x%02X) [%s]",
                    frame[crc_idx], expected_crc, src, pgn, hexbuf);
        }
        return false;
    }

    // Fill outputs
    if (out_src) *out_src = src;
    if (out_pgn) *out_pgn = pgn;
    if (out_payload) *out_payload = frame + AOG_HEADER_SIZE;
    if (out_payload_len) *out_payload_len = len;

    return true;
}

// ===================================================================
// Encoder: Hello Reply – Steer (Src=0x7E, PGN=0x7E, Len=5)
// Payload: AngleLo, AngleHi, CountsLo, CountsHi, Switchbyte
// ===================================================================
size_t encodeAogHelloReplySteer(uint8_t* buf, size_t buf_size,
                                int16_t steerAngle,
                                uint16_t sensorCounts,
                                uint8_t switchByte) {
    AogHelloReplySteer reply;
    reply.steerAngle   = steerAngle;
    reply.sensorCounts = sensorCounts;
    reply.switchByte   = switchByte;
    return aogBuildFrame(buf, buf_size, AOG_SRC_STEER, PGN_HELLO_REPLY_STEER,
                         &reply, sizeof(reply));
}

// ===================================================================
// Encoder: Hello Reply – GPS (Src=0x78, PGN=0x78, Len=5)
// Payload: all reserved (5 zero bytes)
// ===================================================================
size_t encodeAogHelloReplyGps(uint8_t* buf, size_t buf_size) {
    AogHelloReplyGps reply;
    std::memset(&reply, 0, sizeof(reply));
    return aogBuildFrame(buf, buf_size, AOG_SRC_GPS_REPLY, PGN_HELLO_REPLY_GPS,
                         &reply, sizeof(reply));
}

// ===================================================================
// Encoder: Subnet Reply (Src=module, PGN=0xCB, Len=7)
// Payload: IP[4] + Subnet[3]
// ===================================================================
size_t encodeAogSubnetReply(uint8_t* buf, size_t buf_size,
                            uint8_t src,
                            const uint8_t ip[4],
                            const uint8_t subnet[3]) {
    AogSubnetReply reply;
    std::memcpy(reply.ip, ip, 4);
    std::memcpy(reply.subnet, subnet, 3);
    return aogBuildFrame(buf, buf_size, src, PGN_SUBNET_REPLY,
                         &reply, sizeof(reply));
}

// ===================================================================
// Encoder: Steer Status Out (PGN 0xFD, Src=0x7E, Len=8)
// ===================================================================
size_t encodeAogSteerStatusOut(uint8_t* buf, size_t buf_size,
                                int16_t actualAngleX100,
                                int16_t headingX16,
                                int16_t rollX16,
                                uint8_t switchStatus,
                                uint8_t pwmDisplay) {
    AogSteerStatusOut status;
    status.actualSteerAngle = actualAngleX100;
    status.imuHeading       = headingX16;
    status.imuRoll          = rollX16;
    status.switchStatus     = switchStatus;
    status.pwmDisplay       = pwmDisplay;
    return aogBuildFrame(buf, buf_size, AOG_SRC_STEER, PGN_STEER_STATUS_OUT,
                         &status, sizeof(status));
}

// ===================================================================
// Encoder: From Autosteer 2 (PGN 0xFA, Src=0x7E, Len=8)
// ===================================================================
size_t encodeAogFromAutosteer2(uint8_t* buf, size_t buf_size,
                                uint8_t sensorValue) {
    AogFromAutosteer2 msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.sensorValue = sensorValue;
    return aogBuildFrame(buf, buf_size, AOG_SRC_STEER, PGN_FROM_AUTOSTEER_2,
                         &msg, sizeof(msg));
}

// ===================================================================
// Encoder: GPS Main Out (PGN 0xD6, Src=0x7C)
// ===================================================================
size_t encodeAogGpsMainOut(uint8_t* buf, size_t buf_size,
                            const AogGpsMainOut& gps) {
    return aogBuildFrame(buf, buf_size, AOG_SRC_GPS, PGN_GPS_MAIN_OUT,
                         &gps, sizeof(gps));
}

// ===================================================================
// Encoder: Hardware Message (PGN 0xDD, variable length)
// Payload: Duration(1) | Color(1) | Message(null-terminated)
// ===================================================================
size_t encodeAogHardwareMessage(uint8_t* buf, size_t buf_size,
                                 uint8_t src,
                                 uint8_t duration,
                                 uint8_t color,
                                 const char* message) {
    if (!message) return 0;

    // Calculate message length (including null terminator)
    size_t msg_len = std::strlen(message);
    if (msg_len == 0) return 0;
    if (msg_len > AOG_HWMSG_MAX_TEXT) msg_len = AOG_HWMSG_MAX_TEXT;

    // Total payload: duration(1) + color(1) + message(msg_len+1)
    size_t payload_len = 2 + msg_len + 1;

    size_t total = AOG_HEADER_SIZE + payload_len + AOG_CRC_SIZE;
    if (buf_size < total) {
        hal_log("AOG: HW message buffer too small: need %zu, have %zu", total, buf_size);
        return 0;
    }

    // Build payload in-place after header
    size_t pos = AOG_HEADER_SIZE;
    buf[pos++] = duration;
    buf[pos++] = color;
    std::memcpy(buf + pos, message, msg_len);
    pos += msg_len;
    buf[pos++] = '\0';  // null terminator

    // Write header
    buf[0] = AOG_ID_1;
    buf[1] = AOG_ID_2;
    buf[2] = src;
    buf[3] = PGN_HARDWARE_MESSAGE;
    buf[4] = static_cast<uint8_t>(payload_len);

    // CRC over bytes 2..(total-2)
    uint16_t sum = 0;
    for (size_t i = 2; i < total - 1; i++) {
        sum += buf[i];
    }
    buf[total - 1] = static_cast<uint8_t>(sum & 0xFF);

    return total;
}

// ===================================================================
// Decoder: Hello From AgIO (PGN 200, Len=3)
// ===================================================================
bool tryDecodeAogHelloFromAgio(const uint8_t* payload, size_t payload_len,
                                AogHelloFromAgio* out) {
    if (!payload || payload_len < sizeof(AogHelloFromAgio)) return false;

    AogHelloFromAgio msg;
    std::memcpy(&msg, payload, sizeof(msg));

    if (out) *out = msg;
    hal_log("AOG: Hello from AgIO (module=%u, ver=%u)",
            (unsigned)msg.moduleId, (unsigned)msg.agioVersion);
    return true;
}

// ===================================================================
// Decoder: Scan Request (PGN 202, Len=3)
// ===================================================================
bool tryDecodeAogScanRequest(const uint8_t* payload, size_t payload_len) {
    if (!payload || payload_len < 1) return false;
    hal_log("AOG: Scan request received (len=%zu)", payload_len);
    return true;
}

// ===================================================================
// Decoder: Subnet Change (PGN 201, Len=5)
// ===================================================================
bool tryDecodeAogSubnetChange(const uint8_t* payload, size_t payload_len,
                               AogSubnetChange* out) {
    if (!payload || payload_len < sizeof(AogSubnetChange)) return false;

    AogSubnetChange msg;
    std::memcpy(&msg, payload, sizeof(msg));

    if (out) *out = msg;
    hal_log("AOG: Subnet change -> IP=%u.%u.%u.xxx",
            (unsigned)msg.ip_one, (unsigned)msg.ip_two, (unsigned)msg.ip_three);
    return true;
}

// ===================================================================
// Decoder: Steer Data In (PGN 254, Len=8)
// ===================================================================
bool tryDecodeAogSteerDataIn(const uint8_t* payload, size_t payload_len,
                              AogSteerDataIn* out) {
    if (!payload || payload_len < sizeof(AogSteerDataIn)) return false;

    AogSteerDataIn msg;
    std::memcpy(&msg, payload, sizeof(msg));

    if (out) *out = msg;
    hal_log("AOG: SteerDataIn speed=%d angle=%d status=0x%02X",
            (int)msg.speed, (int)msg.steerAngle, (int)msg.status);
    return true;
}

// ===================================================================
// Decoder: Steer Settings In (PGN 252, Len=17)
// ===================================================================
bool tryDecodeAogSteerSettingsIn(const uint8_t* payload, size_t payload_len,
                                 AogSteerSettingsIn* out) {
    if (!payload || payload_len < sizeof(AogSteerSettingsIn)) return false;

    AogSteerSettingsIn msg;
    std::memcpy(&msg, payload, sizeof(msg));

    if (out) *out = msg;
    hal_log("AOG: SteerSettings ack=%u Kp=%u Ki=%u Kd=%u minPWM=%u maxPWM=%u counts=%u hi=%d lo=%d wasOff=%d width=%u",
            (unsigned)msg.ackNumber, (unsigned)msg.kp, (unsigned)msg.ki, (unsigned)msg.kd,
            (unsigned)msg.minPWM, (unsigned)msg.maxPWM, (unsigned)msg.counts,
            (int)msg.hiLimit, (int)msg.loLimit, (int)msg.wasOffset,
            (unsigned)msg.machineWidth);
    return true;
}

// ===================================================================
// Decoder: Hardware Message (PGN 221, variable length)
// ===================================================================
bool tryDecodeAogHardwareMessage(const uint8_t* payload, size_t payload_len,
                                  uint8_t* out_duration,
                                  uint8_t* out_color,
                                  char* out_message, size_t out_msg_size) {
    // Minimum: duration(1) + color(1) + null terminator(1) = 3
    if (!payload || payload_len < 3) return false;

    uint8_t duration = payload[0];
    uint8_t color    = payload[1];

    // Message starts at payload[2], null-terminated
    const char* msg = reinterpret_cast<const char*>(payload + 2);

    // Ensure null terminator exists within payload
    size_t max_msg_len = payload_len - 2;
    size_t msg_len = 0;
    while (msg_len < max_msg_len && msg[msg_len] != '\0') {
        msg_len++;
    }

    if (out_duration) *out_duration = duration;
    if (out_color)    *out_color = color;

    if (out_message && out_msg_size > 0) {
        size_t copy_len = msg_len < (out_msg_size - 1) ? msg_len : (out_msg_size - 1);
        std::memcpy(out_message, msg, copy_len);
        out_message[copy_len] = '\0';
    }

    hal_log("AOG: HW Message dur=%u color=%u msg=\"%s\"", duration, color, msg);
    return true;
}

// ===================================================================
// Utility: Hex dump to log
// ===================================================================
void aogHexDump(const char* label, const uint8_t* data, size_t len) {
    hal_log("%s (%zu bytes):", label ? label : "AOG dump", len);
    for (size_t i = 0; i < len; i += 16) {
        char line[80];
        int pos = 0;
        pos += snprintf(line + pos, sizeof(line) - pos, "  %04zx: ", i);
        for (size_t j = i; j < i + 16 && j < len; j++) {
            pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", data[j]);
        }
        hal_log("%s", line);
    }
}
