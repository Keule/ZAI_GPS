/**
 * @file aog_udp_protocol.cpp
 * @brief AgOpenGPS / AgIO UDP protocol – encoder, decoder, checksum.
 *
 * Pure C++ – no Arduino / ESP32 headers.
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
// Checksum: low byte of sum from byte[2] to byte[n-2]
// (excludes preamble bytes 0,1 and the CRC byte at the end)
// ===================================================================
uint8_t aogChecksum(const uint8_t* frame, size_t frame_len) {
    if (frame_len < AOG_HEADER_SIZE) return 0;

    uint16_t sum = 0;
    // Start at byte[2] (Src), go up to but not including the CRC byte
    for (size_t i = 2; i + 1 < frame_len; i++) {
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
        hal_log("AOG: CRC mismatch: got 0x%02X, expected 0x%02X (Src=%u PGN=0x%02X)",
                frame[crc_idx], expected_crc, src, pgn);
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
// Encoder: Hello Reply – Steer (Src=0x7E, PGN=0x7E)
// ===================================================================
size_t encodeAogHelloReplySteer(uint8_t* buf, size_t buf_size, uint16_t address) {
    AogHelloReplySteer reply;
    reply.address = address;
    return aogBuildFrame(buf, buf_size, AOG_SRC_STEER, PGN_HELLO_REPLY_STEER,
                         &reply, sizeof(reply));
}

// ===================================================================
// Encoder: Hello Reply – GPS (Src=0x78, PGN=0x78)
// ===================================================================
size_t encodeAogHelloReplyGps(uint8_t* buf, size_t buf_size, uint16_t address) {
    AogHelloReplyGps reply;
    reply.address = address;
    return aogBuildFrame(buf, buf_size, 0x78, PGN_HELLO_REPLY_GPS,
                         &reply, sizeof(reply));
}

// ===================================================================
// Encoder: Subnet Reply (Src=module, PGN=0xCB)
// ===================================================================
size_t encodeAogSubnetReply(uint8_t* buf, size_t buf_size, uint8_t src, uint16_t address) {
    AogSubnetReply reply;
    reply.address = address;
    return aogBuildFrame(buf, buf_size, src, PGN_SUBNET_REPLY,
                         &reply, sizeof(reply));
}

// ===================================================================
// Encoder: Steer Status Out (PGN 0xFD, Src=0x7E)
// ===================================================================
size_t encodeAogSteerStatusOut(uint8_t* buf, size_t buf_size,
                                int16_t actualAngleX100,
                                int16_t headingX10,
                                int16_t rollX10,
                                uint8_t switchStatus,
                                uint8_t pwmDisplay) {
    AogSteerStatusOut status;
    status.actualSteerAngle = actualAngleX100;
    status.imuHeading = headingX10;
    status.imuRoll = rollX10;
    status.switchStatus = switchStatus;
    status.pwmDisplay = pwmDisplay;
    return aogBuildFrame(buf, buf_size, AOG_SRC_STEER, PGN_STEER_STATUS_OUT,
                         &status, sizeof(status));
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
// Decoder: Hello From AgIO (PGN 200)
// ===================================================================
bool tryDecodeAogHelloFromAgio(const uint8_t* payload, size_t payload_len,
                                AogHelloFromAgio* out) {
    if (!payload || payload_len < sizeof(AogHelloFromAgio)) return false;

    AogHelloFromAgio msg;
    std::memcpy(&msg, payload, sizeof(msg));

    if (out) *out = msg;
    hal_log("AOG: Hello from AgIO (ver=%u)", msg.agioVersion);
    return true;
}

// ===================================================================
// Decoder: Scan Request (PGN 202)
// ===================================================================
bool tryDecodeAogScanRequest(const uint8_t* payload, size_t payload_len) {
    if (!payload || payload_len < 1) return false;
    hal_log("AOG: Scan request received");
    return true;
}

// ===================================================================
// Decoder: Subnet Change (PGN 201)
// ===================================================================
bool tryDecodeAogSubnetChange(const uint8_t* payload, size_t payload_len,
                               AogSubnetChange* out) {
    if (!payload || payload_len < sizeof(AogSubnetChange)) return false;

    AogSubnetChange msg;
    std::memcpy(&msg, payload, sizeof(msg));

    if (out) *out = msg;
    hal_log("AOG: Subnet change -> address=0x%04X", msg.address);
    return true;
}

// ===================================================================
// Decoder: Steer Data In (PGN 254)
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
