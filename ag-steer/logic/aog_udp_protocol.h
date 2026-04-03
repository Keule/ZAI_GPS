/**
 * @file aog_udp_protocol.h
 * @brief AgOpenGPS / AgIO UDP protocol definitions, frame structs, and codec.
 *
 * Frame layout (AgOpenGPS Ethernet protocol):
 *   Byte0  Byte1  Byte2  Byte3  Byte4  Data...    CRC
 *   0x80   0x81   Src    PGN    Len    <payload>  checksum
 *
 * CRC = low byte of sum(Byte2 .. Byte[n-2])  (Src + PGN + Len + all data bytes)
 *
 * This header contains ONLY constants, structs, and function declarations.
 * No Arduino/ESP32 headers.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

// ===================================================================
// AOG Preamble
// ===================================================================
constexpr uint8_t AOG_ID_1 = 0x80;
constexpr uint8_t AOG_ID_2 = 0x81;

// ===================================================================
// Source IDs (Src field)
// ===================================================================
constexpr uint8_t AOG_SRC_AGIO       = 0x7F;  // 127 – AgIO application
constexpr uint8_t AOG_SRC_STEER      = 0x7E;  // 126 – Steer module (this board)
constexpr uint8_t AOG_SRC_GPS        = 0x7C;  // 124 – GPS module (this board, second role)

// ===================================================================
// PGN Numbers
// ===================================================================

// -- Hello / Scan / Network discovery --
constexpr uint8_t PGN_HELLO_FROM_AGIO   = 0xC8;  // 200  AgIO hello broadcast
constexpr uint8_t PGN_SUBNET_CHANGE     = 0xC9;  // 201  subnet change request
constexpr uint8_t PGN_SCAN_REQUEST      = 0xCA;  // 202  scan / discover modules
constexpr uint8_t PGN_SUBNET_REPLY      = 0xCB;  // 203  subnet info reply

// -- Hello Replies (module -> AgIO) --
// PGN = Src for hello replies
constexpr uint8_t PGN_HELLO_REPLY_STEER = 0x7E;  // 126  steer module hello reply
constexpr uint8_t PGN_HELLO_REPLY_GPS   = 0x78;  // 120  GPS module hello reply

// -- Steer --
constexpr uint8_t PGN_STEER_DATA_IN     = 0xFE;  // 254  AgIO -> steer module
constexpr uint8_t PGN_STEER_STATUS_OUT  = 0xFD;  // 253  steer module -> AgIO
constexpr uint8_t PGN_STEER_SETTINGS_IN = 0xFC;  // 252  AgIO -> steer module (settings)
constexpr uint8_t PGN_STEER_CONFIG_IN   = 0xFB;  // 251  AgIO -> steer module (config)

// -- GPS --
constexpr uint8_t PGN_GPS_MAIN_OUT      = 0xD6;  // 214  GPS module -> AgIO

// ===================================================================
// UDP Ports
// ===================================================================
constexpr uint16_t AOG_PORT_STEER = 5126;
constexpr uint16_t AOG_PORT_GPS   = 5124;
constexpr uint16_t AOG_PORT_AGIO  = 9999;  // AgIO listens here for incoming data
constexpr uint16_t AOG_PORT_NMEA  = 29999;

// ===================================================================
// Default network destination (override at runtime via subnet change)
// ===================================================================
struct AogNetworkConfig {
    uint8_t  dest_ip[4]  = {192, 168, 1, 255};  // broadcast default
    uint16_t dest_port_agio = AOG_PORT_AGIO;
};

extern AogNetworkConfig g_net_cfg;

// ===================================================================
// Maximum frame sizes
// ===================================================================
constexpr size_t AOG_HEADER_SIZE  = 5;   // 0x80 + 0x81 + Src + PGN + Len
constexpr size_t AOG_CRC_SIZE     = 1;
constexpr size_t AOG_MAX_FRAME    = 256;  // safety limit for receive buffer
constexpr size_t AOG_MAX_PAYLOAD  = AOG_MAX_FRAME - AOG_HEADER_SIZE - AOG_CRC_SIZE;

// ===================================================================
// AOG Frame Header (5 bytes on the wire)
// ===================================================================
struct __attribute__((packed)) AogFrameHeader {
    uint8_t  preamble[2];   // 0x80, 0x81
    uint8_t  src;
    uint8_t  pgn;
    uint8_t  len;           // length of payload following this header
};

// ===================================================================
// Hello from AgIO (PGN 200, Src=0x7F)
// ===================================================================

struct __attribute__((packed)) AogHelloFromAgio {
    // payload after header
    uint8_t  helloFromAgio;  // = 0x7F
    uint8_t  spare;
    uint16_t agioVersion;    // AgIO version
};

// ===================================================================
// Scan Request (PGN 202, Src=0x7F)
// ===================================================================

struct __attribute__((packed)) AogScanRequest {
    uint8_t  scanRequest;  // = 0x7F
};

// ===================================================================
// Subnet Change (PGN 201, Src=0x7F)
// ===================================================================

struct __attribute__((packed)) AogSubnetChange {
    uint16_t address;       // new IP subnet address
};

// ===================================================================
// Subnet Reply (PGN 203, Src=module)
// ===================================================================

struct __attribute__((packed)) AogSubnetReply {
    uint16_t address;
};

// ===================================================================
// Hello Reply – Steer Module (PGN=0x7E, Src=0x7E)
// ===================================================================

struct __attribute__((packed)) AogHelloReplySteer {
    uint16_t address;       // module IP (last two octets as uint16)
};

// ===================================================================
// Hello Reply – GPS Module (PGN=0x78, Src=0x78)
// ===================================================================

struct __attribute__((packed)) AogHelloReplyGps {
    uint16_t address;       // module IP (last two octets as uint16)
};

// ===================================================================
// Steer Data In (PGN 254, Src=0x7F -> steer module)
// Total payload: 8 bytes
// ===================================================================

struct __attribute__((packed)) AogSteerDataIn {
    int16_t  speed;           // speed [cm/s], signed
    uint8_t  status;          // bitfield: work switch, section states, etc.
    int16_t  steerAngle;      // desired steer angle [degrees * 100], signed
    uint8_t  sectionControl1; // SC1..SC8
    uint8_t  sectionControl2; // SC9..SC16
    uint8_t  sectionControl3; // SC17..SC24 (or reserved)
};

// ===================================================================
// Steer Status Out (PGN 253, Src=0x7E -> AgIO)
// Total payload: 8 bytes
// ===================================================================

struct __attribute__((packed)) AogSteerStatusOut {
    int16_t  actualSteerAngle;  // measured angle [degrees * 100], signed
    int16_t  imuHeading;        // IMU heading [degrees * 10], signed
    int16_t  imuRoll;           // IMU roll [degrees * 10], signed
    uint8_t  switchStatus;      // steering switch state bits
    uint8_t  pwmDisplay;        // current PWM value (0-255)
};

// ===================================================================
// GPS Main Out (PGN 214, Src=0x7C -> AgIO)
// Total payload: 51 bytes
// ===================================================================

struct __attribute__((packed)) AogGpsMainOut {
    // [0..3]   Longitude  [degrees * 1e7]
    int32_t  longitude;
    // [4..7]   Latitude   [degrees * 1e7]
    int32_t  latitude;
    // [8..9]   Heading (true) [degrees * 16]
    int16_t  heading;
    // [10..11] Dual heading (true) [degrees * 16]
    int16_t  dualHeading;
    // [12..13] Speed [mm/s]
    uint16_t speed;
    // [14..15] Roll [degrees * 16]
    int16_t  roll;
    // [16..19] Altitude [mm]
    int32_t  altitude;
    // [20]     Satellites tracked
    uint8_t  satCount;
    // [21]     Fix quality (0=none,1=GPS,2=DGPS,4=RTK Fix,5=RTK Float)
    uint8_t  fixQuality;
    // [22..23] HDOP * 100
    int16_t  hdop;
    // [24..25] Age (differential age) * 100 [ms]
    int16_t  age;
    // [26..27] IMU heading [degrees * 16]
    int16_t  imuHeading;
    // [28..29] IMU roll [degrees * 16]
    int16_t  imuRoll;
    // [30..31] IMU pitch [degrees * 16]
    int16_t  imuPitch;
    // [32..33] IMU yaw rate [degrees/sec * ??]
    int16_t  imuYawRate;
    // [34..35] reserved
    int16_t  reserved1;
    // [36..39] section / relay bits
    uint32_t relay;
    // [40..41] Section control
    uint8_t  section1_8;
    uint8_t  section9_16;
    // [42..43] Year
    uint16_t year;
    // [44]     Month
    uint8_t  month;
    // [45]     Day
    uint8_t  day;
    // [46]     Hour
    uint8_t  hour;
    // [47]     Minute
    uint8_t  minute;
    // [48..50] second (units TBD, stored as milliseconds)
    uint8_t  second;
    uint8_t  ms_hi;
    uint8_t  ms_lo;
};

static_assert(sizeof(AogGpsMainOut) == 51, "AogGpsMainOut must be exactly 51 bytes");

// ===================================================================
// Checksum
// ===================================================================

/// Compute AOG checksum: low byte of sum of bytes from offset to end-1.
/// For a complete frame, pass data pointing to Src byte, len = total frame - 3
/// (excludes preamble 0x80 0x81 and the CRC byte itself).
///
/// Simpler form: pass the whole frame (including preamble), length = frame_length - 1
/// (excludes the last CRC byte), and we sum from byte[2] onward.
uint8_t aogChecksum(const uint8_t* frame, size_t frame_len);

// ===================================================================
// Encoder functions (return total frame length including preamble & CRC)
// ===================================================================

/// Build a complete AOG frame in buf: preamble + header + payload + CRC.
/// @return total bytes written to buf.
size_t aogBuildFrame(uint8_t* buf, size_t buf_size,
                     uint8_t src, uint8_t pgn,
                     const void* payload, size_t payload_len);

/// Encode Steer Hello Reply frame.
size_t encodeAogHelloReplySteer(uint8_t* buf, size_t buf_size, uint16_t address);

/// Encode GPS Hello Reply frame.
size_t encodeAogHelloReplyGps(uint8_t* buf, size_t buf_size, uint16_t address);

/// Encode Subnet Reply frame.
size_t encodeAogSubnetReply(uint8_t* buf, size_t buf_size, uint8_t src, uint16_t address);

/// Encode Steer Status Out frame.
size_t encodeAogSteerStatusOut(uint8_t* buf, size_t buf_size,
                                int16_t actualAngleX100,
                                int16_t headingX10,
                                int16_t rollX10,
                                uint8_t switchStatus,
                                uint8_t pwmDisplay);

/// Encode GPS Main Out frame.
size_t encodeAogGpsMainOut(uint8_t* buf, size_t buf_size,
                            const AogGpsMainOut& gps);

// ===================================================================
// Decoder functions – return true if frame was valid and decoded.
// ===================================================================

/// Validate AOG frame header (preamble, length bounds, CRC).
/// If valid, fills out_src, out_pgn, out_payload, out_payload_len.
bool aogValidateFrame(const uint8_t* frame, size_t frame_len,
                       uint8_t* out_src, uint8_t* out_pgn,
                       const uint8_t** out_payload, size_t* out_payload_len);

/// Try to decode a Hello From AgIO message (PGN 200).
bool tryDecodeAogHelloFromAgio(const uint8_t* payload, size_t payload_len,
                                AogHelloFromAgio* out);

/// Try to decode a Scan Request (PGN 202).
bool tryDecodeAogScanRequest(const uint8_t* payload, size_t payload_len);

/// Try to decode a Subnet Change (PGN 201).
bool tryDecodeAogSubnetChange(const uint8_t* payload, size_t payload_len,
                               AogSubnetChange* out);

/// Try to decode Steer Data In (PGN 254).
bool tryDecodeAogSteerDataIn(const uint8_t* payload, size_t payload_len,
                              AogSteerDataIn* out);

// ===================================================================
// Utility
// ===================================================================

/// Hex-dump a buffer to log (for debugging).
void aogHexDump(const char* label, const uint8_t* data, size_t len);
