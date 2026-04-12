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
 * Reference: https://github.com/AgOpenGPS-Official/Boards/blob/main/PGN.md
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
constexpr uint8_t AOG_SRC_MACHINE    = 0x7B;  // 123 – Machine module
constexpr uint8_t AOG_SRC_GPS        = 0x7C;  // 124 – GPS module (this board, second role)
constexpr uint8_t AOG_SRC_GPS_REPLY  = 0x78;  // 120 – GPS module hello reply

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
constexpr uint8_t PGN_STEER_SETTINGS_IN = 0xFC;  // 252  AgIO -> steer module (settings / ACK)
constexpr uint8_t PGN_STEER_CONFIG_IN   = 0xFB;  // 251  AgIO -> steer module (config)
constexpr uint8_t PGN_FROM_AUTOSTEER_2  = 0xFA;  // 250  steer module sensor data

// -- GPS --
constexpr uint8_t PGN_GPS_MAIN_OUT      = 0xD6;  // 214  GPS module -> AgIO

// -- Hardware Message --
constexpr uint8_t PGN_HARDWARE_MESSAGE  = 0xDD;  // 221  hardware status message (bidirectional)

// ===================================================================
// Hardware Message Color Codes
// ===================================================================
constexpr uint8_t AOG_HWMSG_COLOR_GREEN  = 0;  // OK / info
constexpr uint8_t AOG_HWMSG_COLOR_RED    = 1;  // Error / critical
constexpr uint8_t AOG_HWMSG_COLOR_YELLOW = 2;  // Warning
constexpr uint8_t AOG_HWMSG_COLOR_BLUE   = 3;  // Informational

/// Special duration value: message persists until updated or dismissed
constexpr uint8_t AOG_HWMSG_DURATION_PERSIST = 0;

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
constexpr size_t AOG_HWMSG_MAX_TEXT = 200; // max text length for hardware messages

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
// Hello from AgIO (PGN 200, Src=0x7F, Len=3)
// ===================================================================

struct __attribute__((packed)) AogHelloFromAgio {
    uint8_t  moduleId;       // = 0x7F (which module this hello is for)
    uint8_t  spare;
    uint16_t agioVersion;    // AgIO version
};

// ===================================================================
// Scan Request (PGN 202, Src=0x7F, Len=3)
// ===================================================================

struct __attribute__((packed)) AogScanRequest {
    uint8_t  scanByte1;      // = 0xCA (202)
    uint8_t  scanByte2;      // = 0xCA (202)
    uint8_t  numberOfModules; // number of modules to scan (5 typically)
};

// ===================================================================
// Subnet Change (PGN 201, Src=0x7F, Len=5)
// Payload: 0xC9, 0xC9, IP_One, IP_Two, IP_Three
// Sets the first 3 octets of the destination IP.
// ===================================================================

struct __attribute__((packed)) AogSubnetChange {
    uint8_t  id1;            // = 0xC9 (201) – repeated PGN
    uint8_t  id2;            // = 0xC9 (201) – repeated PGN
    uint8_t  ip_one;         // first octet of destination IP
    uint8_t  ip_two;         // second octet of destination IP
    uint8_t  ip_three;       // third octet of destination IP
};

static_assert(sizeof(AogSubnetChange) == 5, "AogSubnetChange must be 5 bytes");

// ===================================================================
// Subnet Reply (PGN 203, Src=module, Len=7)
// Payload: IP_One..IP_Four (4 bytes) + Subnet_One..Subnet_Three (3 bytes)
// ===================================================================

struct __attribute__((packed)) AogSubnetReply {
    uint8_t  ip[4];          // module's full IP address
    uint8_t  subnet[3];      // subnet mask first 3 octets (e.g. 255.255.255)
};

static_assert(sizeof(AogSubnetReply) == 7, "AogSubnetReply must be 7 bytes");

// ===================================================================
// Hello Reply – Steer Module (PGN=0x7E, Src=0x7E, Len=5)
// Payload: AngleLo, AngleHi, CountsLo, CountsHi, Switchbyte
// ===================================================================

struct __attribute__((packed)) AogHelloReplySteer {
    int16_t  steerAngle;     // current actual steer angle (degrees * 1)
    uint16_t sensorCounts;   // raw sensor counts
    uint8_t  switchByte;     // switch status bits
};

static_assert(sizeof(AogHelloReplySteer) == 5, "AogHelloReplySteer must be 5 bytes");

// ===================================================================
// Hello Reply – GPS Module (PGN=0x78, Src=0x78, Len=5)
// Payload: all zeros/reserved (5 bytes)
// ===================================================================

struct __attribute__((packed)) AogHelloReplyGps {
    uint8_t  reserved[5];    // reserved / unused
};

static_assert(sizeof(AogHelloReplyGps) == 5, "AogHelloReplyGps must be 5 bytes");

// ===================================================================
// Steer Data In (PGN 254, Src=0x7F -> steer module)
// Total payload: 8 bytes
// ===================================================================

struct __attribute__((packed)) AogSteerDataIn {
    int16_t  speed;           // speed [cm/s], signed
    uint8_t  status;          // bitfield: bit0=workSwitch, bit1=steerSwitch, bit2=on
    int16_t  steerAngle;      // desired steer angle [degrees * 100], signed
    uint8_t  xte;             // cross track error
    uint8_t  sectionControl1; // SC1..SC8
    uint8_t  sectionControl2; // SC9..SC16
};

static_assert(sizeof(AogSteerDataIn) == 8, "AogSteerDataIn must be 8 bytes");

// ===================================================================
// Steer Settings In (PGN 252, Src=0x7F -> steer module)
// "ACK" message from AgIO containing steering parameters.
// Total payload: 17 bytes
// ===================================================================

struct __attribute__((packed)) AogSteerSettingsIn {
    uint8_t  pgn1;            // = 0xFC (repeated PGN byte)
    uint8_t  pgn2;            // = 0xFC (repeated PGN byte)
    uint8_t  ackNumber;       // acknowledgment number (0-254, wraps)
    uint8_t  kp;              // proportional gain (actual = value / 10.0)
    uint8_t  ki;              // integral gain (actual = value / 10.0)
    uint8_t  kd;              // derivative gain (actual = value / 10.0)
    uint16_t minPWM;          // minimum actuator PWM, LE
    uint16_t maxPWM;          // maximum actuator PWM, LE
    uint16_t counts;          // steer angle sensor total counts, LE
    int8_t   hiLimit;         // max steer angle to left [degrees]
    int8_t   loLimit;         // max steer angle to right [degrees]
    int16_t  wasOffset;       // sensor zero offset in counts, LE
    uint8_t  machineWidth;    // implement width [cm]
};

static_assert(sizeof(AogSteerSettingsIn) == 17, "AogSteerSettingsIn must be 17 bytes");

// ===================================================================
// Steer Status Out (PGN 253, Src=0x7E -> AgIO)
// Total payload: 8 bytes
// ===================================================================

struct __attribute__((packed)) AogSteerStatusOut {
    int16_t  actualSteerAngle;  // measured angle [degrees * 100], signed
    int16_t  imuHeading;        // IMU heading [degrees * 16], signed
    int16_t  imuRoll;           // IMU roll [degrees * 16], signed
    uint8_t  switchStatus;      // steering switch state bits
    uint8_t  pwmDisplay;        // current PWM value (0-255)
};

static_assert(sizeof(AogSteerStatusOut) == 8, "AogSteerStatusOut must be 8 bytes");

// ===================================================================
// From Autosteer 2 (PGN 250, Src=0x7E -> AgIO)
// Total payload: 8 bytes – sensor value from steer module
// ===================================================================

struct __attribute__((packed)) AogFromAutosteer2 {
    uint8_t  sensorValue;       // steer angle sensor raw value (low byte)
    uint8_t  reserved[7];       // unused
};

static_assert(sizeof(AogFromAutosteer2) == 8, "AogFromAutosteer2 must be 8 bytes");

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

/// Encode Steer Hello Reply frame (PGN=0x7E, Len=5).
/// Includes current steer angle, sensor counts, and switch status.
size_t encodeAogHelloReplySteer(uint8_t* buf, size_t buf_size,
                                int16_t steerAngle,
                                uint16_t sensorCounts,
                                uint8_t switchByte);

/// Encode GPS Hello Reply frame (PGN=0x78, Len=5).
size_t encodeAogHelloReplyGps(uint8_t* buf, size_t buf_size);

/// Encode Subnet Reply frame (PGN=0xCB, Len=7).
/// @param src        module source ID (0x7E for steer, 0x7C for GPS, etc.)
/// @param ip         module's full IP address (4 bytes)
/// @param subnet     subnet mask (3 bytes: typically 255,255,255)
size_t encodeAogSubnetReply(uint8_t* buf, size_t buf_size,
                            uint8_t src,
                            const uint8_t ip[4],
                            const uint8_t subnet[3]);

/// Encode Steer Status Out frame (PGN=0xFD, Len=8).
size_t encodeAogSteerStatusOut(uint8_t* buf, size_t buf_size,
                                int16_t actualAngleX100,
                                int16_t headingX16,
                                int16_t rollX16,
                                uint8_t switchStatus,
                                uint8_t pwmDisplay);

/// Encode From Autosteer 2 frame (PGN=0xFA, Len=8).
size_t encodeAogFromAutosteer2(uint8_t* buf, size_t buf_size,
                                uint8_t sensorValue);

/// Encode GPS Main Out frame.
size_t encodeAogGpsMainOut(uint8_t* buf, size_t buf_size,
                            const AogGpsMainOut& gps);

/// Encode Hardware Message frame (PGN=0xDD, variable length).
/// @param src      module source ID (0x7E for steer, 0x7C for GPS)
/// @param duration display duration in seconds (0 = persistent)
/// @param color    color code (0=green, 1=red, 2=yellow, 3=blue)
/// @param message  null-terminated ASCII message text
/// @return total frame length, or 0 on error
size_t encodeAogHardwareMessage(uint8_t* buf, size_t buf_size,
                                 uint8_t src,
                                 uint8_t duration,
                                 uint8_t color,
                                 const char* message);

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

/// Try to decode Steer Settings In (PGN 252).
bool tryDecodeAogSteerSettingsIn(const uint8_t* payload, size_t payload_len,
                                 AogSteerSettingsIn* out);

/// Try to decode a Hardware Message (PGN 221).
/// Fills out_duration, out_color, out_message (null-terminated).
bool tryDecodeAogHardwareMessage(const uint8_t* payload, size_t payload_len,
                                  uint8_t* out_duration,
                                  uint8_t* out_color,
                                  char* out_message, size_t out_msg_size);

// ===================================================================
// Utility
// ===================================================================

/// Hex-dump a buffer to log (for debugging).
void aogHexDump(const char* label, const uint8_t* data, size_t len);
