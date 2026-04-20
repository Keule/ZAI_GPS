#pragma once

#include <cstdint>

namespace cfg {

inline constexpr const char* NTRIP_HOST = "";
inline constexpr uint16_t NTRIP_PORT = 2101;
inline constexpr const char* NTRIP_MOUNTPOINT = "";
inline constexpr const char* NTRIP_USER = "";
inline constexpr const char* NTRIP_PASSWORD = "";

inline constexpr uint32_t GNSS_BAUD = 115200;
inline constexpr uint32_t GNSS_RETRY_TIMEOUT_MS = 5000;

inline constexpr bool NET_DHCP = true;
inline constexpr uint8_t NET_STATIC_IP[4] = {192, 168, 1, 70};

inline constexpr bool LOGGING_ENABLED_DEFAULT = false;
inline constexpr uint32_t LOG_INTERVAL_MS = 100;

inline constexpr int8_t MOD_STEER_DEFAULT = 1;
inline constexpr int8_t MOD_GPS_DEFAULT = 1;
inline constexpr int8_t MOD_MACHINE_DEFAULT = 1;
inline constexpr int8_t MOD_NTRIP_DEFAULT = 1;

}  // namespace cfg
