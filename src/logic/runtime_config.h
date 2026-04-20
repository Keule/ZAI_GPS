#pragma once

#include <cstdint>

struct RuntimeConfig {
    char ntrip_host[64];
    uint16_t ntrip_port;
    char ntrip_mountpoint[48];
    char ntrip_user[32];
    char ntrip_password[32];
    uint32_t gnss_baud;
    uint32_t gnss_retry_timeout_ms;
    bool net_dhcp;
    uint8_t net_static_ip[4];
    bool logging_default;
    uint32_t log_interval_ms;
};

void softConfigLoadDefaults(RuntimeConfig* out_cfg);
bool softConfigLoadOverrides(RuntimeConfig* io_cfg);

