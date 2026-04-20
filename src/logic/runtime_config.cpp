#include "runtime_config.h"

#include "soft_config.h"

#include <cstring>

void softConfigLoadDefaults(RuntimeConfig* out_cfg) {
    if (!out_cfg) return;

    std::strncpy(out_cfg->ntrip_host, cfg::NTRIP_HOST, sizeof(out_cfg->ntrip_host) - 1);
    out_cfg->ntrip_host[sizeof(out_cfg->ntrip_host) - 1] = '\0';
    out_cfg->ntrip_port = cfg::NTRIP_PORT;
    std::strncpy(out_cfg->ntrip_mountpoint, cfg::NTRIP_MOUNTPOINT, sizeof(out_cfg->ntrip_mountpoint) - 1);
    out_cfg->ntrip_mountpoint[sizeof(out_cfg->ntrip_mountpoint) - 1] = '\0';
    std::strncpy(out_cfg->ntrip_user, cfg::NTRIP_USER, sizeof(out_cfg->ntrip_user) - 1);
    out_cfg->ntrip_user[sizeof(out_cfg->ntrip_user) - 1] = '\0';
    std::strncpy(out_cfg->ntrip_password, cfg::NTRIP_PASSWORD, sizeof(out_cfg->ntrip_password) - 1);
    out_cfg->ntrip_password[sizeof(out_cfg->ntrip_password) - 1] = '\0';

    out_cfg->gnss_baud = cfg::GNSS_BAUD;
    out_cfg->gnss_retry_timeout_ms = cfg::GNSS_RETRY_TIMEOUT_MS;
    out_cfg->net_dhcp = cfg::NET_DHCP;
    std::memcpy(out_cfg->net_static_ip, cfg::NET_STATIC_IP, sizeof(out_cfg->net_static_ip));
    out_cfg->logging_default = cfg::LOGGING_ENABLED_DEFAULT;
    out_cfg->log_interval_ms = cfg::LOG_INTERVAL_MS;
}

bool softConfigLoadOverrides(RuntimeConfig* io_cfg) {
    (void)io_cfg;
    return false;
}
