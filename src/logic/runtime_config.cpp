/**
 * @file runtime_config.cpp
 * @brief RuntimeConfig implementation — TASK-028.
 *
 * Provides the global RuntimeConfig instance and functions to
 * load compile-time defaults and runtime overrides.
 */

#include "runtime_config.h"
#include "soft_config.h"

#include <cstring>

/// Global runtime configuration instance (zero-initialised).
static RuntimeConfig s_runtime_config = {};

void softConfigLoadDefaults(RuntimeConfig& cfg) {
    // NTRIP
    std::strncpy(cfg.ntrip_host, cfg::NTRIP_HOST, sizeof(cfg.ntrip_host) - 1);
    cfg.ntrip_host[sizeof(cfg.ntrip_host) - 1] = '\0';
    cfg.ntrip_port = cfg::NTRIP_PORT;
    std::strncpy(cfg.ntrip_mountpoint, cfg::NTRIP_MOUNTPOINT, sizeof(cfg.ntrip_mountpoint) - 1);
    cfg.ntrip_mountpoint[sizeof(cfg.ntrip_mountpoint) - 1] = '\0';
    std::strncpy(cfg.ntrip_user, cfg::NTRIP_USER, sizeof(cfg.ntrip_user) - 1);
    cfg.ntrip_user[sizeof(cfg.ntrip_user) - 1] = '\0';
    std::strncpy(cfg.ntrip_password, cfg::NTRIP_PASSWORD, sizeof(cfg.ntrip_password) - 1);
    cfg.ntrip_password[sizeof(cfg.ntrip_password) - 1] = '\0';
    cfg.ntrip_reconnect_ms = cfg::NTRIP_RECONNECT_MS;

    // GNSS
    cfg.gnss_baud = cfg::GNSS_BAUD;

    // Logging
    cfg.log_interval_ms = cfg::LOG_INTERVAL_MS;
    cfg.log_default_active = cfg::LOG_DEFAULT_ACTIVE;
}

bool softConfigLoadOverrides(RuntimeConfig& cfg) {
    // Stub: future implementation for SD card / Serial / WebUI
    (void)cfg;
    return false;  // "not yet implemented"
}

RuntimeConfig& softConfigGet(void) {
    return s_runtime_config;
}
