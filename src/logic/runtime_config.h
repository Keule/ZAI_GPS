/**
 * @file runtime_config.h
 * @brief Mutable RAM copy of user configuration — TASK-028.
 *
 * Loaded from cfg:: defaults at boot, can be overridden at runtime
 * via Serial / SD card / WebUI.
 */
#pragma once
#include <cstdint>

/// Mutable RAM copy of user configuration.
/// Loaded from cfg:: defaults at boot, can be overridden at runtime.
struct RuntimeConfig {
    // NTRIP
    char     ntrip_host[64];
    uint16_t ntrip_port;
    char     ntrip_mountpoint[48];
    char     ntrip_user[32];
    char     ntrip_password[32];
    uint32_t ntrip_reconnect_ms;

    // GNSS
    uint32_t gnss_baud;

    // Logging
    uint32_t log_interval_ms;
    bool     log_default_active;
};

/// Load cfg:: defaults into a RuntimeConfig instance.
void softConfigLoadDefaults(RuntimeConfig& cfg);

/// Load user overrides (stub — future: SD card, Serial, WebUI).
/// Returns true if overrides were loaded, false if no overrides available.
bool softConfigLoadOverrides(RuntimeConfig& cfg);

/// Load user overrides from NVS flash.
/// Returns true if persisted values were found and applied.
bool softConfigLoadFromNvs(RuntimeConfig& cfg);

/// Persist runtime config to NVS flash.
/// Returns true on success.
bool softConfigSaveToNvs(const RuntimeConfig& cfg);

/// Get the global runtime config instance.
RuntimeConfig& softConfigGet(void);
