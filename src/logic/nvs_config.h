/**
 * @file nvs_config.h
 * @brief NVS persistence helpers for RuntimeConfig — Phase 0 (S0-05).
 */

#pragma once

#include "runtime_config.h"

namespace nvs_keys {
constexpr const char* NS = "agsteer";
// NTRIP
constexpr const char* NTRIP_HOST = "ntrip_host";
constexpr const char* NTRIP_PORT = "ntrip_port";
constexpr const char* NTRIP_MOUNT = "ntrip_mnt";
constexpr const char* NTRIP_USER = "ntrip_user";
constexpr const char* NTRIP_PASS = "ntrip_password";
// PID
constexpr const char* PID_KP = "pid_kp";
constexpr const char* PID_KI = "pid_ki";
constexpr const char* PID_KD = "pid_kd";
// Network
constexpr const char* NET_MODE = "net_mode";
constexpr const char* NET_IP = "net_ip";
constexpr const char* NET_GW = "net_gateway";
constexpr const char* NET_SUBNET = "net_subnet";
// Actuator
constexpr const char* ACT_TYPE = "actuator_type";
}  // namespace nvs_keys

/// Lädt alle gespeicherten Werte aus NVS in cfg.
void nvsConfigLoad(RuntimeConfig& cfg);

/// Speichert alle Werte aus cfg in NVS.
/// Gibt true zurück bei Erfolg.
bool nvsConfigSave(const RuntimeConfig& cfg);

/// Löscht den gesamten Namespace "agsteer".
void nvsConfigFactoryReset(void);

/// Prüft ob bereits NVS-Daten im Namespace vorhanden sind.
bool nvsConfigHasData(void);
