#include "config_mode.h"

#include <Arduino.h>
#include <esp_system.h>

#include <cctype>
#include <cstring>
#include <strings.h>

#include "hal/hal.h"
#include "runtime_config.h"

namespace {

static constexpr uint32_t CONFIG_MODE_HEARTBEAT_MS = 5000;
static constexpr uint32_t CONFIG_MODE_IDLE_TIMEOUT_MS = 30000;

struct ParsedLine {
    char* cmd;
    char* arg1;
    char* arg2;
};

static void printPrompt() {
    Serial.print("cfg> ");
    Serial.flush();
}

static void printHelp() {
    Serial.println("Commands:");
    Serial.println("  help");
    Serial.println("  show");
    Serial.println("  set <key> <value>");
    Serial.println("  save");
    Serial.println("  reboot");
    Serial.println("  exit");
}

static void printConfig(const RuntimeConfig& cfg) {
    Serial.println("--- RuntimeConfig ---");
    Serial.printf("ntrip_host=%s\n", cfg.ntrip_host);
    Serial.printf("ntrip_port=%u\n", static_cast<unsigned>(cfg.ntrip_port));
    Serial.printf("ntrip_mountpoint=%s\n", cfg.ntrip_mountpoint);
    Serial.printf("ntrip_user=%s\n", cfg.ntrip_user);
    Serial.printf("ntrip_password=%s\n", cfg.ntrip_password[0] ? "***" : "");
    Serial.printf("ntrip_reconnect_ms=%lu\n", static_cast<unsigned long>(cfg.ntrip_reconnect_ms));
    Serial.printf("gnss_baud=%lu\n", static_cast<unsigned long>(cfg.gnss_baud));
    Serial.printf("log_interval_ms=%lu\n", static_cast<unsigned long>(cfg.log_interval_ms));
    Serial.printf("log_default_active=%s\n", cfg.log_default_active ? "true" : "false");
}

static bool parseBool(const char* s, bool* out) {
    if (!s || !out) return false;
    if (strcasecmp(s, "1") == 0 || strcasecmp(s, "true") == 0 || strcasecmp(s, "on") == 0 ||
        strcasecmp(s, "yes") == 0) {
        *out = true;
        return true;
    }
    if (strcasecmp(s, "0") == 0 || strcasecmp(s, "false") == 0 || strcasecmp(s, "off") == 0 ||
        strcasecmp(s, "no") == 0) {
        *out = false;
        return true;
    }
    return false;
}

static bool parseUint32(const char* s, uint32_t* out) {
    if (!s || !*s || !out) return false;
    char* end = nullptr;
    unsigned long v = strtoul(s, &end, 10);
    if (!end || *end != '\0') return false;
    *out = static_cast<uint32_t>(v);
    return true;
}

static bool setString(char* dst, size_t size, const char* value) {
    if (!dst || size == 0 || !value) return false;
    const size_t len = strlen(value);
    if (len >= size) return false;
    memcpy(dst, value, len + 1);
    return true;
}

static bool handleSet(const char* key, const char* value) {
    if (!key || !value) return false;

    RuntimeConfig& cfg = softConfigGet();

    if (strcmp(key, "ntrip_host") == 0) {
        return setString(cfg.ntrip_host, sizeof(cfg.ntrip_host), value);
    }
    if (strcmp(key, "ntrip_port") == 0) {
        uint32_t port = 0;
        if (!parseUint32(value, &port) || port == 0 || port > 65535) return false;
        cfg.ntrip_port = static_cast<uint16_t>(port);
        return true;
    }
    if (strcmp(key, "ntrip_mountpoint") == 0) {
        return setString(cfg.ntrip_mountpoint, sizeof(cfg.ntrip_mountpoint), value);
    }
    if (strcmp(key, "ntrip_user") == 0) {
        return setString(cfg.ntrip_user, sizeof(cfg.ntrip_user), value);
    }
    if (strcmp(key, "ntrip_password") == 0) {
        return setString(cfg.ntrip_password, sizeof(cfg.ntrip_password), value);
    }
    if (strcmp(key, "ntrip_reconnect_ms") == 0) {
        return parseUint32(value, &cfg.ntrip_reconnect_ms);
    }
    if (strcmp(key, "gnss_baud") == 0) {
        return parseUint32(value, &cfg.gnss_baud);
    }
    if (strcmp(key, "log_interval_ms") == 0) {
        return parseUint32(value, &cfg.log_interval_ms);
    }
    if (strcmp(key, "log_default_active") == 0) {
        bool b = false;
        if (!parseBool(value, &b)) return false;
        cfg.log_default_active = b;
        return true;
    }

    return false;
}

static ParsedLine splitLine(char* line) {
    ParsedLine out{nullptr, nullptr, nullptr};
    if (!line) return out;

    while (*line && isspace(static_cast<unsigned char>(*line))) line++;
    if (!*line) return out;

    out.cmd = strtok(line, " \t");
    out.arg1 = strtok(nullptr, " \t");
    out.arg2 = strtok(nullptr, "");

    if (out.arg2) {
        while (*out.arg2 && isspace(static_cast<unsigned char>(*out.arg2))) out.arg2++;
    }

    return out;
}

}  // namespace

void configModeRun() {
    hal_log("CONTROL DISABLED IN CONFIG MODE");
    hal_actuator_write(0);

    RuntimeConfig& cfg = softConfigGet();
    softConfigLoadDefaults(cfg);
    softConfigLoadOverrides(cfg);

    Serial.println();
    Serial.println("CONFIG MODE ACTIVE (reason: SAFETY_IN LOW)");
    Serial.println("CONTROL DISABLED IN CONFIG MODE");
    printHelp();

    char line[160];
    size_t line_len = 0;
    uint32_t last_rx_ms = millis();
    uint32_t last_heartbeat_ms = 0;

    printPrompt();

    for (;;) {
        const uint32_t now = millis();
        if (now - last_heartbeat_ms >= CONFIG_MODE_HEARTBEAT_MS) {
            last_heartbeat_ms = now;
            const uint32_t idle_ms = now - last_rx_ms;
            Serial.printf("[CONFIG MODE] alive idle_ms=%lu safety=%s\n",
                          static_cast<unsigned long>(idle_ms),
                          hal_safety_ok() ? "HIGH" : "LOW");
            Serial.flush();
        }

        if (now - last_rx_ms >= CONFIG_MODE_IDLE_TIMEOUT_MS) {
            last_rx_ms = now;
            Serial.println("[CONFIG MODE] timeout reminder: use help/show/set/save/reboot/exit");
            printPrompt();
        }

        while (Serial.available() > 0) {
            const int ch = Serial.read();
            if (ch < 0) break;

            last_rx_ms = millis();

            if (ch == '\r') continue;
            if (ch == '\n') {
                line[line_len] = '\0';
                ParsedLine parsed = splitLine(line);
                if (!parsed.cmd) {
                    printPrompt();
                    line_len = 0;
                    continue;
                }

                if (strcmp(parsed.cmd, "help") == 0) {
                    printHelp();
                } else if (strcmp(parsed.cmd, "show") == 0) {
                    printConfig(cfg);
                } else if (strcmp(parsed.cmd, "set") == 0) {
                    if (!parsed.arg1 || !parsed.arg2) {
                        Serial.println("ERR usage: set <key> <value>");
                    } else if (handleSet(parsed.arg1, parsed.arg2)) {
                        Serial.println("OK");
                    } else {
                        Serial.println("ERR invalid key/value");
                    }
                } else if (strcmp(parsed.cmd, "save") == 0) {
                    if (softConfigSaveToNvs(cfg)) {
                        Serial.println("OK saved to NVS");
                    } else {
                        Serial.println("ERR save failed");
                    }
                } else if (strcmp(parsed.cmd, "reboot") == 0) {
                    Serial.println("Rebooting...");
                    Serial.flush();
                    hal_delay_ms(100);
                    esp_restart();
                } else if (strcmp(parsed.cmd, "exit") == 0) {
                    Serial.println("Leaving config mode");
                    Serial.flush();
                    return;
                } else {
                    Serial.println("ERR unknown command");
                }

                line_len = 0;
                printPrompt();
                continue;
            }

            if (line_len + 1 < sizeof(line)) {
                line[line_len++] = static_cast<char>(ch);
            }
        }

        hal_delay_ms(20);
        hal_actuator_write(0);
    }
}
