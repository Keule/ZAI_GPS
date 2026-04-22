/**
 * @file cli.cpp
 * @brief Serial Command Line Interface — Phase 0 (S0-01).
 *
 * Implements lightweight command parsing and dispatch for serial commands.
 */

#include "cli.h"

#include "log_ext.h"
#include "nvs_config.h"
#include "runtime_config.h"
#include "ntrip.h"
#include "control.h"
#include "global_state.h"
#include "hal/hal.h"
#include "modules.h"

#include <Arduino.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

constexpr size_t CLI_MAX_COMMANDS = 32;
constexpr size_t CLI_MAX_ARGS = 8;

using CliHandler = void (*)(int argc, char* argv[]);

struct CliCommand {
    const char* cmd = nullptr;
    CliHandler handler = nullptr;
    const char* help_short = nullptr;
};

CliCommand s_cli_cmd_table[CLI_MAX_COMMANDS] = {};
size_t s_cli_cmd_count = 0;

void cliCmdHelp(int, char**) {
    cliPrintHelp();
}

void cliCmdVersion(int, char**) {
    Serial.printf("AgSteer Build: %s %s\n", __DATE__, __TIME__);
}

void cliCmdUptime(int, char**) {
    const uint32_t sec = millis() / 1000UL;
    const uint32_t h = sec / 3600UL;
    const uint32_t m = (sec % 3600UL) / 60UL;
    const uint32_t s = sec % 60UL;
    Serial.printf("Uptime: %luh %lum %lus\n",
                  static_cast<unsigned long>(h),
                  static_cast<unsigned long>(m),
                  static_cast<unsigned long>(s));
}

void cliCmdFree(int, char**) {
    Serial.printf("Heap: %lu KB free (largest: %lu KB) PSRAM: %lu KB free\n",
                  static_cast<unsigned long>(ESP.getFreeHeap() / 1024UL),
                  static_cast<unsigned long>(ESP.getMaxAllocHeap() / 1024UL),
                  static_cast<unsigned long>(ESP.getFreePsram() / 1024UL));
}

void cliCmdTasks(int, char**) {
#if (configUSE_TRACE_FACILITY == 1)
    static char task_list[1024];
    task_list[0] = '\0';
    vTaskList(task_list);
    Serial.println("Task         State Prio Stack Num");
    Serial.print(task_list);
#else
    Serial.printf("Tasks: %lu\n", static_cast<unsigned long>(uxTaskGetNumberOfTasks()));
    Serial.println("Task listing unavailable (configUSE_TRACE_FACILITY=0)");
#endif
}

void cliCmdRestart(int, char**) {
    Serial.println("Restarting...");
    Serial.flush();
    ESP.restart();
}

void cliCmdSave(int, char**) {
    RuntimeConfig& cfg = softConfigGet();
    if (nvsConfigSave(cfg)) {
        Serial.println("Config saved to NVS.");
        Serial.println("WARNING: ntrip_password is stored in plaintext for now.");
    } else {
        Serial.println("ERROR: failed to save config to NVS.");
    }
}

void cliCmdLoad(int, char**) {
    RuntimeConfig& cfg = softConfigGet();
    nvsConfigLoad(cfg);
    Serial.println("Config loaded from NVS.");
}

void cliCmdFactory(int argc, char** argv) {
    if (argc < 2 || std::strcmp(argv[1], "confirm") != 0) {
        Serial.println("WARNING: This will erase all saved configuration.");
        Serial.println("Run: factory confirm");
        return;
    }

    nvsConfigFactoryReset();
    Serial.println("NVS erased. Restarting with defaults...");
    Serial.flush();
    ESP.restart();
}

#if FEAT_ENABLED(FEAT_COMPILED_NTRIP)
const char* ntripConnStateToStr(NtripConnState state) {
    switch (state) {
        case NtripConnState::IDLE: return "IDLE";
        case NtripConnState::CONNECTING: return "CONNECTING";
        case NtripConnState::AUTHENTICATING: return "AUTHENTICATING";
        case NtripConnState::CONNECTED: return "CONNECTED";
        case NtripConnState::DISCONNECTED: return "DISCONNECTED";
        case NtripConnState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}
#endif

void cliCmdNtrip(int argc, char** argv) {
#if FEAT_ENABLED(FEAT_COMPILED_NTRIP)
    RuntimeConfig& cfg = softConfigGet();
    if (argc < 2) {
        Serial.println("usage: ntrip <show|status|set|connect|disconnect>");
        return;
    }

    if (std::strcmp(argv[1], "show") == 0 || std::strcmp(argv[1], "status") == 0) {
        const NtripState state = ntripGetState();
        Serial.println("NTRIP:");
        Serial.printf("  Host:       %s\n", cfg.ntrip_host);
        Serial.printf("  Port:       %u\n", static_cast<unsigned>(cfg.ntrip_port));
        Serial.printf("  Mountpoint: %s\n", cfg.ntrip_mountpoint);
        Serial.printf("  User:       %s\n", cfg.ntrip_user);
        Serial.printf("  Password:   %s\n", cfg.ntrip_password[0] ? "********" : "(empty)");
        Serial.printf("  State:      %s\n", ntripConnStateToStr(state.conn_state));
        Serial.printf("  Bytes RX:   %lu\n", static_cast<unsigned long>(state.rx_bytes));
        Serial.printf("  Fwd bytes:  %lu\n", static_cast<unsigned long>(state.forwarded_bytes));
        return;
    }

    if (std::strcmp(argv[1], "set") == 0) {
        if (argc < 4) {
            Serial.println("usage: ntrip set <host|port|mount|user|pass> <value>");
            return;
        }
        if (std::strcmp(argv[2], "host") == 0) {
            std::strncpy(cfg.ntrip_host, argv[3], sizeof(cfg.ntrip_host) - 1);
            cfg.ntrip_host[sizeof(cfg.ntrip_host) - 1] = '\0';
        } else if (std::strcmp(argv[2], "port") == 0) {
            cfg.ntrip_port = static_cast<uint16_t>(std::atoi(argv[3]));
        } else if (std::strcmp(argv[2], "mount") == 0 || std::strcmp(argv[2], "mountpoint") == 0) {
            std::strncpy(cfg.ntrip_mountpoint, argv[3], sizeof(cfg.ntrip_mountpoint) - 1);
            cfg.ntrip_mountpoint[sizeof(cfg.ntrip_mountpoint) - 1] = '\0';
        } else if (std::strcmp(argv[2], "user") == 0) {
            std::strncpy(cfg.ntrip_user, argv[3], sizeof(cfg.ntrip_user) - 1);
            cfg.ntrip_user[sizeof(cfg.ntrip_user) - 1] = '\0';
        } else if (std::strcmp(argv[2], "pass") == 0 || std::strcmp(argv[2], "password") == 0) {
            std::strncpy(cfg.ntrip_password, argv[3], sizeof(cfg.ntrip_password) - 1);
            cfg.ntrip_password[sizeof(cfg.ntrip_password) - 1] = '\0';
        } else {
            Serial.println("usage: ntrip set <host|port|mount|user|pass> <value>");
            return;
        }

        ntripSetConfig(cfg.ntrip_host, cfg.ntrip_port, cfg.ntrip_mountpoint, cfg.ntrip_user, cfg.ntrip_password);
        Serial.println("NTRIP config updated (runtime).");
        return;
    }

    if (std::strcmp(argv[1], "connect") == 0) {
        ntripSetConfig(cfg.ntrip_host, cfg.ntrip_port, cfg.ntrip_mountpoint, cfg.ntrip_user, cfg.ntrip_password);
        Serial.println("NTRIP connect requested (state machine will connect).");
        return;
    }

    if (std::strcmp(argv[1], "disconnect") == 0) {
        // Force IDLE by clearing host/mount at runtime.
        cfg.ntrip_host[0] = '\0';
        cfg.ntrip_mountpoint[0] = '\0';
        ntripSetConfig(cfg.ntrip_host, cfg.ntrip_port, cfg.ntrip_mountpoint, cfg.ntrip_user, cfg.ntrip_password);
        Serial.println("NTRIP disconnected (runtime config cleared host/mount).");
        return;
    }

    Serial.println("usage: ntrip <show|status|set|connect|disconnect>");
#else
    (void)argc;
    (void)argv;
    Serial.println("NTRIP not compiled in this profile.");
#endif
}

void printIpU32(uint32_t ip) {
    Serial.printf("%u.%u.%u.%u",
                  static_cast<unsigned>((ip >> 24) & 0xFF),
                  static_cast<unsigned>((ip >> 16) & 0xFF),
                  static_cast<unsigned>((ip >> 8) & 0xFF),
                  static_cast<unsigned>(ip & 0xFF));
}

bool parseIp4(const char* text, uint32_t* out_ip) {
    if (!text || !out_ip) return false;
    unsigned a = 0, b = 0, c = 0, d = 0;
    if (std::sscanf(text, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return false;
    if (a > 255 || b > 255 || c > 255 || d > 255) return false;
    *out_ip = (a << 24) | (b << 16) | (c << 8) | d;
    return true;
}

void cliCmdPid(int argc, char** argv) {
    RuntimeConfig& cfg = softConfigGet();
    if (argc < 2) {
        Serial.println("usage: pid <show|set>");
        return;
    }

    if (std::strcmp(argv[1], "show") == 0) {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f;
        uint8_t min_pwm = 0;
        uint8_t high_pwm = 0;
        {
            StateLock lock;
            min_pwm = g_nav.pid.settings_min_pwm;
            high_pwm = g_nav.pid.settings_high_pwm;
        }
        controlGetPidGains(&kp, &ki, &kd);
        Serial.println("PID:");
        Serial.printf("  Kp: %.3f\n", kp);
        Serial.printf("  Ki: %.3f\n", ki);
        Serial.printf("  Kd: %.3f\n", kd);
        Serial.printf("  MinPWM: %u\n", static_cast<unsigned>(min_pwm));
        Serial.printf("  HighPWM: %u\n", static_cast<unsigned>(high_pwm));
        Serial.println("  Note: PGN 252 can overwrite runtime values.");
        return;
    }

    if (std::strcmp(argv[1], "set") == 0) {
        if (argc < 4) {
            Serial.println("usage: pid set <kp|ki|kd|minpwm|highpwm> <value>");
            return;
        }

        if (std::strcmp(argv[2], "kp") == 0) {
            cfg.pid_kp = static_cast<float>(std::atof(argv[3]));
        } else if (std::strcmp(argv[2], "ki") == 0) {
            cfg.pid_ki = static_cast<float>(std::atof(argv[3]));
        } else if (std::strcmp(argv[2], "kd") == 0) {
            cfg.pid_kd = static_cast<float>(std::atof(argv[3]));
        } else if (std::strcmp(argv[2], "minpwm") == 0) {
            StateLock lock;
            g_nav.pid.settings_min_pwm = static_cast<uint8_t>(std::atoi(argv[3]));
        } else if (std::strcmp(argv[2], "highpwm") == 0) {
            StateLock lock;
            g_nav.pid.settings_high_pwm = static_cast<uint8_t>(std::atoi(argv[3]));
        } else {
            Serial.println("usage: pid set <kp|ki|kd|minpwm|highpwm> <value>");
            return;
        }

        uint8_t min_pwm = 0;
        uint8_t high_pwm = 0;
        {
            StateLock lock;
            min_pwm = g_nav.pid.settings_min_pwm;
            high_pwm = g_nav.pid.settings_high_pwm;
        }
        controlSetPidGains(cfg.pid_kp, cfg.pid_ki, cfg.pid_kd);
        controlSetPidOutputLimits(static_cast<float>(min_pwm),
                                  static_cast<float>(high_pwm));
        Serial.println("PID updated.");
        return;
    }

    Serial.println("usage: pid <show|set>");
}

void cliCmdNet(int argc, char** argv) {
    RuntimeConfig& cfg = softConfigGet();
    if (argc < 2) {
        Serial.println("usage: net <show|mode|ip|gw|mask|restart>");
        return;
    }

    if (std::strcmp(argv[1], "show") == 0) {
        Serial.println("Network:");
        Serial.printf("  Mode: %s\n", cfg.net_mode == 0 ? "DHCP" : "STATIC");
        Serial.print("  IP: "); printIpU32(hal_net_get_ip()); Serial.println();
        Serial.print("  Mask: "); printIpU32(hal_net_get_subnet()); Serial.println();
        Serial.print("  Gateway: "); printIpU32(hal_net_get_gateway()); Serial.println();
        Serial.printf("  Link: %s\n", hal_net_link_up() ? "UP" : "DOWN");
        return;
    }

    if (std::strcmp(argv[1], "mode") == 0) {
        if (argc < 3) {
            Serial.println("usage: net mode <dhcp|static>");
            return;
        }
        if (std::strcmp(argv[2], "dhcp") == 0) cfg.net_mode = 0;
        else if (std::strcmp(argv[2], "static") == 0) cfg.net_mode = 1;
        else {
            Serial.println("usage: net mode <dhcp|static>");
            return;
        }
        Serial.println("Network mode updated (apply with: net restart).");
        return;
    }

    if (std::strcmp(argv[1], "ip") == 0 || std::strcmp(argv[1], "gw") == 0 || std::strcmp(argv[1], "mask") == 0) {
        if (argc < 3) {
            Serial.println("usage: net <ip|gw|mask> <a.b.c.d>");
            return;
        }
        uint32_t ip = 0;
        if (!parseIp4(argv[2], &ip)) {
            Serial.println("ERROR: invalid IPv4 format.");
            return;
        }
        if (std::strcmp(argv[1], "ip") == 0) cfg.net_ip = ip;
        else if (std::strcmp(argv[1], "gw") == 0) cfg.net_gateway = ip;
        else cfg.net_subnet = ip;
        Serial.println("Network parameter updated (apply with: net restart).");
        return;
    }

    if (std::strcmp(argv[1], "restart") == 0) {
        if (cfg.net_mode == 1) {
            hal_net_set_static_config(cfg.net_ip, cfg.net_gateway, cfg.net_subnet);
        }
        Serial.print("Restarting network");
        for (int i = 0; i < 3; ++i) {
            Serial.print(".");
            delay(100);
        }
        Serial.println();
        const bool ok = hal_net_restart();
        Serial.printf("Network restart %s\n", ok ? "OK" : "DONE (link pending)");
        return;
    }

    Serial.println("usage: net <show|mode|ip|gw|mask|restart>");
}

const char* modStateToStr(ModState s) {
    switch (s) {
        case MOD_UNAVAILABLE: return "UNAVAILABLE";
        case MOD_OFF: return "OFF";
        case MOD_ON: return "ON";
        default: return "?";
    }
}

bool parseModuleName(const char* name, FirmwareFeatureId* out_id) {
    if (!name || !out_id) return false;
    struct Entry { const char* name; FirmwareFeatureId id; };
    static const Entry kEntries[] = {
        {"imu", MOD_IMU}, {"ads", MOD_ADS}, {"act", MOD_ACT}, {"eth", MOD_ETH},
        {"gnss", MOD_GNSS}, {"ntrip", MOD_NTRIP}, {"safety", MOD_SAFETY},
        {"logsw", MOD_LOGSW}, {"sd", MOD_SD},
    };
    for (const auto& e : kEntries) {
        if (std::strcmp(name, e.name) == 0) {
            *out_id = e.id;
            return true;
        }
    }
    return false;
}

void cliCmdModule(int argc, char** argv) {
    if (argc < 2) {
        Serial.println("usage: module <list|enable|disable|pins> [name]");
        return;
    }

    if (std::strcmp(argv[1], "list") == 0) {
        Serial.println("Module Status:");
        for (int i = 0; i < MOD_COUNT; ++i) {
            const auto* info = moduleGetInfo(static_cast<FirmwareFeatureId>(i));
            if (!info) continue;
            Serial.printf("  %-6s (%d) = %-11s pins=%u deps=%s\n",
                          info->name ? info->name : "?",
                          i,
                          modStateToStr(moduleGetState(static_cast<FirmwareFeatureId>(i))),
                          static_cast<unsigned>(info->pin_count),
                          info->deps ? "yes" : "none");
        }
        return;
    }

    if (argc < 3) {
        Serial.println("usage: module <enable|disable|pins> <name>");
        return;
    }

    FirmwareFeatureId id = MOD_COUNT;
    if (!parseModuleName(argv[2], &id)) {
        Serial.println("ERROR: unknown module name.");
        return;
    }

    if (std::strcmp(argv[1], "enable") == 0) {
        const bool ok = moduleActivate(id);
        Serial.printf("module %s -> %s\n", argv[2], ok ? "ON" : "ERROR");
        return;
    }

    if (std::strcmp(argv[1], "disable") == 0) {
        if (id == MOD_ETH) {
            Serial.println("ERROR: ETH is mandatory and cannot be disabled.");
            return;
        }
        const bool ok = moduleDeactivate(id);
        Serial.printf("module %s -> %s\n", argv[2], ok ? "OFF" : "ERROR");
        return;
    }

    if (std::strcmp(argv[1], "pins") == 0) {
        const auto* info = moduleGetInfo(id);
        if (!info) {
            Serial.println("ERROR: module not found.");
            return;
        }
        Serial.printf("%s pins:", info->name ? info->name : argv[2]);
        if (!info->pins || info->pin_count == 0) {
            Serial.println(" (none)");
            return;
        }
        for (uint8_t i = 0; i < info->pin_count; ++i) {
            Serial.printf(" %d", static_cast<int>(info->pins[i]));
        }
        Serial.println();
        return;
    }

    Serial.println("usage: module <list|enable|disable|pins> [name]");
}

void cliCmdUnknown(const char* cmd) {
    Serial.printf("Unknown command: %s\n", cmd ? cmd : "");
    Serial.println("Type 'help' for available commands.");
}

void cliDispatch(int argc, char* argv[]) {
    if (argc <= 0 || argv == nullptr || argv[0] == nullptr) {
        return;
    }

    for (size_t i = 0; i < s_cli_cmd_count; ++i) {
        if (std::strcmp(argv[0], s_cli_cmd_table[i].cmd) == 0) {
            s_cli_cmd_table[i].handler(argc, argv);
            return;
        }
    }

    cliCmdUnknown(argv[0]);
}

}  // namespace

void cliInit(void) {
    s_cli_cmd_count = 0;
    (void)cliRegisterCommand("help", &cliCmdHelp, "List all commands");
    (void)cliRegisterCommand("version", &cliCmdVersion, "Show firmware build version");
    (void)cliRegisterCommand("uptime", &cliCmdUptime, "Show uptime (h m s)");
    (void)cliRegisterCommand("free", &cliCmdFree, "Show heap/PSRAM memory");
    (void)cliRegisterCommand("tasks", &cliCmdTasks, "Show FreeRTOS task list");
    (void)cliRegisterCommand("restart", &cliCmdRestart, "Restart ESP32");
    (void)cliRegisterCommand("save", &cliCmdSave, "Save runtime config to NVS");
    (void)cliRegisterCommand("load", &cliCmdLoad, "Load runtime config from NVS");
    (void)cliRegisterCommand("factory", &cliCmdFactory, "Factory reset (use: factory confirm)");
    (void)cliRegisterCommand("ntrip", &cliCmdNtrip, "NTRIP runtime config and status");
    (void)cliRegisterCommand("pid", &cliCmdPid, "PID tuning and status");
    (void)cliRegisterCommand("net", &cliCmdNet, "Network runtime config");
    (void)cliRegisterCommand("module", &cliCmdModule, "Module runtime control");
}

bool cliRegisterCommand(const char* cmd,
                        void (*handler)(int argc, char* argv[]),
                        const char* help_short) {
    if (!cmd || !*cmd || !handler || !help_short) {
        return false;
    }

    for (size_t i = 0; i < s_cli_cmd_count; ++i) {
        if (std::strcmp(s_cli_cmd_table[i].cmd, cmd) == 0) {
            return false;
        }
    }

    if (s_cli_cmd_count >= CLI_MAX_COMMANDS) {
        return false;
    }

    s_cli_cmd_table[s_cli_cmd_count++] = {cmd, handler, help_short};
    return true;
}

void cliPrintHelp(void) {
    Serial.println("Available commands:");
    for (size_t i = 0; i < s_cli_cmd_count; ++i) {
        Serial.printf("  %-10s %s\n", s_cli_cmd_table[i].cmd, s_cli_cmd_table[i].help_short);
    }
    Serial.println("  log ...    Runtime log controls");
    Serial.println("  filter ... Runtime log file:line filter");
}

void cliProcessLine(const char* line) {
    if (!line) {
        return;
    }

    while (*line == ' ' || *line == '\t') {
        ++line;
    }
    if (*line == '\0') {
        return;
    }

    if (std::strncmp(line, "log", 3) == 0 || std::strncmp(line, "filter", 6) == 0) {
        logProcessSerialCmd(line);
        return;
    }

    char buffer[128] = {0};
    std::strncpy(buffer, line, sizeof(buffer) - 1);

    char* argv[CLI_MAX_ARGS] = {};
    int argc = 0;

    char* token = std::strtok(buffer, " \t");
    while (token && argc < static_cast<int>(CLI_MAX_ARGS)) {
        argv[argc++] = token;
        token = std::strtok(nullptr, " \t");
    }

    cliDispatch(argc, argv);
}
