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
#include "diag.h"
#include "setup_wizard.h"
#include "um980_uart_setup.h"

#include <Arduino.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

static Stream* s_cli_out = &Serial;

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
    s_cli_out->printf("AgSteer Build: %s %s\n", __DATE__, __TIME__);
}

void cliCmdUptime(int, char**) {
    const uint32_t sec = millis() / 1000UL;
    const uint32_t h = sec / 3600UL;
    const uint32_t m = (sec % 3600UL) / 60UL;
    const uint32_t s = sec % 60UL;
    s_cli_out->printf("Uptime: %luh %lum %lus\n",
                  static_cast<unsigned long>(h),
                  static_cast<unsigned long>(m),
                  static_cast<unsigned long>(s));
}

void cliCmdFree(int, char**) {
    s_cli_out->printf("Heap: %lu KB free (largest: %lu KB) PSRAM: %lu KB free\n",
                  static_cast<unsigned long>(ESP.getFreeHeap() / 1024UL),
                  static_cast<unsigned long>(ESP.getMaxAllocHeap() / 1024UL),
                  static_cast<unsigned long>(ESP.getFreePsram() / 1024UL));
}

void cliCmdTasks(int, char**) {
#if (configUSE_TRACE_FACILITY == 1)
    static char task_list[1024];
    task_list[0] = '\0';
    vTaskList(task_list);
    s_cli_out->println("Task         State Prio Stack Num");
    s_cli_out->print(task_list);
#else
    s_cli_out->printf("Tasks: %lu\n", static_cast<unsigned long>(uxTaskGetNumberOfTasks()));
    s_cli_out->println("Task listing unavailable (configUSE_TRACE_FACILITY=0)");
#endif
}

void cliCmdRestart(int, char**) {
    s_cli_out->println("Restarting...");
    s_cli_out->flush();
    ESP.restart();
}

void cliCmdSave(int, char**) {
    RuntimeConfig& cfg = softConfigGet();
    if (nvsConfigSave(cfg)) {
        s_cli_out->println("Config saved to NVS.");
        s_cli_out->println("WARNING: ntrip_password is stored in plaintext for now.");
    } else {
        s_cli_out->println("ERROR: failed to save config to NVS.");
    }
}

void cliCmdLoad(int, char**) {
    RuntimeConfig& cfg = softConfigGet();
    nvsConfigLoad(cfg);
    s_cli_out->println("Config loaded from NVS.");
}

void cliCmdFactory(int argc, char** argv) {
    if (argc < 2 || std::strcmp(argv[1], "confirm") != 0) {
        s_cli_out->println("WARNING: This will erase all saved configuration.");
        s_cli_out->println("Run: factory confirm");
        return;
    }

    nvsConfigFactoryReset();
    s_cli_out->println("NVS erased. Restarting with defaults...");
    s_cli_out->flush();
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
        s_cli_out->println("usage: ntrip <show|status|set|connect|disconnect>");
        return;
    }

    if (std::strcmp(argv[1], "show") == 0 || std::strcmp(argv[1], "status") == 0) {
        const NtripState state = ntripGetState();
        s_cli_out->println("NTRIP:");
        s_cli_out->printf("  Host:       %s\n", cfg.ntrip_host);
        s_cli_out->printf("  Port:       %u\n", static_cast<unsigned>(cfg.ntrip_port));
        s_cli_out->printf("  Mountpoint: %s\n", cfg.ntrip_mountpoint);
        s_cli_out->printf("  User:       %s\n", cfg.ntrip_user);
        s_cli_out->printf("  Password:   %s\n", cfg.ntrip_password[0] ? "********" : "(empty)");
        s_cli_out->printf("  State:      %s\n", ntripConnStateToStr(state.conn_state));
        s_cli_out->printf("  Bytes RX:   %lu\n", static_cast<unsigned long>(state.rx_bytes));
        s_cli_out->printf("  Fwd bytes:  %lu\n", static_cast<unsigned long>(state.forwarded_bytes));
        return;
    }

    if (std::strcmp(argv[1], "set") == 0) {
        if (argc < 4) {
            s_cli_out->println("usage: ntrip set <host|port|mount|user|pass> <value>");
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
            s_cli_out->println("usage: ntrip set <host|port|mount|user|pass> <value>");
            return;
        }

        ntripSetConfig(cfg.ntrip_host, cfg.ntrip_port, cfg.ntrip_mountpoint, cfg.ntrip_user, cfg.ntrip_password);
        s_cli_out->println("NTRIP config updated (runtime).");
        return;
    }

    if (std::strcmp(argv[1], "connect") == 0) {
        ntripSetConfig(cfg.ntrip_host, cfg.ntrip_port, cfg.ntrip_mountpoint, cfg.ntrip_user, cfg.ntrip_password);
        s_cli_out->println("NTRIP connect requested (state machine will connect).");
        return;
    }

    if (std::strcmp(argv[1], "disconnect") == 0) {
        // Force IDLE by clearing host/mount at runtime.
        cfg.ntrip_host[0] = '\0';
        cfg.ntrip_mountpoint[0] = '\0';
        ntripSetConfig(cfg.ntrip_host, cfg.ntrip_port, cfg.ntrip_mountpoint, cfg.ntrip_user, cfg.ntrip_password);
        s_cli_out->println("NTRIP disconnected (runtime config cleared host/mount).");
        return;
    }

    s_cli_out->println("usage: ntrip <show|status|set|connect|disconnect>");
#else
    (void)argc;
    (void)argv;
    s_cli_out->println("NTRIP not compiled in this profile.");
#endif
}

void printIpU32(uint32_t ip) {
    s_cli_out->printf("%u.%u.%u.%u",
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
        s_cli_out->println("usage: pid <show|set>");
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
        s_cli_out->println("PID:");
        s_cli_out->printf("  Kp: %.3f\n", kp);
        s_cli_out->printf("  Ki: %.3f\n", ki);
        s_cli_out->printf("  Kd: %.3f\n", kd);
        s_cli_out->printf("  MinPWM: %u\n", static_cast<unsigned>(min_pwm));
        s_cli_out->printf("  HighPWM: %u\n", static_cast<unsigned>(high_pwm));
        s_cli_out->println("  Note: PGN 252 can overwrite runtime values.");
        return;
    }

    if (std::strcmp(argv[1], "set") == 0) {
        if (argc < 4) {
            s_cli_out->println("usage: pid set <kp|ki|kd|minpwm|highpwm> <value>");
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
            s_cli_out->println("usage: pid set <kp|ki|kd|minpwm|highpwm> <value>");
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
        s_cli_out->println("PID updated.");
        return;
    }

    s_cli_out->println("usage: pid <show|set>");
}

void cliCmdNet(int argc, char** argv) {
    RuntimeConfig& cfg = softConfigGet();
    if (argc < 2) {
        s_cli_out->println("usage: net <show|mode|ip|gw|mask|restart>");
        return;
    }

    if (std::strcmp(argv[1], "show") == 0) {
        s_cli_out->println("Network:");
        s_cli_out->printf("  Mode: %s\n", cfg.net_mode == 0 ? "DHCP" : "STATIC");
        s_cli_out->print("  IP: "); printIpU32(hal_net_get_ip()); s_cli_out->println();
        s_cli_out->print("  Mask: "); printIpU32(hal_net_get_subnet()); s_cli_out->println();
        s_cli_out->print("  Gateway: "); printIpU32(hal_net_get_gateway()); s_cli_out->println();
        s_cli_out->printf("  Link: %s\n", hal_net_link_up() ? "UP" : "DOWN");
        return;
    }

    if (std::strcmp(argv[1], "mode") == 0) {
        if (argc < 3) {
            s_cli_out->println("usage: net mode <dhcp|static>");
            return;
        }
        if (std::strcmp(argv[2], "dhcp") == 0) cfg.net_mode = 0;
        else if (std::strcmp(argv[2], "static") == 0) cfg.net_mode = 1;
        else {
            s_cli_out->println("usage: net mode <dhcp|static>");
            return;
        }
        s_cli_out->println("Network mode updated (apply with: net restart).");
        return;
    }

    if (std::strcmp(argv[1], "ip") == 0 || std::strcmp(argv[1], "gw") == 0 || std::strcmp(argv[1], "mask") == 0) {
        if (argc < 3) {
            s_cli_out->println("usage: net <ip|gw|mask> <a.b.c.d>");
            return;
        }
        uint32_t ip = 0;
        if (!parseIp4(argv[2], &ip)) {
            s_cli_out->println("ERROR: invalid IPv4 format.");
            return;
        }
        if (std::strcmp(argv[1], "ip") == 0) cfg.net_ip = ip;
        else if (std::strcmp(argv[1], "gw") == 0) cfg.net_gateway = ip;
        else cfg.net_subnet = ip;
        s_cli_out->println("Network parameter updated (apply with: net restart).");
        return;
    }

    if (std::strcmp(argv[1], "restart") == 0) {
        if (cfg.net_mode == 1) {
            hal_net_set_static_config(cfg.net_ip, cfg.net_gateway, cfg.net_subnet);
        }
        s_cli_out->print("Restarting network");
        for (int i = 0; i < 3; ++i) {
            s_cli_out->print(".");
            delay(100);
        }
        s_cli_out->println();
        const bool ok = hal_net_restart();
        s_cli_out->printf("Network restart %s\n", ok ? "OK" : "DONE (link pending)");
        return;
    }

    s_cli_out->println("usage: net <show|mode|ip|gw|mask|restart>");
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
        s_cli_out->println("usage: module <list|enable|disable|pins> [name]");
        return;
    }

    if (std::strcmp(argv[1], "list") == 0) {
        s_cli_out->println("Module Status:");
        for (int i = 0; i < MOD_COUNT; ++i) {
            const auto* info = moduleGetInfo(static_cast<FirmwareFeatureId>(i));
            if (!info) continue;
            s_cli_out->printf("  %-6s (%d) = %-11s pins=%u deps=%s\n",
                          info->name ? info->name : "?",
                          i,
                          modStateToStr(moduleGetState(static_cast<FirmwareFeatureId>(i))),
                          static_cast<unsigned>(info->pin_count),
                          info->deps ? "yes" : "none");
        }
        return;
    }

    if (argc < 3) {
        s_cli_out->println("usage: module <enable|disable|pins> <name>");
        return;
    }

    FirmwareFeatureId id = MOD_COUNT;
    if (!parseModuleName(argv[2], &id)) {
        s_cli_out->println("ERROR: unknown module name.");
        return;
    }

    if (std::strcmp(argv[1], "enable") == 0) {
        const bool ok = moduleActivate(id);
        s_cli_out->printf("module %s -> %s\n", argv[2], ok ? "ON" : "ERROR");
        return;
    }

    if (std::strcmp(argv[1], "disable") == 0) {
        if (id == MOD_ETH) {
            s_cli_out->println("ERROR: ETH is mandatory and cannot be disabled.");
            return;
        }
        const bool ok = moduleDeactivate(id);
        s_cli_out->printf("module %s -> %s\n", argv[2], ok ? "OFF" : "ERROR");
        return;
    }

    if (std::strcmp(argv[1], "pins") == 0) {
        const auto* info = moduleGetInfo(id);
        if (!info) {
            s_cli_out->println("ERROR: module not found.");
            return;
        }
        s_cli_out->printf("%s pins:", info->name ? info->name : argv[2]);
        if (!info->pins || info->pin_count == 0) {
            s_cli_out->println(" (none)");
            return;
        }
        for (uint8_t i = 0; i < info->pin_count; ++i) {
            s_cli_out->printf(" %d", static_cast<int>(info->pins[i]));
        }
        s_cli_out->println();
        return;
    }

    s_cli_out->println("usage: module <list|enable|disable|pins> [name]");
}

void cliCmdActuator(int argc, char** argv) {
    if (argc < 2) {
        s_cli_out->println("usage: actuator <status|test>");
        return;
    }

    if (std::strcmp(argv[1], "status") == 0) {
        s_cli_out->printf("Actuator manual mode: %s\n", controlManualActuatorMode() ? "ON" : "OFF");
        s_cli_out->printf("Safety: %s\n", hal_safety_ok() ? "OK" : "KICK");
        return;
    }

    if (std::strcmp(argv[1], "test") == 0) {
        if (argc < 3) {
            s_cli_out->println("usage: actuator test <pwm|stop> [value]");
            return;
        }
        if (std::strcmp(argv[2], "stop") == 0) {
            controlSetManualActuatorMode(false);
            hal_actuator_write(0);
            s_cli_out->println("Actuator stopped. Manual mode OFF.");
            return;
        }
        if (std::strcmp(argv[2], "pwm") == 0 && argc >= 4) {
            int value = std::atoi(argv[3]);
            if (value < 0) value = 0;
            if (value > 65535) value = 65535;
            controlSetManualActuatorMode(true);
            hal_actuator_write(static_cast<uint16_t>(value));
            s_cli_out->println("WARNING: manual actuator command active (PID paused).");
            s_cli_out->printf("Actuator PWM command: %d\n", value);
            return;
        }
        s_cli_out->println("usage: actuator test <pwm|stop> [value]");
        return;
    }

    s_cli_out->println("usage: actuator <status|test>");
}

void cliCmdDiag(int argc, char** argv) {
    if (argc < 2) {
        s_cli_out->println("usage: diag <hw|mem|net>");
        return;
    }

    if (std::strcmp(argv[1], "hw") == 0) {
        diagPrintHw();
        return;
    }
    if (std::strcmp(argv[1], "mem") == 0) {
        diagPrintMem();
        return;
    }
    if (std::strcmp(argv[1], "net") == 0) {
        diagPrintNet();
        return;
    }

    s_cli_out->println("usage: diag <hw|mem|net>");
}

static bool parseOnOff(const char* text, bool* out_value) {
    if (!text || !out_value) return false;
    if (std::strcmp(text, "on") == 0 || std::strcmp(text, "1") == 0) {
        *out_value = true;
        return true;
    }
    if (std::strcmp(text, "off") == 0 || std::strcmp(text, "0") == 0) {
        *out_value = false;
        return true;
    }
    return false;
}

static bool parseUartPort(const char* text, uint8_t* out_port) {
    if (!text || !out_port) return false;
    if (std::strcmp(text, "a") == 0 || std::strcmp(text, "A") == 0) {
        *out_port = 0;
        return true;
    }
    if (std::strcmp(text, "b") == 0 || std::strcmp(text, "B") == 0) {
        *out_port = 1;
        return true;
    }
    return false;
}

void cliCmdUart(int argc, char** argv) {
    if (argc < 2) {
        s_cli_out->println("usage: uart <show|apply|set|console>");
        return;
    }

    if (std::strcmp(argv[1], "show") == 0) {
        const Um980UartSetup setup = um980SetupGet();
        s_cli_out->println("UM980 UART setup:");
        s_cli_out->printf("  A: baud=%lu swap=%s console=%s\n",
                      static_cast<unsigned long>(setup.baud_a),
                      setup.swap_a ? "ON" : "OFF",
                      setup.console_a ? "ON" : "OFF");
        s_cli_out->printf("  B: baud=%lu swap=%s console=%s\n",
                      static_cast<unsigned long>(setup.baud_b),
                      setup.swap_b ? "ON" : "OFF",
                      setup.console_b ? "ON" : "OFF");
        return;
    }

    if (std::strcmp(argv[1], "apply") == 0) {
        if (argc < 3 || std::strcmp(argv[2], "all") == 0) {
            const bool ok = um980SetupApply();
            s_cli_out->printf("UM980 UART apply all -> %s\n", ok ? "OK" : "ERROR");
            return;
        }
        uint8_t port = 0;
        if (!parseUartPort(argv[2], &port)) {
            s_cli_out->println("usage: uart apply <a|b|all>");
            return;
        }
        const bool ok = um980SetupApplyPort(port);
        s_cli_out->printf("UM980 UART apply %c -> %s\n", port == 0 ? 'A' : 'B', ok ? "OK" : "ERROR");
        return;
    }

    if (std::strcmp(argv[1], "set") == 0) {
        if (argc < 5) {
            s_cli_out->println("usage: uart set <a|b> <baud|swap> <value>");
            return;
        }
        uint8_t port = 0;
        if (!parseUartPort(argv[2], &port)) {
            s_cli_out->println("usage: uart set <a|b> <baud|swap> <value>");
            return;
        }
        if (std::strcmp(argv[3], "baud") == 0) {
            const uint32_t baud = static_cast<uint32_t>(std::strtoul(argv[4], nullptr, 10));
            if (baud == 0) {
                s_cli_out->println("ERROR: invalid baud value.");
                return;
            }
            um980SetupSetBaud(port, baud);
            s_cli_out->printf("UM980 UART %c baud set to %lu (pending apply).\n",
                          port == 0 ? 'A' : 'B',
                          static_cast<unsigned long>(baud));
            return;
        }
        if (std::strcmp(argv[3], "swap") == 0) {
            bool enabled = false;
            if (!parseOnOff(argv[4], &enabled)) {
                s_cli_out->println("usage: uart set <a|b> swap <on|off>");
                return;
            }
            um980SetupSetSwap(port, enabled);
            s_cli_out->printf("UM980 UART %c swap set to %s (pending apply).\n",
                          port == 0 ? 'A' : 'B',
                          enabled ? "ON" : "OFF");
            return;
        }
        s_cli_out->println("usage: uart set <a|b> <baud|swap> <value>");
        return;
    }

    if (std::strcmp(argv[1], "console") == 0) {
        if (argc < 4) {
            s_cli_out->println("usage: uart console <a|b> <on|off>");
            return;
        }
        uint8_t port = 0;
        if (!parseUartPort(argv[2], &port)) {
            s_cli_out->println("usage: uart console <a|b> <on|off>");
            return;
        }
        bool enabled = false;
        if (!parseOnOff(argv[3], &enabled)) {
            s_cli_out->println("usage: uart console <a|b> <on|off>");
            return;
        }
        um980SetupSetConsole(port, enabled);
        s_cli_out->printf("UM980 UART %c console -> %s\n",
                      port == 0 ? 'A' : 'B',
                      enabled ? "ON" : "OFF");
        return;
    }

    s_cli_out->println("usage: uart <show|apply|set|console>");
}

void cliCmdSetup(int argc, char** argv) {
    (void)argc;
    (void)argv;
    setupWizardRequestStart();
    s_cli_out->println("Setup wizard requested. It will start in loop context.");
}

void cliCmdUnknown(const char* cmd) {
    s_cli_out->printf("Unknown command: %s\n", cmd ? cmd : "");
    s_cli_out->println("Type 'help' for available commands.");
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

void cliSetOutput(Stream* out) {
    s_cli_out = out ? out : &Serial;
}

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
    (void)cliRegisterCommand("actuator", &cliCmdActuator, "Actuator manual test mode");
    (void)cliRegisterCommand("diag", &cliCmdDiag, "Diagnostics (hw/mem/net)");
    (void)cliRegisterCommand("setup", &cliCmdSetup, "Start setup wizard");
    (void)cliRegisterCommand("uart", &cliCmdUart, "UM980 UART setup + live console");
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
    s_cli_out->println("Available commands:");
    for (size_t i = 0; i < s_cli_cmd_count; ++i) {
        s_cli_out->printf("  %-10s %s\n", s_cli_cmd_table[i].cmd, s_cli_cmd_table[i].help_short);
    }
    s_cli_out->println("  log ...    Runtime log controls");
    s_cli_out->println("  filter ... Runtime log file:line filter");
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
