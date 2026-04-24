/**
 * @file actuator.cpp
 * @brief Steering actuator implementation.
 *
 * Uses HAL SPI path and applies PGN-251 runtime bits before write.
 */

#include "actuator.h"
#include "hal/hal.h"

#include "log_config.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_ACT
#include "esp_log.h"
#include "log_ext.h"

namespace {
static constexpr uint8_t CONFIG_BIT_MOTOR_DIR_INVERT = 0x04;
static constexpr uint8_t CONFIG_BIT_DRIVER_CYTRON = 0x10;
static constexpr uint32_t ACT_FRESHNESS_TIMEOUT_MS = 300;

static struct {
    bool detected = false;
    bool quality_ok = false;
    uint32_t last_update_ms = 0;
    int32_t error_code = 0;
} s_act_state;

static struct {
    bool motor_dir_invert = false;
    bool driver_cytron = false;
    uint8_t max_pulse = 255;
} s_act_cfg;
static uint16_t s_last_cmd = 0;

bool actuator_enabled_check() {
    return feat::act();
}
}  // namespace

void actuatorInit(void) {
    hal_actuator_begin();
    s_act_state.detected = hal_actuator_detect();
    s_act_state.quality_ok = s_act_state.detected;
    s_act_state.last_update_ms = hal_millis();
    s_act_state.error_code = s_act_state.detected ? 0 : 1;

    LOGI("ACT", "initialised (SPI protocol, detected=%u)",
         (unsigned)s_act_state.detected);
}

void actuatorApplyConfigBits(uint8_t set0, uint8_t max_pulse) {
    s_act_cfg.motor_dir_invert = (set0 & CONFIG_BIT_MOTOR_DIR_INVERT) != 0;
    s_act_cfg.driver_cytron = (set0 & CONFIG_BIT_DRIVER_CYTRON) != 0;
    s_act_cfg.max_pulse = max_pulse;
}

bool actuatorUpdate(uint16_t cmd) {
    if (!s_act_state.detected) {
        s_act_state.quality_ok = false;
        s_act_state.error_code = 2;
        return false;
    }

    uint16_t bounded = cmd;
    if (bounded > s_act_cfg.max_pulse) {
        bounded = s_act_cfg.max_pulse;
    }

    // Driver-aware command shaping:
    // - Cytron path: 0..max directly.
    // - IBT2 path: mirror magnitude into signed midpoint range.
    uint16_t hw_cmd = bounded;
    if (!s_act_cfg.driver_cytron) {
        const uint16_t midpoint = 32768;
        hw_cmd = static_cast<uint16_t>(midpoint + bounded);
    }

    if (s_act_cfg.motor_dir_invert) {
        hw_cmd = static_cast<uint16_t>(0xFFFFu - hw_cmd);
    }

    hal_actuator_write(hw_cmd);
    s_last_cmd = cmd;

    s_act_state.quality_ok = true;
    s_act_state.last_update_ms = hal_millis();
    s_act_state.error_code = 0;
    return true;
}

bool actuatorIsHealthy(uint32_t now_ms) {
    return s_act_state.detected &&
           s_act_state.quality_ok &&
           (now_ms - s_act_state.last_update_ms <= ACT_FRESHNESS_TIMEOUT_MS) &&
           (s_act_state.error_code == 0);
}

const ModuleOps actuator_ops = {
    "ACT",
    actuator_enabled_check,
    actuatorInit,
    []() -> bool { return actuatorUpdate(s_last_cmd); },
    actuatorIsHealthy
};
