/**
 * @file was.cpp
 * @brief Wheel Angle Sensor (WAS) implementation.
 */

#include "was.h"

#include "dependency_policy.h"
#include "hal/hal.h"

#include "log_config.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_WAS
#include "esp_log.h"
#include "log_ext.h"

namespace {
static constexpr uint8_t CONFIG_BIT_INVERT_WAS = 0x01;
static constexpr uint8_t CONFIG_BIT_SINGLE_INPUT_WAS = 0x08;

float s_last_was_angle_deg = 0.0f;
int16_t s_last_was_raw = 0;
uint32_t s_last_was_timestamp_ms = 0;
bool s_last_was_quality = false;

static struct {
    bool detected = false;
    bool quality_ok = false;
    uint32_t last_update_ms = 0;
    int32_t error_code = 0;
} s_was_state;

static struct {
    bool invert_was = false;
    bool single_input = false;
} s_was_cfg;

bool was_enabled_check() {
    return feat::ads();
}

bool was_health_check(uint32_t now_ms) {
    return wasIsHealthy(now_ms);
}
}  // namespace

void wasInit(void) {
    hal_steer_angle_begin();
    s_was_state.detected = hal_steer_angle_detect();
    s_was_state.quality_ok = s_was_state.detected;
    s_was_state.last_update_ms = hal_millis();
    s_was_state.error_code = s_was_state.detected ? 0 : 1;
    LOGI("WAS", "initialised (SPI protocol, detected=%u)",
         (unsigned)s_was_state.detected);
}

void wasApplyConfigBits(uint8_t set0) {
    s_was_cfg.invert_was = (set0 & CONFIG_BIT_INVERT_WAS) != 0;
    s_was_cfg.single_input = (set0 & CONFIG_BIT_SINGLE_INPUT_WAS) != 0;
}

bool wasUpdate(void) {
    if (!s_was_state.detected) {
        s_was_state.quality_ok = false;
        s_was_state.error_code = 2;
        return false;
    }

    float angle = hal_steer_angle_read_deg();
    int16_t raw = hal_steer_angle_read_raw();

    if (s_was_cfg.invert_was) {
        angle = -angle;
        raw = static_cast<int16_t>(-raw);
    }
    if (s_was_cfg.single_input && angle < 0.0f) {
        angle = 0.0f;
    }

    s_last_was_angle_deg = angle;
    s_last_was_raw = raw;
    s_last_was_timestamp_ms = hal_millis();
    s_last_was_quality = dep_policy::isSteerAnglePlausible(s_last_was_angle_deg) &&
                         dep_policy::isSteerAngleRawPlausible(s_last_was_raw);

    s_was_state.quality_ok = s_last_was_quality;
    s_was_state.last_update_ms = s_last_was_timestamp_ms;
    s_was_state.error_code = s_last_was_quality ? 0 : 3;
    return s_last_was_quality;
}

bool wasIsHealthy(uint32_t now_ms) {
    return s_was_state.detected &&
           s_was_state.quality_ok &&
           dep_policy::isFresh(now_ms,
                               s_was_state.last_update_ms,
                               dep_policy::STEER_ANGLE_FRESHNESS_TIMEOUT_MS) &&
           (s_was_state.error_code == 0);
}

float wasGetAngleDeg(void) {
    return s_last_was_angle_deg;
}

int16_t wasGetRaw(void) {
    return s_last_was_raw;
}

uint32_t wasGetTimestampMs(void) {
    return s_last_was_timestamp_ms;
}

bool wasGetQuality(void) {
    return s_last_was_quality;
}

float steerAngleReadDeg(void) {
    const bool ok = wasUpdate();
    if (!ok) return 0.0f;
    return s_last_was_angle_deg;
}

const ModuleOps was_ops = {
    "WAS",
    was_enabled_check,
    wasInit,
    wasUpdate,
    was_health_check
};
