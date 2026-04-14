/**
 * @file global_state.cpp
 * @brief Global navigation state instance definition.
 */

#include "global_state.h"

namespace {
constexpr uint32_t FRESH_STEER_DATA_MS = 2500;
constexpr uint32_t FRESH_IMU_MS = 200;
constexpr uint32_t FRESH_STEER_ANGLE_MS = 200;
constexpr uint32_t FRESH_SETTINGS_MS = 30000;
constexpr uint32_t FRESH_CONFIG_MS = 30000;
constexpr float MIN_STEER_SPEED_KMH = 0.1f;
}  // namespace

/// Global navigation state – single source of truth
NavigationState g_nav = {};

/// Setpoint from AgIO (written by commTask when steer data arrives)
volatile float desiredSteerAngleDeg = 0.0f;

void markInputMeta(Capability capability,
                   uint32_t timestamp_ms,
                   uint8_t quality,
                   bool valid) {
    InputMeta* meta = nullptr;
    {
        StateLock lock;
        switch (capability) {
            case Capability::SteerDataIn:   meta = &g_nav.meta_steer_data; break;
            case Capability::Imu:           meta = &g_nav.meta_imu; break;
            case Capability::SteerAngle:    meta = &g_nav.meta_steer_angle; break;
            case Capability::SteerSettings: meta = &g_nav.meta_steer_settings; break;
            case Capability::SteerConfig:   meta = &g_nav.meta_steer_config; break;
            default: break;
        }
        if (meta != nullptr) {
            meta->timestamp_ms = timestamp_ms;
            meta->quality = quality;
            meta->valid = valid;
        }
    }
}

bool isFresh(const InputMeta& meta, uint32_t now_ms, uint32_t max_age_ms) {
    if (meta.timestamp_ms == 0) return false;
    return static_cast<uint32_t>(now_ms - meta.timestamp_ms) <= max_age_ms;
}

bool isValidAndFresh(const InputMeta& meta, uint32_t now_ms, uint32_t max_age_ms) {
    return meta.valid && isFresh(meta, now_ms, max_age_ms);
}

bool canBuildSteerStatusOut(const NavigationState& nav, uint32_t now_ms) {
    return isValidAndFresh(nav.meta_steer_angle, now_ms, FRESH_STEER_ANGLE_MS) &&
           isValidAndFresh(nav.meta_imu, now_ms, FRESH_IMU_MS);
}

bool canBuildFromAutosteer2(const NavigationState& nav, uint32_t now_ms) {
    return isValidAndFresh(nav.meta_steer_angle, now_ms, FRESH_STEER_ANGLE_MS);
}

bool canActuateSteer(const NavigationState& nav, uint32_t now_ms) {
    return nav.safety_ok &&
           nav.work_switch &&
           nav.steer_switch &&
           (nav.gps_speed_kmh >= MIN_STEER_SPEED_KMH) &&
           isValidAndFresh(nav.meta_steer_data, now_ms, FRESH_STEER_DATA_MS) &&
           isValidAndFresh(nav.meta_steer_angle, now_ms, FRESH_STEER_ANGLE_MS) &&
           isValidAndFresh(nav.meta_imu, now_ms, FRESH_IMU_MS) &&
           isValidAndFresh(nav.meta_steer_settings, now_ms, FRESH_SETTINGS_MS) &&
           isValidAndFresh(nav.meta_steer_config, now_ms, FRESH_CONFIG_MS);
}
