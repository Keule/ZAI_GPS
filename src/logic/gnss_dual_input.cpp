/**
 * @file gnss_dual_input.cpp
 * @brief Dual-receiver GNSS validation and steering selection model.
 */

#include "gnss_dual_input.h"

#include <cmath>
#include <limits>

namespace gnss_dual {

namespace {

constexpr uint32_t GNSS_FRESHNESS_TIMEOUT_MS = 250;
constexpr float MAX_ABS_ROLL_DEG = 45.0f;
constexpr float MAX_HDOP = 3.5f;
constexpr uint8_t MIN_SAT_FOR_STEERING = 6;

bool isFinite(const Observation& obs) {
    return std::isfinite(obs.latitude_deg) && std::isfinite(obs.longitude_deg) &&
           std::isfinite(obs.heading_deg) && std::isfinite(obs.speed_mps) &&
           std::isfinite(obs.roll_deg) && std::isfinite(obs.hdop) &&
           std::isfinite(obs.diff_age_s);
}

bool inRange(const Observation& obs) {
    if (obs.latitude_deg < -90.0 || obs.latitude_deg > 90.0) return false;
    if (obs.longitude_deg < -180.0 || obs.longitude_deg > 180.0) return false;
    if (obs.heading_deg < 0.0f || obs.heading_deg >= 360.0f) return false;
    if (obs.speed_mps < 0.0f || obs.speed_mps > 60.0f) return false;
    if (std::fabs(obs.roll_deg) > MAX_ABS_ROLL_DEG) return false;
    if (obs.hdop < 0.0f || obs.hdop > 99.0f) return false;
    if (obs.diff_age_s < 0.0f || obs.diff_age_s > 60.0f) return false;
    if (obs.source_index > 1u) return false;
    return true;
}

bool geometryOk(const Observation& obs) {
    if (!obs.frame_crc_ok) return false;
    if (obs.fix == FixType::NONE) return false;
    if (obs.sat_count < MIN_SAT_FOR_STEERING) return false;
    if (obs.hdop > MAX_HDOP) return false;
    return true;
}

bool freshEnough(uint32_t now_ms, uint32_t ts_ms) {
    if (ts_ms == 0u) return false;
    return (now_ms - ts_ms) <= GNSS_FRESHNESS_TIMEOUT_MS;
}

uint8_t toQuality(FixType fix) {
    switch (fix) {
        case FixType::RTK_FIXED:
            return 3;
        case FixType::DGPS_OR_RTK_FLOAT:
            return 2;
        case FixType::SINGLE:
            return 1;
        default:
            return 0;
    }
}

}  // namespace

Validation validateObservation(const Observation& obs, uint32_t now_ms) {
    Validation v{};
    v.finite_ok = isFinite(obs);
    v.range_ok = v.finite_ok && inRange(obs);
    v.geometry_ok = v.range_ok && geometryOk(obs);
    v.freshness_ok = v.range_ok && freshEnough(now_ms, obs.timestamp_ms);
    v.valid_for_steering = v.geometry_ok && v.freshness_ok;
    return v;
}

void DualInputModel::update(const Observation& obs) {
    if (obs.source_index > 1u) return;

    Slot& slot = slots_[obs.source_index];
    if (!slot.has_sample || obs.timestamp_ms >= slot.obs.timestamp_ms) {
        slot.obs = obs;
        slot.has_sample = true;
    }
}

int DualInputModel::score(const Observation& obs, const Validation& val) {
    if (!val.valid_for_steering) return std::numeric_limits<int>::min();

    int s = 0;
    s += static_cast<int>(obs.fix) * 1000;
    s += static_cast<int>(obs.sat_count) * 10;
    s -= static_cast<int>(obs.hdop * 100.0f);
    s -= static_cast<int>(obs.diff_age_s * 10.0f);
    return s;
}

bool DualInputModel::buildSteerSample(uint32_t now_ms,
                                      gnss_contract::SteerCoreGnssSample& out) const {
    int best_index = -1;
    int best_score = std::numeric_limits<int>::min();

    for (size_t i = 0; i < slots_.size(); ++i) {
        const Slot& slot = slots_[i];
        if (!slot.has_sample) continue;

        const Validation val = validateObservation(slot.obs, now_ms);
        const int current_score = score(slot.obs, val);
        if (current_score > best_score) {
            best_score = current_score;
            best_index = static_cast<int>(i);
        }
    }

    if (best_index < 0) return false;

    const Observation& sel = slots_[best_index].obs;
    const Validation val = validateObservation(sel, now_ms);

    out.timestamp_ms = sel.timestamp_ms;
    out.latitude_deg = sel.latitude_deg;
    out.longitude_deg = sel.longitude_deg;
    out.heading_deg = sel.heading_deg;
    out.speed_mps = sel.speed_mps;
    out.roll_deg = sel.roll_deg;
    out.hdop = sel.hdop;
    out.sat_count = sel.sat_count;
    out.quality_code = toQuality(sel.fix);
    out.source_index = sel.source_index;
    out.valid_for_steering = val.valid_for_steering;
    return out.valid_for_steering;
}

}  // namespace gnss_dual
