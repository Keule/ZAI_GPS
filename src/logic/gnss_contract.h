/**
 * @file gnss_contract.h
 * @brief Bridge-agnostic GNSS contract for steering-core integration.
 */

#pragma once

#include <cstdint>

namespace gnss_contract {

/// Steering-relevant output cadence derived from AgOpenGPS behavior.
constexpr uint32_t STEERING_GNSS_OUTPUT_HZ = 20;   // 50 ms
constexpr uint32_t STEERING_GNSS_MAX_AGE_MS = 250;

/// Minimal input contract consumed by steering core.
/// Deliberately bridge/internal-parser agnostic.
struct SteerCoreGnssSample {
    uint32_t timestamp_ms;      ///< sample creation timestamp (bridge monotonic time)
    double latitude_deg;
    double longitude_deg;
    float heading_deg;
    float speed_mps;
    float roll_deg;
    float hdop;
    uint8_t sat_count;
    uint8_t quality_code;       ///< 0=invalid,1=single,2=dgps/float,3=rtk-fixed
    uint8_t source_index;       ///< selected receiver index (0/1)
    bool valid_for_steering;
};

static_assert(sizeof(uint8_t) == 1, "uint8_t size assumption failed");

}  // namespace gnss_contract
