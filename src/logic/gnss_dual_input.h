/**
 * @file gnss_dual_input.h
 * @brief Dual-receiver GNSS validation and steering selection model.
 */

#pragma once

#include <array>
#include <cstdint>

#include "gnss_contract.h"

namespace gnss_dual {

enum class FixType : uint8_t {
    NONE = 0,
    SINGLE = 1,
    DGPS_OR_RTK_FLOAT = 2,
    RTK_FIXED = 3,
};

/// Raw observation after transport/parsing step.
/// This type intentionally carries no UM980/UM982-specific details.
struct Observation {
    uint8_t source_index;   ///< 0 or 1
    uint32_t timestamp_ms;
    double latitude_deg;
    double longitude_deg;
    float heading_deg;
    float speed_mps;
    float roll_deg;
    float hdop;
    uint8_t sat_count;
    FixType fix;
    float diff_age_s;
    bool frame_crc_ok;
};

struct Validation {
    bool finite_ok;
    bool range_ok;
    bool geometry_ok;
    bool freshness_ok;
    bool valid_for_steering;
};

Validation validateObservation(const Observation& obs, uint32_t now_ms);

class DualInputModel {
public:
    void update(const Observation& obs);

    /// Select best candidate and map to steering-core contract.
    bool buildSteerSample(uint32_t now_ms, gnss_contract::SteerCoreGnssSample& out) const;

private:
    struct Slot {
        bool has_sample;
        Observation obs;
    };

    static int score(const Observation& obs, const Validation& val);

    std::array<Slot, 2> slots_{};
};

}  // namespace gnss_dual
