#pragma once

// Legacy include shim (Phase 2 migration).
#include "was.h"

inline constexpr bool steerAngleIsEnabled() { return wasIsEnabled(); }
inline void steerAngleModuleInit() { wasInit(); }
inline bool steerAngleModuleUpdate() { return wasUpdate(); }
inline bool steerAngleModuleIsHealthy(uint32_t now_ms) { return wasIsHealthy(now_ms); }
