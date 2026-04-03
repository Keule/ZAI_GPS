/**
 * @file global_state.cpp
 * @brief Global navigation state instance definition.
 */

#include "global_state.h"

/// Global navigation state – single source of truth
NavigationState g_nav = {};

/// Setpoint from AgIO (written by commTask when steer data arrives)
volatile float desiredSteerAngleDeg = 0.0f;
