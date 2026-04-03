/**
 * @file gnss.h
 * @brief GNSS module – NMEA parser for GGA and RMC sentences.
 *
 * Pure logic – depends on HAL for UART read.
 */

#pragma once

#include "global_state.h"

/// Initialise GNSS processing (calls hal_gnss_init).
void gnssInit(void);

/// Process all available NMEA lines from GNSS MAIN UART.
/// Updates g_nav fields (lat, lon, alt, sog, cog, fix_quality).
void gnssPollMain(void);

/// Process all available NMEA lines from GNSS HEADING UART.
/// Updates g_nav.heading_deg from COG (stub for dual-antenna fusion).
void gnssPollHeading(void);

// ===================================================================
// NMEA Parser helpers (exposed for testing)
// ===================================================================

/// Parse a single GGA sentence and fill state fields.
/// Returns true if fix was valid.
bool nmeaParseGGA(const char* sentence, NavigationState* state);

/// Parse a single RMC sentence and fill state fields.
/// Returns true if sentence was valid.
bool nmeaParseRMC(const char* sentence, NavigationState* state);
