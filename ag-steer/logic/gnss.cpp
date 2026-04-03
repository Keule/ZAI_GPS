/**
 * @file gnss.cpp
 * @brief GNSS NMEA parser implementation.
 *
 * Supports:
 *   - GGA: position, altitude, fix quality, satellites
 *   - RMC: speed over ground, course over ground
 *
 * Pure C++ with HAL calls only.
 */

#include "gnss.h"
#include "hal/hal.h"

#include <cctype>
#include <cmath>
#include <cstring>
#include <cstdio>

// ===================================================================
// Helpers
// ===================================================================

/// Find the n-th comma-separated field (0-based). Returns pointer to
/// the start of the field content, or nullptr if not found.
static const char* nmeaField(const char* sentence, int field_index) {
    const char* p = sentence;
    for (int i = 0; i < field_index; i++) {
        p = std::strchr(p, ',');
        if (!p) return nullptr;
        p++;  // skip comma
    }
    return p;
}

/// Read a double from a field. Handles empty fields.
static double nmeaDouble(const char* field) {
    if (!field || *field == ',' || *field == '\0' || *field == '*') return 0.0;
    char* end;
    double val = strtod(field, &end);
    return val;
}

/// Convert NMEA latitude (DDMM.MMMMM) to decimal degrees.
static double nmeaLatToDecimal(double raw, char hemisphere) {
    double deg = std::floor(raw / 100.0);
    double minutes = raw - deg * 100.0;
    double decimal = deg + minutes / 60.0;
    if (hemisphere == 'S' || hemisphere == 's') decimal = -decimal;
    return decimal;
}

/// Convert NMEA longitude (DDDMM.MMMMM) to decimal degrees.
static double nmeaLonToDecimal(double raw, char hemisphere) {
    double deg = std::floor(raw / 100.0);
    double minutes = raw - deg * 100.0;
    double decimal = deg + minutes / 60.0;
    if (hemisphere == 'W' || hemisphere == 'w') decimal = -decimal;
    return decimal;
}

/// Check if sentence starts with a valid NMEA talker + sentence ID.
static bool nmeaStartsWith(const char* sentence, const char* prefix) {
    return std::strncmp(sentence, prefix, std::strlen(prefix)) == 0;
}

// ===================================================================
// GGA parser: $GNGGA or $GPGGA
// $GNGGA,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,geoidSep,M,age,stnID
// nmeaField counts commas from string start (including after $GNGGA)
// Comma 1=time, 2=lat, 3=N/S, 4=lon, 5=E/W, 6=fix, 7=sats, 8=hdop, 9=alt, 10=M
// ===================================================================
bool nmeaParseGGA(const char* sentence, NavigationState* state) {
    if (!nmeaStartsWith(sentence, "$GN") && !nmeaStartsWith(sentence, "$GP")) return false;
    // Check if it's a GGA sentence (find "GGA" after the talker ID)
    const char* gga = std::strstr(sentence, "GGA");
    if (!gga) return false;

    // Fix quality (field after 6th comma)
    const char* fixField = nmeaField(sentence, 6);
    int fix = (int)nmeaDouble(fixField);

    if (fix == 0) {
        state->fix_quality = 0;
        return false;
    }

    state->fix_quality = static_cast<uint8_t>(fix);

    // Latitude (field after 2nd comma)
    const char* latField = nmeaField(sentence, 2);
    double latRaw = nmeaDouble(latField);
    const char* nsField = nmeaField(sentence, 3);
    char ns = nsField ? nsField[0] : 'N';
    state->lat_deg = nmeaLatToDecimal(latRaw, ns);

    // Longitude (field after 4th comma)
    const char* lonField = nmeaField(sentence, 4);
    double lonRaw = nmeaDouble(lonField);
    const char* ewField = nmeaField(sentence, 5);
    char ew = ewField ? ewField[0] : 'E';
    state->lon_deg = nmeaLonToDecimal(lonRaw, ew);

    // Altitude (field after 9th comma)
    const char* altField = nmeaField(sentence, 9);
    state->alt_m = static_cast<float>(nmeaDouble(altField));

    return true;
}

// ===================================================================
// RMC parser: $GNRMC or $GPRMC
// $GNRMC,time,status,lat,N/S,lon,E/W,sog,cog,date,magVar,magVarDir,mode
// Comma 1=time, 2=status, 3=lat, 4=N/S, 5=lon, 6=E/W, 7=sog, 8=cog
// ===================================================================
bool nmeaParseRMC(const char* sentence, NavigationState* state) {
    if (!nmeaStartsWith(sentence, "$GN") && !nmeaStartsWith(sentence, "$GP")) return false;
    const char* rmc = std::strstr(sentence, "RMC");
    if (!rmc) return false;

    // Status (field after 2nd comma)
    const char* statusField = nmeaField(sentence, 2);
    if (!statusField || statusField[0] != 'A') return false;

    // Speed over ground (field after 7th comma, knots -> m/s: *0.514444)
    const char* sogField = nmeaField(sentence, 7);
    double sog_knots = nmeaDouble(sogField);
    state->sog_mps = static_cast<float>(sog_knots * 0.514444);

    // Course over ground (field after 8th comma)
    const char* cogField = nmeaField(sentence, 8);
    state->cog_deg = static_cast<float>(nmeaDouble(cogField));

    return true;
}

// ===================================================================
// Public API
// ===================================================================

void gnssInit(void) {
    hal_gnss_init();
    hal_log("GNSS: initialised (2x UART, 460800 baud)");
}

void gnssPollMain(void) {
    char line[256];
    while (hal_gnss_main_read_line(line, sizeof(line))) {
        StateLock lock;

        if (nmeaStartsWith(line, "$GN") || nmeaStartsWith(line, "$GP")) {
            // Try GGA first (it gives us position + fix quality)
            if (std::strstr(line, "GGA")) {
                nmeaParseGGA(line, &g_nav);
            }
            // Then RMC (speed and course)
            if (std::strstr(line, "RMC")) {
                nmeaParseRMC(line, &g_nav);
            }

            g_nav.timestamp_ms = hal_millis();
        }
    }
}

void gnssPollHeading(void) {
    char line[256];
    while (hal_gnss_heading_read_line(line, sizeof(line))) {
        StateLock lock;

        if (nmeaStartsWith(line, "$GN") || nmeaStartsWith(line, "$GP")) {
            if (std::strstr(line, "RMC")) {
                // Stub: use COG from heading GNSS as heading.
                // TODO: implement dual-antenna heading fusion.
                const char* cogField = nmeaField(line, 8);  // COG after 8th comma
                float cog = static_cast<float>(nmeaDouble(cogField));
                if (cog > 0.0f) {
                    g_nav.heading_deg = cog;
                }
            }

            g_nav.timestamp_ms = hal_millis();
        }
    }
}
