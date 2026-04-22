#pragma once

/**
 * @file features.h
 * @brief Zentrale Feature-Flags und Helper fuer Build-Profile.
 *
 * Alle Feature-Entscheidungen im Code sollen ueber diesen Header laufen.
 */

// -------------------------------------------------------------------
// Roh-Flags aus Build-System (platformio.ini -D...) auf 0/1 normieren.
// Wichtig: Build-Defines selbst NICHT neu definieren.
// -------------------------------------------------------------------
#if defined(FEAT_PROFILE_COMM_ONLY)
#define FEAT_CFG_PROFILE_COMM_ONLY 1
#else
#define FEAT_CFG_PROFILE_COMM_ONLY 0
#endif

#if defined(FEAT_PROFILE_SENSOR_FRONT)
#define FEAT_CFG_PROFILE_SENSOR_FRONT 1
#else
#define FEAT_CFG_PROFILE_SENSOR_FRONT 0
#endif

#if defined(FEAT_PROFILE_ACTOR_REAR)
#define FEAT_CFG_PROFILE_ACTOR_REAR 1
#else
#define FEAT_CFG_PROFILE_ACTOR_REAR 0
#endif

#if defined(FEAT_PROFILE_FULL_STEER)
#define FEAT_CFG_PROFILE_FULL_STEER 1
#else
#define FEAT_CFG_PROFILE_FULL_STEER 0
#endif

#define FEAT_CFG_PROFILE_ANY ( \
    FEAT_CFG_PROFILE_COMM_ONLY || \
    FEAT_CFG_PROFILE_SENSOR_FRONT || \
    FEAT_CFG_PROFILE_ACTOR_REAR || \
    FEAT_CFG_PROFILE_FULL_STEER)

// -------------------------------------------------------------------
// Kanonische Capability-Rohwerte aus -D FEAT_... Flags
// Pflicht:
//   FEAT_COMM
// Optional:
//   FEAT_GNSS, FEAT_IMU, FEAT_STEER_SENSOR, FEAT_STEER_ACTOR,
//   FEAT_MACHINE_SENSOR, FEAT_MACHINE_ACTOR
// -------------------------------------------------------------------

#if defined(FEAT_COMM) || defined(FEAT_COMM_ETH)
#define FEAT_CFG_RAW_COMM 1
#else
#define FEAT_CFG_RAW_COMM 0
#endif

#if defined(FEAT_GNSS)
#define FEAT_CFG_RAW_GNSS 1
#else
#define FEAT_CFG_RAW_GNSS 0
#endif

#if defined(FEAT_IMU) || defined(FEAT_IMU_FRONT)
#define FEAT_CFG_RAW_IMU 1
#else
#define FEAT_CFG_RAW_IMU 0
#endif

#if defined(FEAT_STEER_SENSOR) || defined(FEAT_SENSOR_FRONT)
#define FEAT_CFG_RAW_STEER_SENSOR 1
#else
#define FEAT_CFG_RAW_STEER_SENSOR 0
#endif

#if defined(FEAT_STEER_ACTOR) || defined(FEAT_ACTOR_REAR)
#define FEAT_CFG_RAW_STEER_ACTOR 1
#else
#define FEAT_CFG_RAW_STEER_ACTOR 0
#endif

#if defined(FEAT_MACHINE_SENSOR)
#define FEAT_CFG_RAW_MACHINE_SENSOR 1
#else
#define FEAT_CFG_RAW_MACHINE_SENSOR 0
#endif

#if defined(FEAT_MACHINE_ACTOR) || defined(FEAT_CONTROL_LOOP)
#define FEAT_CFG_RAW_MACHINE_ACTOR 1
#else
#define FEAT_CFG_RAW_MACHINE_ACTOR 0
#endif

#if defined(FEAT_PID_STEER)
#define FEAT_CFG_RAW_PID_STEER 1
#else
#define FEAT_CFG_RAW_PID_STEER 0
#endif

// Zusätzliche Compile-Time-Capabilities (SPI/UART), optional per -D übersteuerbar.
#if defined(FEAT_CAP_SENSOR_SPI2)
#define FEAT_CFG_RAW_CAP_SENSOR_SPI2 1
#else
#define FEAT_CFG_RAW_CAP_SENSOR_SPI2 0
#endif

#if defined(FEAT_GNSS_UART_MIRROR) || defined(FEAT_CAP_GNSS_UART_MIRROR)
#define FEAT_CFG_RAW_CAP_GNSS_UART_MIRROR 1
#else
#define FEAT_CFG_RAW_CAP_GNSS_UART_MIRROR 0
#endif

// Profil-Defaults (falls Profil aktiv, aber einzelne Raw-Flags fehlen)
#define FEAT_CFG_PROF_COMM          (FEAT_CFG_PROFILE_COMM_ONLY || FEAT_CFG_PROFILE_SENSOR_FRONT || FEAT_CFG_PROFILE_ACTOR_REAR || FEAT_CFG_PROFILE_FULL_STEER)
#define FEAT_CFG_PROF_GNSS          (0)
#define FEAT_CFG_PROF_IMU           (FEAT_CFG_PROFILE_SENSOR_FRONT || FEAT_CFG_PROFILE_FULL_STEER)
#define FEAT_CFG_PROF_STEER_SENSOR  (FEAT_CFG_PROFILE_SENSOR_FRONT || FEAT_CFG_PROFILE_FULL_STEER)
#define FEAT_CFG_PROF_STEER_ACTOR   (FEAT_CFG_PROFILE_ACTOR_REAR || FEAT_CFG_PROFILE_FULL_STEER)
#define FEAT_CFG_PROF_MACHINE_SENSOR (FEAT_CFG_PROFILE_SENSOR_FRONT || FEAT_CFG_PROFILE_FULL_STEER)
#define FEAT_CFG_PROF_MACHINE_ACTOR (FEAT_CFG_PROFILE_ACTOR_REAR || FEAT_CFG_PROFILE_FULL_STEER)
#define FEAT_CFG_PROF_PID           (FEAT_CFG_PROFILE_FULL_STEER)

// Legacy-Default: ohne Profil alles aktiv (bisheriges Verhalten).
#define FEAT_CFG_DEFAULT_ON (!FEAT_CFG_PROFILE_ANY)

// -------------------------------------------------------------------
// Abgeleitete, zentrale Feature-Makros (kanonisch)
// -------------------------------------------------------------------
#define FEAT_COMM_NORM           (FEAT_CFG_RAW_COMM || (FEAT_CFG_PROF_COMM && !FEAT_CFG_RAW_COMM) || FEAT_CFG_DEFAULT_ON)
#define FEAT_GNSS_NORM           (FEAT_CFG_RAW_GNSS || (FEAT_CFG_PROF_GNSS && !FEAT_CFG_RAW_GNSS) || FEAT_CFG_DEFAULT_ON)
#define FEAT_IMU_NORM            (FEAT_CFG_RAW_IMU || (FEAT_CFG_PROF_IMU && !FEAT_CFG_RAW_IMU) || FEAT_CFG_DEFAULT_ON)
#define FEAT_STEER_SENSOR_NORM   (FEAT_CFG_RAW_STEER_SENSOR || (FEAT_CFG_PROF_STEER_SENSOR && !FEAT_CFG_RAW_STEER_SENSOR) || FEAT_CFG_DEFAULT_ON)
#define FEAT_STEER_ACTOR_NORM    (FEAT_CFG_RAW_STEER_ACTOR || (FEAT_CFG_PROF_STEER_ACTOR && !FEAT_CFG_RAW_STEER_ACTOR) || FEAT_CFG_DEFAULT_ON)
#define FEAT_MACHINE_SENSOR_NORM (FEAT_CFG_RAW_MACHINE_SENSOR || (FEAT_CFG_PROF_MACHINE_SENSOR && !FEAT_CFG_RAW_MACHINE_SENSOR) || FEAT_CFG_DEFAULT_ON)
#define FEAT_MACHINE_ACTOR_NORM  (FEAT_CFG_RAW_MACHINE_ACTOR || (FEAT_CFG_PROF_MACHINE_ACTOR && !FEAT_CFG_RAW_MACHINE_ACTOR) || FEAT_CFG_DEFAULT_ON)
#define FEAT_PID_STEER_NORM      (FEAT_CFG_RAW_PID_STEER || (FEAT_CFG_PROF_PID && !FEAT_CFG_RAW_PID_STEER) || FEAT_CFG_DEFAULT_ON)

// Pflicht-Capability
#ifndef FEAT_COMM
#define FEAT_COMM (FEAT_COMM_NORM)
#endif

// Optionale Capabilities
#ifndef FEAT_GNSS
#define FEAT_GNSS (FEAT_GNSS_NORM)
#endif
#ifndef FEAT_IMU
#define FEAT_IMU (FEAT_STEER_SENSOR_NORM && FEAT_IMU_NORM)
#endif
#ifndef FEAT_STEER_SENSOR
#define FEAT_STEER_SENSOR (FEAT_STEER_SENSOR_NORM)
#endif
#ifndef FEAT_STEER_ACTOR
#define FEAT_STEER_ACTOR (FEAT_STEER_ACTOR_NORM)
#endif
#ifndef FEAT_MACHINE_SENSOR
#define FEAT_MACHINE_SENSOR (FEAT_MACHINE_SENSOR_NORM)
#endif
#ifndef FEAT_MACHINE_ACTOR
#define FEAT_MACHINE_ACTOR (FEAT_MACHINE_ACTOR_NORM && FEAT_STEER_SENSOR && FEAT_STEER_ACTOR)
#endif

#define FEAT_STEER_ALL (FEAT_COMM && FEAT_STEER_SENSOR && FEAT_STEER_ACTOR && FEAT_MACHINE_ACTOR)

// Abgeleitete Zusatz-Capabilities (Compile-Time-Gating für zusätzliche Interfaces)
#if defined(FEAT_GNSS_BUILDUP)
#define FEAT_CFG_MODE_GNSS_BUILDUP 1
#else
#define FEAT_CFG_MODE_GNSS_BUILDUP 0
#endif

#define FEAT_CFG_MOD_NEEDS_SENSOR_SPI2 (FEAT_STEER_SENSOR || FEAT_STEER_ACTOR || FEAT_IMU || FEAT_MACHINE_ACTOR)
#define FEAT_CFG_MOD_NEEDS_GNSS_UART_MIRROR (FEAT_GNSS && FEAT_CFG_MODE_GNSS_BUILDUP)

#ifndef FEAT_CAP_SENSOR_SPI2
#define FEAT_CAP_SENSOR_SPI2 (FEAT_CFG_RAW_CAP_SENSOR_SPI2 || FEAT_CFG_MOD_NEEDS_SENSOR_SPI2)
#endif

#ifndef FEAT_CAP_GNSS_UART_MIRROR
#define FEAT_CAP_GNSS_UART_MIRROR (FEAT_CFG_RAW_CAP_GNSS_UART_MIRROR && FEAT_CFG_MOD_NEEDS_GNSS_UART_MIRROR)
#endif

// -------------------------------------------------------------------
// NTRIP-Client Capability — TASK-025
// Abhaengig von FEAT_GNSS && FEAT_COMM (braucht Ethernet + GNSS UART).
// Kann per -DFEAT_NTRIP aktiviert werden.
// -------------------------------------------------------------------
#if defined(FEAT_NTRIP)
#define FEAT_CFG_RAW_NTRIP 1
#else
#define FEAT_CFG_RAW_NTRIP 0
#endif

#define FEAT_NTRIP_NORM (FEAT_CFG_RAW_NTRIP && FEAT_GNSS && FEAT_COMM)

#ifndef FEAT_NTRIP
#define FEAT_NTRIP (FEAT_NTRIP_NORM)
#endif

static_assert(FEAT_COMM, "FEAT_COMM muss aktiv sein (mindestens Ethernet/UDP Kommunikation).");

// -------------------------------------------------------------------
// Legacy-Kompatibilitaetsschicht (alt -> neu)
// -------------------------------------------------------------------
// FEAT_COMM_ETH    -> FEAT_COMM
// FEAT_SENSOR_FRONT-> FEAT_STEER_SENSOR
// FEAT_IMU_FRONT   -> FEAT_IMU
// FEAT_ACTOR_REAR  -> FEAT_STEER_ACTOR
// FEAT_CONTROL_LOOP-> FEAT_MACHINE_ACTOR
// FEAT_SENSOR      -> FEAT_STEER_SENSOR
// FEAT_ACTOR       -> FEAT_STEER_ACTOR
// FEAT_CONTROL     -> FEAT_MACHINE_ACTOR
// FEAT_PID         -> FEAT_MACHINE_ACTOR && FEAT_PID_STEER_NORM

#define FEAT_COMM_ETH   (FEAT_COMM)
#define FEAT_SENSOR     (FEAT_STEER_SENSOR)
#define FEAT_IMU_FRONT  (FEAT_IMU)
#define FEAT_SENSOR_FRONT (FEAT_STEER_SENSOR)
#define FEAT_ACTOR      (FEAT_STEER_ACTOR)
#define FEAT_ACTOR_REAR (FEAT_STEER_ACTOR)
#define FEAT_CONTROL_LOOP (FEAT_MACHINE_ACTOR)
#define FEAT_CONTROL    (FEAT_MACHINE_ACTOR)
#define FEAT_PID        (FEAT_MACHINE_ACTOR && FEAT_PID_STEER_NORM)

// -------------------------------------------------------------------
// Helper-Makros/Funktionen fuer Feature-Abfragen
// -------------------------------------------------------------------
#define FEAT_ENABLED(flag_macro) ((flag_macro) != 0)
#define FEAT_DISABLED(flag_macro) ((flag_macro) == 0)

// -------------------------------------------------------------------
// Neue flache Hardware-Feature-Sicht (TASK-040 Migrationsschritt)
// -------------------------------------------------------------------
#if defined(FEAT_ETH)
#define FEAT_COMPILED_ETH 1
#else
#define FEAT_COMPILED_ETH FEAT_COMM
#endif

#if defined(FEAT_ADS)
#define FEAT_COMPILED_ADS 1
#else
#define FEAT_COMPILED_ADS FEAT_STEER_SENSOR
#endif

#if defined(FEAT_ACT)
#define FEAT_COMPILED_ACT 1
#else
#define FEAT_COMPILED_ACT FEAT_STEER_ACTOR
#endif

#if defined(FEAT_SAFETY)
#define FEAT_COMPILED_SAFETY 1
#else
#define FEAT_COMPILED_SAFETY FEAT_MACHINE_ACTOR
#endif

#if defined(FEAT_SD)
#define FEAT_COMPILED_SD 1
#else
#define FEAT_COMPILED_SD 0
#endif

#if defined(FEAT_LOGSW)
#define FEAT_COMPILED_LOGSW 1
#else
#define FEAT_COMPILED_LOGSW 0
#endif

#define FEAT_COMPILED_IMU   FEAT_IMU
#define FEAT_COMPILED_GNSS  FEAT_GNSS
#define FEAT_COMPILED_NTRIP FEAT_NTRIP

namespace feat {
inline constexpr bool eth()           { return FEAT_ENABLED(FEAT_COMPILED_ETH); }
inline constexpr bool gnss()          { return FEAT_ENABLED(FEAT_GNSS); }
inline constexpr bool imu()           { return FEAT_ENABLED(FEAT_COMPILED_IMU); }
inline constexpr bool ads()           { return FEAT_ENABLED(FEAT_COMPILED_ADS); }
inline constexpr bool act()           { return FEAT_ENABLED(FEAT_COMPILED_ACT); }
inline constexpr bool safety()        { return FEAT_ENABLED(FEAT_COMPILED_SAFETY); }
inline constexpr bool logsw()         { return FEAT_ENABLED(FEAT_COMPILED_LOGSW); }
inline constexpr bool machine_sensor(){ return FEAT_ENABLED(FEAT_MACHINE_SENSOR); }
inline constexpr bool machine_actor() { return act() && safety(); }
inline constexpr bool comm()          { return eth(); }   // legacy helper
inline constexpr bool steer_sensor()  { return ads(); }   // legacy helper
inline constexpr bool steer_actor()   { return act(); }   // legacy helper
inline constexpr bool sensor()        { return ads(); }   // legacy helper
inline constexpr bool actor()         { return act(); }   // legacy helper
inline constexpr bool control()       { return act() && safety(); }  // legacy helper
inline constexpr bool pid()           { return FEAT_ENABLED(FEAT_PID); }
inline constexpr bool ntrip()         { return FEAT_ENABLED(FEAT_COMPILED_NTRIP); }
}  // namespace feat
