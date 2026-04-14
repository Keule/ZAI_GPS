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

namespace feat {
inline constexpr bool comm()          { return FEAT_ENABLED(FEAT_COMM); }
inline constexpr bool gnss()          { return FEAT_ENABLED(FEAT_GNSS); }
inline constexpr bool imu()           { return FEAT_ENABLED(FEAT_IMU); }
inline constexpr bool steer_sensor()  { return FEAT_ENABLED(FEAT_STEER_SENSOR); }
inline constexpr bool steer_actor()   { return FEAT_ENABLED(FEAT_STEER_ACTOR); }
inline constexpr bool machine_sensor(){ return FEAT_ENABLED(FEAT_MACHINE_SENSOR); }
inline constexpr bool machine_actor() { return FEAT_ENABLED(FEAT_MACHINE_ACTOR); }
inline constexpr bool sensor()        { return FEAT_ENABLED(FEAT_STEER_SENSOR); }  // legacy helper
inline constexpr bool actor()         { return FEAT_ENABLED(FEAT_STEER_ACTOR); }   // legacy helper
inline constexpr bool control()       { return FEAT_ENABLED(FEAT_MACHINE_ACTOR); }
inline constexpr bool pid()           { return FEAT_ENABLED(FEAT_PID); }
}  // namespace feat
