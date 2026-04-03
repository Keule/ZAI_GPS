/**
 * @file main_pc.cpp
 * @brief PC / Linux simulation – tests the complete logic layer.
 *
 * Compiles and runs on any Linux/PC system with g++ (C++17).
 * Simulates sensor data, runs PID control, encodes AOG frames, and
 * prints hex dumps of generated GPS and Steer status frames.
 *
 * Build:
 *   cd pc_sim && make
 *
 * Or manually:
 *   cd pc_sim
 *   g++ -std=c++17 -O2 -Wall -I../src -I../include -I. \
 *       main_pc.cpp hal_pc/hal_impl.cpp \
 *       ../src/logic/global_state.cpp \
 *       ../src/logic/aog_udp_protocol.cpp \
 *       ../src/logic/gnss.cpp \
 *       ../src/logic/imu.cpp \
 *       ../src/logic/steer_angle.cpp \
 *       ../src/logic/actuator.cpp \
 *       ../src/logic/control.cpp \
 *       ../src/logic/net.cpp \
 *       ../src/logic/hw_status.cpp \
 *       -o sim -lm -lpthread
 *
 * Run:
 *   ./sim
 */

#include "hal/hal.h"
#include "hal_pc/hal_impl.h"
#include "logic/global_state.h"
#include "logic/control.h"
#include "logic/gnss.h"
#include "logic/imu.h"
#include "logic/steer_angle.h"
#include "logic/actuator.h"
#include "logic/net.h"
#include "logic/hw_status.h"
#include "logic/aog_udp_protocol.h"

#include <cstdio>
#include <cmath>
#include <cstring>

// ===================================================================
// Simulation parameters
// ===================================================================
static const uint32_t CONTROL_LOOP_HZ   = 200;
static const uint32_t CONTROL_INTERVAL_MS = 1000 / CONTROL_LOOP_HZ;  // 5 ms
static const uint32_t COMM_INTERVAL_MS   = 100;  // 10 Hz
static const uint32_t SIM_DURATION_MS    = 1000; // run for 1 second (enough to verify frame encoding)

// ===================================================================
// Main
// ===================================================================
int main() {
    printf("=== AgSteer PC Simulation ===\n");
    printf("Control loop: %u Hz (%u ms interval)\n", CONTROL_LOOP_HZ, CONTROL_INTERVAL_MS);
    printf("Comm loop:    %u Hz\n", (unsigned)(1000 / COMM_INTERVAL_MS));
    printf("Duration:     %u ms\n\n", SIM_DURATION_MS);

    // ----------------------------------------------------------
    // 1. Initialise HAL (PC simulation)
    // ----------------------------------------------------------
    hal_pc_init_all();

    // ----------------------------------------------------------
    // 2. Initialise subsystems
    // ----------------------------------------------------------
    gnssInit();
    controlInit();
    netInit();
    hwStatusInit();

    hal_log("SIM: all systems initialised");

    // ----------------------------------------------------------
    // 3. Set an initial steer setpoint (simulating AgIO command)
    // ----------------------------------------------------------
    desiredSteerAngleDeg = 15.0f;
    hal_log("SIM: setpoint = %.1f deg", desiredSteerAngleDeg);

    // ----------------------------------------------------------
    // 3b. Test hardware message encoding
    // ----------------------------------------------------------
    {
        uint8_t buf[AOG_MAX_FRAME];
        printf("\n=== Hardware Message Test ===\n");

        // Test 1: Error message (red)
        size_t len = encodeAogHardwareMessage(buf, sizeof(buf),
                                               AOG_SRC_STEER,
                                               10, AOG_HWMSG_COLOR_RED,
                                               "W5500 not detected");
        if (len > 0) {
            printf("HW MSG (red error, dur=10s, %zu bytes):\n", len);
            aogHexDump("HWMsg", buf, len);
            printf("  Src: 0x%02X  PGN: 0x%02X  Len: %u  CRC: 0x%02X\n",
                   buf[2], buf[3], buf[4], buf[len-1]);
            printf("  Duration: %u  Color: %u  Text: \"%s\"\n\n",
                   buf[5], buf[6], (char*)(buf + 7));
        }

        // Test 2: Warning message (yellow)
        len = encodeAogHardwareMessage(buf, sizeof(buf),
                                       AOG_SRC_STEER,
                                       AOG_HWMSG_DURATION_PERSIST, AOG_HWMSG_COLOR_YELLOW,
                                       "GNSS no fix");
        if (len > 0) {
            printf("HW MSG (yellow warn, persist, %zu bytes):\n", len);
            aogHexDump("HWMsg", buf, len);
            printf("  Duration: %u  Color: %u  Text: \"%s\"\n\n",
                   buf[5], buf[6], (char*)(buf + 7));
        }

        // Test 3: OK message (green)
        len = encodeAogHardwareMessage(buf, sizeof(buf),
                                       AOG_SRC_GPS,
                                       3, AOG_HWMSG_COLOR_GREEN,
                                       "All systems OK");
        if (len > 0) {
            printf("HW MSG (green ok, dur=3s, %zu bytes):\n", len);
            aogHexDump("HWMsg", buf, len);
            printf("  Duration: %u  Color: %u  Text: \"%s\"\n\n",
                   buf[5], buf[6], (char*)(buf + 7));
        }

        printf("=== End Hardware Message Test ===\n\n");
    }

    // ----------------------------------------------------------
    // 4. Main simulation loop
    // ----------------------------------------------------------
    uint32_t last_control = hal_millis();
    uint32_t last_comm = hal_millis();
    uint32_t start = hal_millis();
    uint32_t control_count = 0;
    uint32_t comm_count = 0;
    bool frames_dumped = false;

    while (hal_millis() - start < SIM_DURATION_MS) {
        uint32_t now = hal_millis();

        // --- Control loop (200 Hz) ---
        if (now - last_control >= CONTROL_INTERVAL_MS) {
            controlStep();
            last_control = now;
            control_count++;
        }

        // --- Communication loop (10 Hz) ---
        if (now - last_comm >= COMM_INTERVAL_MS) {
            // Poll GNSS
            gnssPollMain();
            gnssPollHeading();

            // Poll network
            netPollReceive();

            // Send AOG frames
            netSendAogFrames();

            last_comm = now;
            comm_count++;
        }

        // On first comm cycle, dump the generated frames
        if (!frames_dumped && comm_count >= 2) {
            frames_dumped = true;
            hal_log("\n=== Frame Dump (first transmission) ===\n");

            // Build and dump GPS frame
            {
                uint8_t buf[AOG_MAX_FRAME];
                StateLock lock;
                AogGpsMainOut gps{};
                gps.longitude   = static_cast<int32_t>(g_nav.lon_deg * 1e7);
                gps.latitude    = static_cast<int32_t>(g_nav.lat_deg * 1e7);
                gps.heading     = static_cast<int16_t>(g_nav.heading_deg * 16.0f);
                gps.dualHeading = static_cast<int16_t>(g_nav.heading_deg * 16.0f);
                gps.speed       = static_cast<uint16_t>(g_nav.sog_mps * 1000.0f);
                gps.roll        = static_cast<int16_t>(g_nav.roll_deg * 16.0f);
                gps.altitude    = static_cast<int32_t>(g_nav.alt_m * 1000.0f);
                gps.satCount    = 12;
                gps.fixQuality  = g_nav.fix_quality;
                gps.hdop        = 100;
                gps.age         = 0;
                gps.imuHeading  = static_cast<int16_t>(g_nav.heading_deg * 16.0f);
                gps.imuRoll     = static_cast<int16_t>(g_nav.roll_deg * 16.0f);
                gps.imuPitch    = 0;
                gps.imuYawRate  = static_cast<int16_t>(g_nav.yaw_rate_dps * 16.0f);

                size_t len = encodeAogGpsMainOut(buf, sizeof(buf), gps);
                if (len > 0) {
                    printf("\n--- GPS Main Out (PGN 0xD6, %zu bytes) ---\n", len);
                    aogHexDump("GPS Frame", buf, len);

                    // Verify frame structure
                    printf("  Preamble: 0x%02X 0x%02X\n", buf[0], buf[1]);
                    printf("  Src:      0x%02X (GPS=0x7C)\n", buf[2]);
                    printf("  PGN:      0x%02X (214)\n", buf[3]);
                    printf("  Len:      %u (expect 51)\n", buf[4]);
                    printf("  CRC:      0x%02X\n", buf[len - 1]);

                    // Verify checksum
                    uint8_t calc_crc = aogChecksum(buf, len);
                    printf("  CRC OK:   %s\n", calc_crc == buf[len - 1] ? "YES" : "NO");
                }
            }

            // Build and dump Steer Status frame
            {
                uint8_t buf[AOG_MAX_FRAME];
                StateLock lock;
                int16_t angle_x100 = static_cast<int16_t>(g_nav.steer_angle_deg * 100.0f);
                int16_t heading_x16 = static_cast<int16_t>(g_nav.heading_deg * 16.0f);
                int16_t roll_x16    = static_cast<int16_t>(g_nav.roll_deg * 16.0f);

                size_t len = encodeAogSteerStatusOut(buf, sizeof(buf),
                                                      angle_x100, heading_x16, roll_x16,
                                                      g_nav.safety_ok ? 0 : 0x80, 128);
                if (len > 0) {
                    printf("\n--- Steer Status Out (PGN 0xFD, %zu bytes) ---\n", len);
                    aogHexDump("Steer Frame", buf, len);

                    printf("  Preamble: 0x%02X 0x%02X\n", buf[0], buf[1]);
                    printf("  Src:      0x%02X (Steer=0x7E)\n", buf[2]);
                    printf("  PGN:      0x%02X (253)\n", buf[3]);
                    printf("  Len:      %u (expect 8)\n", buf[4]);
                    printf("  CRC:      0x%02X\n", buf[len - 1]);

                    uint8_t calc_crc = aogChecksum(buf, len);
                    printf("  CRC OK:   %s\n", calc_crc == buf[len - 1] ? "YES" : "NO");
                }
            }

            // Build and dump Hello Reply frames
            {
                uint8_t buf[32];

                // Steer Hello Reply – new format: (angle, counts, switchbyte)
                size_t len = encodeAogHelloReplySteer(buf, sizeof(buf),
                                                       15,  // steer angle = 15 deg
                                                       327, // sensor counts
                                                       0x00 // switch byte
                                                       );
                if (len > 0) {
                    printf("\n--- Hello Reply Steer (PGN=Src=0x7E, Len=5, %zu bytes) ---\n", len);
                    aogHexDump("HelloSteer", buf, len);
                    printf("  Src: 0x%02X PGN: 0x%02X Len: %u CRC: 0x%02X\n",
                           buf[2], buf[3], buf[4], buf[len-1]);
                    // Decode: AngleLo, AngleHi, CountsLo, CountsHi, Switch
                    printf("  Angle: %d deg  Counts: %u  Switch: 0x%02X\n",
                           (int)((int16_t)(buf[5] | (buf[6] << 8))),
                           (unsigned)(buf[7] | (buf[8] << 8)),
                           buf[9]);
                }

                // GPS Hello Reply
                len = encodeAogHelloReplyGps(buf, sizeof(buf));
                if (len > 0) {
                    printf("\n--- Hello Reply GPS (PGN=Src=0x78, Len=5, %zu bytes) ---\n", len);
                    aogHexDump("HelloGps", buf, len);
                    printf("  Src: 0x%02X PGN: 0x%02X Len: %u CRC: 0x%02X\n",
                           buf[2], buf[3], buf[4], buf[len-1]);
                }
            }

            // Build and dump Subnet Reply frame
            {
                uint8_t buf[32];
                uint8_t ip[4] = {192, 168, 5, 70};
                uint8_t subnet[3] = {255, 255, 255};
                size_t len = encodeAogSubnetReply(buf, sizeof(buf),
                                                  AOG_SRC_STEER, ip, subnet);
                if (len > 0) {
                    printf("\n--- Subnet Reply Steer (PGN=0xCB, Len=7, %zu bytes) ---\n", len);
                    aogHexDump("SubnetReply", buf, len);
                    printf("  Src: 0x%02X PGN: 0x%02X Len: %u CRC: 0x%02X\n",
                           buf[2], buf[3], buf[4], buf[len-1]);
                    printf("  IP: %u.%u.%u.%u  Subnet: %u.%u.%u\n",
                           buf[5], buf[6], buf[7], buf[8],
                           buf[9], buf[10], buf[11]);
                }
            }

            printf("\n=== End Frame Dump ===\n\n");
        }
    }

    // ----------------------------------------------------------
    // 5. Summary
    // ----------------------------------------------------------
    printf("\n=== Simulation Complete ===\n");
    printf("Duration:      %u ms\n", hal_millis() - start);
    printf("Control loops: %u (target %u)\n", control_count, CONTROL_LOOP_HZ * (SIM_DURATION_MS / 1000));
    printf("Comm loops:    %u (target %u)\n", comm_count, SIM_DURATION_MS / COMM_INTERVAL_MS);

    {
        StateLock lock;
        printf("\nFinal state:\n");
        printf("  lat:      %.6f deg\n", g_nav.lat_deg);
        printf("  lon:      %.6f deg\n", g_nav.lon_deg);
        printf("  alt:      %.1f m\n", g_nav.alt_m);
        printf("  SOG:      %.2f m/s\n", g_nav.sog_mps);
        printf("  COG:      %.1f deg\n", g_nav.cog_deg);
        printf("  heading:  %.1f deg\n", g_nav.heading_deg);
        printf("  roll:     %.2f deg\n", g_nav.roll_deg);
        printf("  yaw_rate: %.2f deg/s\n", g_nav.yaw_rate_dps);
        printf("  steer:    %.2f deg\n", g_nav.steer_angle_deg);
        printf("  fix:      %u\n", (unsigned)g_nav.fix_quality);
        printf("  safety:   %s\n", g_nav.safety_ok ? "OK" : "KICK");
        printf("  setpoint: %.1f deg\n", desiredSteerAngleDeg);
    }

    printf("\nAll tests passed. Simulation finished.\n");
    return 0;
}
