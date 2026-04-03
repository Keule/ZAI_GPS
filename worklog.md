---
Task ID: 1
Agent: Main Agent
Task: Create complete AgOpenGPS steering controller firmware for LilyGO T-ETH-Lite-S3 (ESP32-S3 + W5500)

Work Log:
- Created project directory structure: ag-steer/{logic,hal,hal_esp32,hal_pc,firmware_esp32/src,firmware_pc}
- Created hardware_pins.h with all GPIO pin definitions for LilyGO T-ETH-Lite-S3
- Created logic/global_state.h/.cpp with NavigationState struct and thread-safe StateLock RAII helper
- Created hal/hal.h with complete hardware abstraction layer (C linkage)
- Created logic/aog_udp_protocol.h/.cpp with full AOG/AgIO Ethernet protocol:
  - Frame header, preamble, source IDs, PGN numbers, UDP ports
  - Packed structs for all message types (Hello, Scan, Subnet, SteerData, SteerStatus, GPS)
  - AogGpsMainOut validated at exactly 51 bytes (static_assert)
  - Checksum computation (low byte of sum)
  - Encoders: HelloReplySteer, HelloReplyGps, SubnetReply, SteerStatusOut, GpsMainOut
  - Decoders: HelloFromAgio, ScanRequest, SubnetChange, SteerDataIn (with full header/CRC validation)
  - Hex dump utility
- Created logic/gnss.h/.cpp with NMEA parser (GGA, RMC) for latitude, longitude, altitude, SOG, COG, fix quality
- Created logic/imu.h/.cpp with BNO085 SPI stub (reads yaw_rate_dps and roll_deg)
- Created logic/steer_angle.h/.cpp with SPI steering angle sensor stub
- Created logic/actuator.h/.cpp with SPI actuator stub
- Created logic/control.h/.cpp with complete PID controller:
  - Anti-windup integral clamping
  - Error wrapping to [-180, +180] for steering
  - Safety integration (immediate shutdown on safety fault)
  - 200 Hz control loop sequence: Safety -> IMU -> SteerAngle -> PID -> Actuator
- Created logic/net.h/.cpp with UDP communication layer:
  - Sends GPS Main Out on port 5124 @ 10 Hz
  - Sends Steer Status Out on port 5126 @ 10 Hz
  - Receives and processes: HelloFromAgIO, ScanRequest, SubnetChange, SteerDataIn
  - Auto-replies to Hello/Scan with Steer and GPS hello replies
  - Configurable destination IP via subnet change
- Created hal_pc/hal_impl.h/.cpp with full PC simulation:
  - std::chrono timing, std::mutex for thread safety
  - Simulated NMEA sentences (GGA/RMC with Berlin coordinates)
  - Simulated IMU (sinusoidal yaw rate and roll)
  - Simulated steer angle sensor (follows actuator command)
  - Network stubs (logging only)
- Created hal_esp32/hal_impl.h/.cpp with real ESP32-S3 implementation:
  - HSPI for W5500 Ethernet (GPIO48/21/47/45)
  - FSPI for sensor bus (GPIO12/11/13)
  - HardwareSerial for 2x GNSS UART (460800 baud)
  - SPI transactions with proper CS handling for IMU, steer angle, actuator
  - FreeRTOS recursive mutex for thread safety
  - Ethernet3 library integration for W5500
- Created firmware_esp32/src/main.cpp with two FreeRTOS tasks:
  - controlTask on Core 1 at 200 Hz
  - commTask on Core 0 at 100 Hz
- Created firmware_pc/main_pc.cpp with comprehensive simulation:
  - 1-second run with full state dump and frame verification
  - CRC verification for all generated frames
- Fixed NMEA parser field index off-by-one bug (nmeaField counts from string start including $GNGGA prefix)
- Fixed missing includes (cstdarg, cmath, cstdint)
- Fixed duplicate definition of desiredSteerAngleDeg
- Fixed PC HAL GNSS simulation (rate-limited returns to prevent infinite spin)
- Successfully built and tested PC simulation: g++ C++17, all frames CRC-verified

Stage Summary:
- Complete embedded firmware project with 18 files across 6 directories
- Clean architecture: hardware_pins -> HAL -> logic -> firmware
- AOG/AgIO protocol fully prepared with correct frame format, PGNs, source IDs
- GPS Main Out (PGN 0xD6): 51-byte payload with all fields encoded
- Steer Status Out (PGN 0xFD): 8-byte payload with angle/heading/roll/switch/PWM
- Hello replies for both Steer (0x7E) and GPS (0x78) modules
- PC simulation confirms: lat=52.517°, lon=13.4°, fix=4(RTK), SOG=5.0m/s, heading=91.5°
- Control loop verified at 200 Hz, PID converges correctly
- All CRC checksums verified correct
- ESP32 firmware ready for PlatformIO/Arduino IDE build (needs Ethernet3 library)
