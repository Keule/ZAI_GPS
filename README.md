# AgSteer — Autosteer System für AgOpenGPS

Multi-Gerät ESP32-Firmware für agriculturische Lenkung (AgOpenGPS / AgIO).

## Projektstruktur

```
ag-steer/
├── steer-controller/          # Lenk-Controller Firmware (ESP32-S3)
│   ├── platformio.ini         #   PlatformIO Konfiguration
│   ├── src/
│   │   ├── main.cpp           #   Einstiegspunkt, FreeRTOS Tasks
│   │   ├── hal/               #   Hardware Abstraction Layer (C API)
│   │   ├── hal_esp32/         #   ESP32-S3 HAL Implementierung
│   │   └── logic/             #   Reine Logik (PID, PGN, Module, ...)
│   ├── include/               #   Pin-Definitionen (hardware_pins.h)
│   ├── lib/                   #   Lokale Bibliotheken (ads1118, ETHClass2)
│   └── boards/                #   Custom Board Definition
│
├── gps-bridge/                # GPS-Bridge Firmware (TODO)
│   └── reference/             #   Ausgelagerte GNSS-Code Referenz
│       ├── gnss.h             #     NMEA Parser (GGA, RMC)
│       └── gnss.cpp           #     GNSS UART Implementierung
│
└── shared/                    # Gemeinsamer Code (TODO)
```

## Geräte

### Steering Controller (`steer-controller/`)
- **Board:** LilyGO T-ETH-Lite-S3 (ESP32-S3 + W5500 Ethernet)
- **Funktionen:**
  - ADS1118 Lenkwinkelsensor (SPI, FSPI)
  - BNO085 IMU (SPI)
  - Hydraulik-/Elektro-Aktuator (SPI)
  - PID Regelkreis (200 Hz)
  - Ethernet/UDP Kommunikation mit AgIO
  - SD-Karte OTA Firmware-Update
  - Lenkwinkel-Kalibrierung mit NVS-Persistenz

### GPS Bridge (`gps-bridge/`) — TODO
- **Board:** TBD (ESP32-S3)
- **Funktionen:**
  - 2x GNSS UART (UM980 RTK-Rover)
  - NMEA Parsing (GGA, RMC)
  - GPS PGN (214) an AgIO senden
  - Dual-Antennen Heading

## Bauen

```bash
cd steer-controller
pio run -t upload
```

## Testbare Matrix (Build + Smoke)

Im Repository ist eine CI-Matrix hinterlegt (`.github/workflows/test-matrix.yml`):

- **Build-Matrix** fuer alle Profile:
  - `profile_comm_only`
  - `profile_sensor_front`
  - `profile_actor_rear`
  - `profile_full_steer`
  - (spaeter erweiterbar fuer GNSS/Machine-Profile)
- **Smoke-Matrix** (Host-Runner) fuer:
  - Discovery/Hello/Subnet Frames
  - PGN-I/O-Szenarien je Profil
  - Timing-Metriken (Jitter + Deadline-Miss-Zaehler)

Lokal ausfuehren:

```bash
python3 tools/run_test_matrix.py
```

Nur Host-Smoke (ohne PlatformIO-Builds):

```bash
SKIP_PROFILE_BUILDS=1 python3 tools/run_test_matrix.py
```

### CI-Trigger-Pfade (`.github/workflows/test-matrix.yml`)

Die Test-Matrix wird nur bei relevanten Firmware-/Test-Aenderungen gestartet:

- `src/**`
- `include/**`
- `lib/**`
- `boards/**`
- `tools/**`
- `platformio.ini`
- `auto_version.py`
- `partitions_ota.csv`
- `.github/workflows/test-matrix.yml`

Hinweis: Das Repository-Root ist das Projekt-Root (kein `ag-steer/steer-controller`-Unterordner). Deshalb nutzt die CI keinen `working-directory`-Override mehr.
