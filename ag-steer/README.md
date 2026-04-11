# AgSteer – Autosteer-Steuergerät für AgOpenGPS / AgIO

> Embedded-Firmware für ein landwirtschaftliches Autosteer-Lenksteuergerät auf Basis des **LilyGO T-ETH-Lite-S3** (ESP32-S3-WROOM-1 + W5500 Ethernet).

Das Steuergerät verbindet sich über Ethernet/UDP mit **AgOpenGPS** (bzw. AgIO auf dem Tablet) und steuert einen hydraulischen oder elektrischen Lenkaktuator. Es liest zwei GNSS-Module (Hauptposition + Heading), eine IMU, einen Lenkwinkelsensor und überwacht einen Safety-Eingang.

---

## Quick Start (VS Code + PlatformIO)

```bash
# 1. Projekt klonen
git clone <repo-url> ag-steer
cd ag-steer

# 2. VS Code öffnen (PlatformIO Extension muss installiert sein)
code .

# 3. Firmware kompilieren
#    Menu: PIO → Build

# 4. Firmware flashen (ESP32-S3 per USB angeschlossen)
#    Menu: PIO → Upload

# 5. Seriellen Monitor öffnen
#    Menu: PIO → Serial Monitor  (115200 baud)

# 6. PC-Simulation testen (keine Hardware nötig)
cd pc_sim && make run
```

---

## Projektstruktur (PlatformIO)

```
ag-steer/
├── platformio.ini                  # PlatformIO-Konfiguration (ESP32-S3)
├── partitions_ota.csv              # OTA-Partitionstabelle (2 App-Slots + SPIFFS)
├── README.md                       # Diese Datei
│
├── boards/                         # Custom Board-Definition
│   └── lilygo_t_eth_lite_s3.json   #   LilyGO T-ETH-Lite-S3
│
├── lib/                            # Lokale Bibliotheken
│   └── ETHClass2/                  #   LilyGO W5500 SPI-Treiber (Arduino Core < 3.x)
│       ├── ETHClass2.h
│       └── ETHClass2.cpp
│
├── include/                        # Projektweite Header (globaler Include-Pfad)
│   └── hardware_pins.h             #   GPIO-Pin-Definitionen
│
├── src/                            # Firmware-Quellcode (rekursiv kompiliert)
│   ├── main.cpp                    #   Arduino setup()/loop() + FreeRTOS-Tasks
│   ├── hal/                        #   Hardware Abstraction Layer (C-API)
│   │   └── hal.h                   #     Alle HW-Funktionen als pure C-Deklarationen
│   ├── hal_esp32/                  #   ESP32-S3 HAL-Implementierung
│   │   ├── hal_impl.h
│   │   ├── hal_impl.cpp            #     UART, SPI, ETH (W5500), FreeRTOS
│   │   └── sd_ota_esp32.cpp        #     SD-Karte → OTA Firmware-Update
│   └── logic/                      #   Reine C++ Logik (keine Arduino-Header)
│       ├── global_state.h/.cpp     #     NavigationState, Mutex, StateLock
│       ├── aog_udp_protocol.h/.cpp #     AOG-Frame-Strukturen, PGNs, Encoder/Decoder
│       ├── gnss.h/.cpp             #     NMEA-Parser (GGA + RMC)
│       ├── imu.h/.cpp              #     BNO085 SPI-Stub (yaw_rate, roll)
│       ├── steer_angle.h/.cpp      #     Lenkwinkelsensor SPI
│       ├── actuator.h/.cpp         #     Aktuator SPI
│       ├── control.h/.cpp          #     PID-Regler + 200Hz-Control-Loop
│       ├── net.h/.cpp              #     UDP-Kommunikation mit AgIO
│       ├── hw_status.h/.cpp        #     Hardware-Status-Monitoring + PGN 0xDD
│       ├── sd_ota.h                #     OTA Update – öffentliche API
│       └── sd_ota_version.cpp      #     OTA – Versionsvergleich (plattformunabhängig)
│
└── pc_sim/                         # PC-Simulation (nicht von PlatformIO kompiliert)
    ├── Makefile                    #   g++ Build
    ├── main_pc.cpp                 #   Simulations-Hauptprogramm
    └── hal_pc/                     #   PC-HAL (std::chrono, std::mutex, Dummy-Sensoren)
```

---

## Hardware

### Zielplattform

| Bauteil | Beschreibung |
|---------|-------------|
| **MCU** | ESP32-S3-WROOM-1 (dual-core, 240 MHz, 16 MB Flash) |
| **Board** | LilyGO T-ETH-Lite-S3 |
| **Ethernet** | W5500 via SPI (onboard, SPI3_HOST) |
| **GNSS** | 2× UM980 (RTK-Rover, 460800 baud, 8N1) |
| **IMU** | BNO085 (oder kompatibel, SPI) |
| **Lenkwinkelsensor** | SPI-basiert |
| **Aktuator** | SPI-basiert (0–65535 Kommandowert) |
| **Safety** | GPIO4, active LOW (Pull-Up intern) |

### Pin-Belegung

Alle Pins sind zentral in [`include/hardware_pins.h`](include/hardware_pins.h) definiert.

#### SPI Bus 1 – Ethernet / W5500 (onboard, fest verdrahtet)

Treiber: ESP-IDF `ETH`-Klasse (oder LilyGO `ETHClass2` bei Arduino Core < 3.x).
Verwaltet den SPI-Bus intern – kein manueller SPI-Code nötig.

| Signal | GPIO | Arduino-Bus | ESP-IDF | Funktion |
|--------|------|-------------|---------|----------|
| ETH_CS | 9 | — | — | Chip Select W5500 |
| ETH_SCK | 10 | HSPI | SPI3_HOST | SPI-Takt |
| ETH_MISO | 11 | HSPI | SPI3_HOST | Daten W5500 → MCU |
| ETH_MOSI | 12 | HSPI | SPI3_HOST | Daten MCU → W5500 |
| ETH_INT | 13 | — | — | Interrupt W5500 |
| ETH_RST | 14 | — | — | Reset W5500 |

> ⚠️ Diese Pins sind durch das Board-Design festgelegt. **Nicht ändern!**

#### SPI Bus 2 – Sensoren / Aktuator

Treiber: Arduino `SPIClass(FSPI)` = ESP-IDF `SPI2_HOST`.

| Signal | GPIO | Funktion |
|--------|------|----------|
| SENS_SPI_SCK | 35 | SPI-Takt Sensorbus |
| SENS_SPI_MISO | 36 | Daten Sensor → MCU |
| SENS_SPI_MOSI | 37 | Daten MCU → Sensor |
| CS_IMU | 38 | Chip Select IMU (BNO085) |
| CS_STEER_ANG | 39 | Chip Select Lenkwinkelsensor |
| CS_ACT | 40 | Chip Select Aktuator |
| IMU_INT | 41 | Interrupt IMU (BNO085) |

> ⚠️ **Wichtig:** FSPI (= SPI2_HOST) verwenden, NICHT HSPI!
> Auf ESP32-S3 (Arduino Core 2.x) ist HSPI = SPI3_HOST, was vom W5500 belegt wird.
> Gleichzeitiger Zugriff führt zum Absturz (`spi_hal_setup_trans` assert).

#### UARTs – GNSS

| Signal | GPIO | Funktion |
|--------|------|----------|
| GNSS_HEADING_TX | 15 | Heading-GNSS senden |
| GNSS_HEADING_RX | 16 | Heading-GNSS empfangen |
| GNSS_MAIN_TX | 17 | Haupt-GNSS senden |
| GNSS_MAIN_RX | 18 | Haupt-GNSS empfangen |

Beide UARTs: **460800 Baud, 8N1**

#### Sonstige

| Signal | GPIO | Funktion |
|--------|------|----------|
| SAFETY_IN | 4 | Safety-Eingang, **active LOW** (Pull-Up intern) |

### GPIO-Übersicht am Header (physisch gruppiert)

```
GPIO   9  10  11  12  13  14        W5500 Ethernet (onboard, fest)
       CS  SCK MISO MOSI INT RST

GPIO  15  16  17  18                GNSS UARTs
       TX2 RX2 TX1 RX1

GPIO  35  36  37  38  39  40  41    Sensor-SPI + CS + INT
       SCK MISO MOSI CS1 CS2 CS3 INT

GPIO   5   6   7   42                SD-Karte (OTA)
       MISO MOSI SCK CS

GPIO   4                            Safety (Pull-Up Input)
```

### SD-Karte (OTA Firmware Update)

Treiber: Arduino `SD.h` über `SPIClass(FSPI)` = ESP-IDF `SPI2_HOST`.
**Wird nur beim Firmware-Update verwendet** – teilt sich den Bus temporär mit den Sensoren.

| Signal | GPIO | Funktion |
|--------|------|----------|
| SD_SPI_MISO | 5 | Daten SD → MCU |
| SD_SPI_MOSI | 6 | Daten MCU → SD |
| SD_SPI_SCK | 7 | SPI-Takt SD-Karte |
| SD_CS | 42 | Chip Select SD-Karte |

> ⚠️ **GPIO 5/6/7:** Auf LilyGO T-ETH-Lite-S3 sind diese GPIOs frei verfügbar.
> Falls deine Board-Revision hier Strapping-Pins oder USB verwendet, passe die
> Pins in [`include/hardware_pins.h`](include/hardware_pins.h) an.

#### SPI Bus 3 – SD-Karte (zeitweilig, teilt SPI2_HOST)

Die SD-Karte nutzt den selben SPI2_HOST (FSPI) wie der Sensorbus.
Während normaler Laufzeit gehört der Bus den Sensoren. Nur beim OTA-Update
wird der Sensorbus per `hal_sensor_spi_deinit()` freigegeben, der Bus mit
SD-Pins neu initialisiert, und nach dem Update (oder Fehler) per
`hal_sensor_spi_reinit()` wiederhergestellt.

### Bus-Topologie

```
  ESP32-S3

  ┌─── SPI2_HOST (FSPI) ─────────────────────────────────┐
  │  Normalbetrieb:  SCK=35  MISO=36  MOSI=37            │
  │  OTA-Update:     SCK=7   MISO=5   MOSI=6             │
  │                                                        │
  │  CS=38 → BNO085 (IMU)                                 │
  │  CS=39 → Lenkwinkelsensor                              │
  │  CS=40 → Aktuator                                     │
  │  CS=42 → SD-Karte (nur bei OTA)                       │
  └────────────────────────────────────────────────────────┘

  ┌─── SPI3_HOST (HSPI) ────┐
  │  SCK=10  MISO=11  MOSI=12 │   (managed by ETH driver)
  │  CS=9   INT=13   RST=14   │
  └──────────┬────────────────┘
         ┌────┴────┐
         │  W5500  │
         │Ethernet │
         └────┬────┘
              │ RJ45
           ───┴─── Netzwerk (AgIO Tablet)

  ┌─── UART1 ──┐   ┌─── UART2 ───┐
  │ RX=18 TX=17│   │ RX=16 TX=15 │
  └─────┬──────┘   └──────┬──────┘
   UM980 #1            UM980 #2
   (Position)          (Heading)

  Safety: GPIO4 (active LOW, Pull-Up)
```

---

## Software-Architektur

### Schichtenmodell

```
┌─────────────────────────────────────────────────┐
│                    src/                          │  Entry Point
│             main.cpp (Arduino setup/loop)       │  FreeRTOS-Tasks
├─────────────────────────────────────────────────┤
│                 src/logic/                       │
│  gnss · imu · steer_angle · actuator · control  │  Reine C++ Logik
│  net · hw_status · aog_udp_protocol · state     │  Keine HW-Abhängigkeit
├─────────────────────────────────────────────────┤
│                src/hal/ (Schnittstelle)          │  C-API
│  hal_millis · hal_gnss_* · hal_imu_* · hal_net  │
├─────────────────────────────────────────────────┤
│          src/hal_esp32/  Implementierung        │  Arduino/FreeRTOS
│  ETH.h / ETHClass2, WiFiUDP, SPI, Semaphore     │
├─────────────────────────────────────────────────┤
│            include/hardware_pins.h               │  Pin-Definitionen
└─────────────────────────────────────────────────┘
```

### Ethernet-Treiber

Der W5500 wird über den ESP-IDF `ETH`-Treiber angesteuert (kein Arduino Ethernet/Ethernet3!).

| Arduino ESP32 Core | Treiber | Bibliothek |
|---------------------|---------|------------|
| >= 3.0.0 | Nativer `ETH` | `<ETH.h>` (built-in) |
| < 3.0.0 | LilyGO `ETHClass2` | `lib/ETHClass2/` (lokal) |

Der Treiber wird automatisch via `#if ESP_ARDUINO_VERSION` ausgewählt.
Link-Status wird über `WiFi.onEvent()` (ARDUINO_EVENT_ETH_CONNECTED / GOT_IP) trackt.

### FreeRTOS-Tasks

| Task | Core | Rate | Funktion |
|------|------|------|----------|
| **controlTask** | 1 | 200 Hz | Safety → IMU → Lenkwinkel → PID → Aktuator |
| **commTask** | 0 | 100 Hz | GNSS → UDP Rx → UDP Tx → HW-Status |

#### controlTask – Core 1 – 200 Hz (5 ms)

```
1. Safety prüfen (GPIO4)  → bei LOW: PID reset, Aktuator=0
2. IMU lesen (SPI2/FSPI, CS=GPIO38)  → yaw_rate_dps, roll_deg
3. Lenkwinkel lesen (SPI2/FSPI, CS=GPIO39)  → steer_angle_deg
4. PID berechnen  Fehler = desiredSteerAngleDeg − g_nav.steer_angle_deg
5. Aktuator ansteuern (SPI2/FSPI, CS=GPIO40)  → uint16_t Kommandowert
```

#### commTask – Core 0 – 100 Hz (10 ms)

```
1. GNSS MAIN pollen (UART1, RX=18)  → GGA, RMC → lat, lon, alt, sog, cog, fix
2. GNSS HEADING pollen (UART2, RX=16)  → RMC → heading_deg
3. Netzwerk empfangen (UDP)  → Hello, Scan, SubnetChange, SteerDataIn
4. AOG-Frames senden (@ 10 Hz)  → GPS Main Out + Steer Status Out
5. HW-Status überwachen (@ 1 Hz)  → PGN 0xDD Hardware Messages
```

### Globaler Zustand

Definiert in [`src/logic/global_state.h`](src/logic/global_state.h):

```cpp
struct NavigationState {
    double  lat_deg;         float alt_m;       float heading_deg;
    double  lon_deg;         float sog_mps;     float roll_deg;
    float   cog_deg;         float yaw_rate_dps;
    uint8_t fix_quality;     float steer_angle_deg;
    bool    safety_ok;       uint32_t timestamp_ms;
};

extern NavigationState g_nav;
extern volatile float desiredSteerAngleDeg;  // Sollwinkel von AgIO
```

**Thread-Sicherheit:** RAII-Wrapper `StateLock` schützt alle Zugriffe via Mutex.

---

## Komponenten

### GNSS ([`src/logic/gnss.h`](src/logic/gnss.h))

| Rolle | UART | Pins | Funktion |
|-------|------|------|----------|
| GNSS_MAIN | UART1 | RX=18, TX=17 | Hauptposition, RTK-Rover |
| GNSS_HEADING | UART2 | RX=16, TX=15 | Heading-Quelle |

NMEA-Parser: **GGA** (lat, lon, alt, fix), **RMC** (sog, cog). 460800 Baud.

### IMU ([`src/logic/imu.h`](src/logic/imu.h))

BNO085 über SPI2/FSPI (CS=GPIO38, INT=GPIO41). Liest `yaw_rate_dps` und `roll_deg`. SH-2 Protokoll TODO.

### Lenkwinkelsensor ([`src/logic/steer_angle.h`](src/logic/steer_angle.h))

SPI2/FSPI (CS=GPIO39). `steerAngleReadDeg()` → Winkel in Grad. Sensorprotokoll TODO.

### Aktuator ([`src/logic/actuator.h`](src/logic/actuator.h))

SPI2/FSPI (CS=GPIO40). `actuatorWriteCommand(uint16_t cmd)`. Bei `safety_ok == false` → cmd = 0.

### PID-Regler ([`src/logic/control.h`](src/logic/control.h))

| Parameter | Standard |
|-----------|----------|
| Kp | 1.0 |
| Ki | 0.0 |
| Kd | 0.01 |
| Output | 0 – 65535 |

Anti-Windup aktiv, Fehler auf [-180°, +180°] gewrappt.

### Netzwerk ([`src/logic/net.h`](src/logic/net.h))

- **Sendet @ 10 Hz:** GPS Main Out (Port 5124), Steer Status Out (Port 5126)
- **Empfängt:** Hello (→ Reply), Scan (→ Reply), SubnetChange (→ IP Update), SteerDataIn (→ Sollwinkel)

### Hardware-Status ([`src/logic/hw_status.h`](src/logic/hw_status.h))

Überwacht 7 Subsysteme und sendet **PGN 0xDD Hardware Messages** an AgIO:
Ethernet, GNSS Main, GNSS Heading, IMU, Lenkwinkelsensor, Aktuator, Safety.
Fehler werden mit Debounce und Rate-Limiting gemeldet (Farbe: grün/gelb/rot/blau).

### HAL ([`src/hal/hal.h`](src/hal/hal.h))

Reine C-API mit Implementierungen in `src/hal_esp32/` und `pc_sim/hal_pc/`. Keine Arduino-Header in der Schnittstelle.

---

## AOG/AgIO UDP-Protokoll

### Frame-Format

```
Offset: 0    1    2    3    4    5...n-2    n-1
       ┌────┬────┬────┬────┬────┬────────┬────┐
       │0x80│0x81│Src │PGN │Len │ Payload│CRC │
       └────┴────┴────┴────┴────┴────────┴────┘

CRC = Low-Byte der Summe von Byte2 bis Byte(n-2)
```

### Unterstützte PGNs

| PGN | Name | Richtung | Payload | Status |
|-----|------|----------|---------|--------|
| 200 | Hello from AgIO | AgIO → Modul | 4 B | Decoder ✓ |
| 201 | Subnet Change | AgIO → Modul | 5 B | Decoder ✓ |
| 202 | Scan Request | AgIO → Modul | 3 B | Decoder ✓ |
| 203 | Subnet Reply | Modul → AgIO | 7 B | Encoder ✓ |
| **254** | **Steer Data In** | **AgIO → Steer** | **8 B** | **Decoder ✓** |
| **253** | **Steer Status Out** | **Steer → AgIO** | **8 B** | **Encoder ✓** |
| **250** | **From Autosteer 2** | **Steer → AgIO** | **8 B** | **Encoder ✓** |
| **214** | **GPS Main Out** | **GPS → AgIO** | **51 B** | **Encoder ✓** |
| **221** | **Hardware Message** | **bidirektional** | **variabel** | **Enc+Dec ✓** |

### OTA Firmware-Update ([`src/logic/sd_ota.h`](src/logic/sd_ota.h))

Firmware-Update von einer SD-Karte in die inaktive OTA-Partition des Flash.

**Funktionsweise:**
1. Beim Boot prüft die Firmware, ob `/firmware.bin` (oder `/update.bin`) auf der SD-Karte liegt.
2. Optional: Version prüfen – nur updaten, wenn die neue Firmware neuer ist als die aktuelle.
3. Die BIN-Datei wird blockweise (4 KB) von SD gelesen und per ESP32 `Update` API in den Flash geschrieben.
4. Nach erfolgreichem Schreiben wird die OTA-Partition als Boot-Partition gesetzt und der ESP32 neu gestartet.
5. Bei jedem Fehler bleibt die alte Firmware aktiv.

**Voraussetzungen:**
- SD-Karte per SPI angeschlossen (MISO=5, MOSI=6, SCLK=7, CS=42)
- OTA-Partitionstabelle aktiv (`partitions_ota.csv`, zwei App-Slots)
- Firmware-Datei (`firmware.bin`) ist eine gültige ESP32-Arduino BIN-Datei

**Dateien auf der SD-Karte:**

| Datei | Pflicht | Inhalt |
|-------|---------|--------|
| `/firmware.bin` | Ja¹ | ESP32 BIN-Datei (max 3 MB) |
| `/update.bin` | Nein² | Alternative Firmware-Datei |
| `/firmware.version` | Nein | Versionsstring, z.B. `1.2.3` |

¹ Wird zuerst gesucht. Fehlt sie, wird `/update.bin` verwendet.² Nur wenn `/firmware.bin` nicht existiert.

**Versionsprüfung (optional):**
- `/firmware.version` enthält z.B. `1.2.3` (nur Text, mit oder ohne Zeilenumbruch)
- Die aktuelle Firmware-Version ist über `FIRMWARE_VERSION` in `platformio.ini` definiert.
- Das Update wird **nur** ausgeführt, wenn die SD-Version **neuer** ist als die aktuell laufende.
- Fehlt die Datei, wird das Update ohne Versionsprüfung ausgeführt.

**Serielles Log (Beispiel):**
```
[         0] Main: firmware v0.1.0
[       150] OTA: checking for firmware update on SD card...
[       160] OTA: running from partition 'ota_0' (0x20000, subtype=0x10)
[       180] OTA: SD card mounted OK
[       185] OTA: found /firmware.bin (1048576 bytes)
[       190] OTA: SD version = 0.2.0, current version = 0.1.0  (NEWER)
[       195] OTA: firmware update available on SD card
[       200] Main: firmware update detected on SD card – starting update
[       205] OTA: ===== STARTING FIRMWARE UPDATE FROM SD =====
[       215] OTA: phase 1 – releasing sensor SPI bus...
[       230] OTA: SD card mounted OK
[       240] OTA: phase 2 – opening firmware file...
[       250] OTA: /firmware.bin opened, size = 1048576 bytes
[       260] OTA: phase 3 – starting OTA write to flash...
[       270] OTA: OTA partition initialised (1048576 bytes)
[       275] OTA: phase 4 – writing firmware to flash...
[      1800] OTA:  10%  (104857 / 1048576 bytes, 543 KB/s)
[      3200] OTA:  20%  (209714 / 1048576 bytes, 576 KB/s)
[      ...  ]
[      8200] OTA: 100%  (1048576 / 1048576 bytes, 589 KB/s)
[      8250] OTA: phase 5 – validating and finalising...
[      8400] OTA: ===== UPDATE SUCCESSFUL =====
[      8400] OTA: wrote 1048576 bytes in 8150 ms (125 KB/s)
[      8400] OTA: rebooting into new firmware in 2 seconds...
[     10400] OTA: RESTARTING NOW
```

**Update-Ablauf deaktivieren:**
Einfach die `firmware.bin` von der SD-Karte löschen oder die SD-Karte abziehen.
Die Firmware bootet dann normal ohne Update-Versuch.

---

## Partitionstabelle (OTA)

Die Datei [`partitions_ota.csv`](partitions_ota.csv) definiert die Flash-Aufteilung:

| Partition | Typ | Offset | Größe | Beschreibung |
|-----------|-----|--------|-------|--------------|
| nvs | data/nvs | 0x9000 | 24 KB | Non-Volatile Storage (Kalibrierung etc.) |
| otadata | data/ota | 0xF000 | 8 KB | Boot-Slot-Selektor |
| phy_init | data/phy | 0x11000 | 4 KB | PHY-Kalibrierungsdaten |
| **ota_0** | **app/ota_0** | **0x20000** | **3.75 MB** | **Erster App-Slot** |
| **ota_1** | **app/ota_1** | **0x3E0000** | **3.75 MB** | **Zweiter App-Slot (OTA-Ziel)** |
| spiffs | data/spiffs | 0x7A0000 | ~8.25 MB | Dateisystem (zukünftig) |

Der ESP32-Bootloader liest `otadata` um zu entscheiden, ob von `ota_0` oder `ota_1` gebootet wird. Bei einem fehlgeschlagenen Update (Power-Loss, Crash während des Schreibens) bleibt der alte Slot aktiv – **eingebautes Rollback**.

> ⚠️ **Wichtig:** Nach dem Wechsel auf `partitions_ota.csv` muss die Firmware einmalig per USB geflasht werden (es existiert noch kein OTA-Slot mit gültiger Firmware). Danach können weitere Updates über SD-Karte erfolgen.

---

## Bauen & Testen

### ESP32-Firmware (PlatformIO)

```bash
# Kompilieren
pio run

# Flashen (ESP32-S3 per USB)
pio run --target upload

# Serieller Monitor
pio device monitor -b 115200
```

**Voraussetzungen:**
- VS Code mit PlatformIO Extension
- ESP32-S3 per USB verbunden
- Keine externe Ethernet-Bibliothek nötig (ESP-IDF ETH-Treiber wird verwendet)

### PC-Simulation

```bash
cd pc_sim
make          # kompilieren
make run      # ausführen (1s Simulation mit Frame-Verifikation)
make clean    # aufräumen
```

**Erwartete Ergebnisse:**
```
GPS Main Out (PGN 0xD6):  57 bytes  CRC: OK ✓
Steer Status (PGN 0xFD):  14 bytes  CRC: OK ✓
Hello Reply Steer:         8 bytes  CRC: OK ✓
```

---

## Konfiguration

### IP-Konfiguration

Standard-Werte in [`src/hal_esp32/hal_impl.cpp`](src/hal_esp32/hal_impl.cpp):

| Parameter | Standard |
|-----------|----------|
| Lokale IP | 192.168.1.70 |
| Subnetz | 255.255.255.0 |
| Gateway | 192.168.1.1 |
| DNS | 8.8.8.8 |
| Ziel-IP | 192.168.1.255 (Broadcast) |

Wird automatisch durch **Subnet Change** (PGN 201) von AgIO aktualisiert.

### PID-Parameter

In [`src/logic/control.cpp`](src/logic/control.cpp) – `controlInit()`:

```cpp
pidInit(&s_steer_pid, 1.0f, 0.0f, 0.01f, 0.0f, 65535.0f);
```

---

## Bekannte Probleme

| Problem | Status | Lösung |
|---------|--------|--------|
| PSRAM ID read error | ⚠️ Nicht kritisch | Board-Typvariante; `CONFIG_SPIRAM_IGNORE_NOTFOUND=y` unterdrückt die Meldung. System läuft trotzdem. |
| GNSS Main Warnung | ℹ️ Normal ohne GNSS | Verschwindet sobald ein GNSS-Modul an UART1 angeschlossen ist. |

---

## TODO

| Bereich | Status |
|---------|--------|
| BNO085 SH-2 SPI-Protokoll | 🔲 TODO |
| Echtes Lenkwinkelsensor-Protokoll | 🔲 TODO |
| Echtes Aktuator-Protokoll | 🔲 TODO |
| Dual-Antenna Heading-Fusion | 🔲 TODO |
| SteerSettings In (PGN 252) empfangen | 🔲 TODO |
| SteerConfig In (PGN 251) empfangen | 🔲 TODO |
| HDOP/Satelliten aus GGA | 🔲 TODO |
| Subnet-Change aktualisiert auch die lokale IP | 🔲 TODO |
| OTA-Update (SD-Karte) | ✅ Implementiert |
