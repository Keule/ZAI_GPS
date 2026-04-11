# AgSteer – Autosteer-Steuergerät für AgOpenGPS / AgIO

> Embedded-Firmware für ein landwirtschaftliches Autosteer-Lenksteuergerät auf Basis des **LilyGO T-ETH-Lite-S3** (ESP32-S3-WROOM-1 + W5500 Ethernet).

Das Steuergerät verbindet sich über Ethernet/UDP mit **AgOpenGPS** (bzw. AgIO auf dem Tablet) und steuert einen hydraulischen oder elektrischen Lenkaktuator. Es liest zwei GNSS-Module (Hauptposition + Heading), eine IMU, einen Lenkwinkelsensor (ADS1118 ADC) und überwacht einen Safety-Eingang.

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
│   ├── ads1118/                    #   ADS1118 16-Bit ADC Treiber
│   │   ├── ads1118.h
│   │   ├── ads1118.cpp
│   │   └── library.json
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
│   │   ├── sd_logger_esp32.cpp     #     SD-Karte Datenlogger
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
| **MCU** | ESP32-S3-WROOM-1 (dual-core, 240 MHz, 16 MB Flash, 8 MB PSRAM) |
| **Board** | LilyGO T-ETH-Lite-S3 |
| **Ethernet** | W5500 via SPI (onboard, SPI3_HOST) |
| **GNSS** | 2× UM980 (RTK-Rover, 460800 baud, 8N1) |
| **IMU** | BNO085 (oder kompatibel, SPI) |
| **Lenkwinkelsensor** | ADS1118 16-Bit ADC + Potentiometer (SPI) |
| **Aktuator** | SPI-basiert (0–65535 Kommandowert) |
| **Safety** | GPIO4, active LOW (Pull-Up intern) |

### Pin-Belegung

Alle Pins sind zentral in [`include/hardware_pins.h`](include/hardware_pins.h) definiert.

#### SPI Bus 1 – Ethernet / W5500 (onboard, fest verdrahtet)

Treiber: ESP-IDF `ETH`-Klasse (oder LilyGO `ETHClass2` bei Arduino Core < 3.x).
Verwaltet den SPI-Bus intern – kein manueller SPI-Code nötig.

| Signal | GPIO | ESP-IDF | Funktion |
|--------|------|---------|----------|
| ETH_CS | 9 | — | Chip Select W5500 |
| ETH_SCK | 10 | SPI3_HOST | SPI-Takt |
| ETH_MISO | 11 | SPI3_HOST | Daten W5500 → MCU |
| ETH_MOSI | 12 | SPI3_HOST | Daten MCU → W5500 |
| ETH_INT | 13 | — | Interrupt W5500 |
| ETH_RST | 14 | — | Reset W5500 |

> ⚠️ Diese Pins sind durch das Board-Design festgelegt. **Nicht ändern!**

#### SPI Bus 2 – Sensoren / Aktuator (FSPI = SPI2_HOST)

Treiber: Arduino `SPIClass(FSPI)` = ESP-IDF `SPI2_HOST`.
Dedizierter SPI-Bus für ADS1118, IMU und Aktuator.

| Signal | GPIO | Funktion |
|--------|------|----------|
| SENS_SPI_SCK | 16 | SPI-Takt Sensorbus |
| SENS_SPI_MISO | 15 | Daten Sensor → MCU |
| SENS_SPI_MOSI | 17 | Daten MCU → Sensor |
| CS_STEER_ANG | 18 | Chip Select ADS1118 (Lenkwinkel) |
| CS_IMU | 38 | Chip Select IMU (BNO085) |
| CS_ACT | 40 | Chip Select Aktuator |

#### ADS1118 Lenkwinkelsensor – Verdrahtung

```
ADS1118 Modul          ESP32-S3 (T-ETH-Lite-S3)
─────────────          ────────────────────────
VDD            →       3.3V
GND            →       GND
DOUT           →       GPIO 15  (MISO)
DIN            →       GPIO 17  (MOSI)
SCLK           →       GPIO 16  (SCK)
CS             →       GPIO 18
```

> ⚠️ **Wichtig:** FSPI (= SPI2_HOST) verwenden, NICHT HSPI!
> Auf ESP32-S3 (Arduino Core 2.x) ist HSPI = SPI3_HOST, was vom W5500 belegt wird.

#### UARTs – GNSS

| Signal | GPIO | Funktion |
|--------|------|----------|
| GNSS_HEADING_TX | 44 | Heading-GNSS senden |
| GNSS_HEADING_RX | 43 | Heading-GNSS empfangen |
| GNSS_MAIN_TX | 46 | Haupt-GNSS senden |
| GNSS_MAIN_RX | 45 | Haupt-GNSS empfangen |

Beide UARTs: **460800 Baud, 8N1**

#### Sonstige

| Signal | GPIO | Funktion |
|--------|------|----------|
| SAFETY_IN | 4 | Safety-Eingang, **active LOW** (Pull-Up intern) |
| IMU_INT | 48 | IMU Interrupt (BNO085) |
| LOG_SWITCH_PIN | 47 | Logging-Schalter, **active LOW** |

### GPIO-Übersicht am Header (nach Funktion gruppiert)

```
GPIO   9  10  11  12  13  14        W5500 Ethernet (onboard, fest)
       CS  SCK MISO MOSI INT RST

GPIO  15  16  17  18  38  40        Sensor-SPI (FSPI/SPI2_HOST) + CS
       MISO SCK MOSI CS  CS1 CS2         ADS1118  IMU    Aktuator

GPIO  43  44  45  46                 GNSS UARTs
       RX2 TX2 RX1 TX1                  Heading   Main

GPIO   5   6   7   42                SD-Karte (OTA only, FSPI)
       MISO MOSI SCK CS

GPIO   4   47  48                     Sonstige
       Safety  LogSw  IMU_INT
```

### SD-Karte (OTA Firmware Update)

Treiber: Arduino `SD.h` über `SPIClass(FSPI)` = ESP-IDF `SPI2_HOST`.
**Wird nur beim Firmware-Update verwendet** – eigene Pins, FSPI wird temporär umkonfiguriert.

| Signal | GPIO | Funktion |
|--------|------|----------|
| SD_SPI_SCK | 7 | SPI-Takt SD-Karte |
| SD_SPI_MISO | 5 | Daten SD → MCU |
| SD_SPI_MOSI | 6 | Daten MCU → SD |
| SD_CS | 42 | Chip Select SD-Karte |

Die SD-Karte nutzt den **gleichen** SPI-Peripher (FSPI/SPI2_HOST) wie der Sensorbus, aber mit **anderen Pins**. Während normaler Laufzeit gehört FSPI den Sensoren (GPIO 15/16/17). Nur beim OTA-Update wird FSPI per `hal_sensor_spi_deinit()` freigegeben, mit SD-Pins (5/6/7) neu initialisiert, und nach dem Update per `hal_sensor_spi_reinit()` wiederhergestellt.

### Bus-Topologie

```
  ESP32-S3

  ┌─── FSPI (SPI2_HOST) ───────────────────────────┐
  │  Sensorbus:  SCK=16  MISO=15  MOSI=17           │
  │                                                 │
  │  CS=18 → ADS1118 (Lenkwinkel ADC)               │
  │  CS=38 → BNO085 (IMU)                           │
  │  CS=40 → Aktuator                               │
  │                                                 │
  │  SD-Karte (OTA only, gleicher Bus, andere Pins): │
  │         SCK=7  MISO=5  MOSI=6  CS=42            │
  └─────────────────────────────────────────────────┘

  ┌─── SPI3_HOST ────────────┐
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
  │ RX=45 TX=46│   │ RX=43 TX=44 │
  └─────┬──────┘   └──────┬──────┘
   UM980 #1            UM980 #2
   (Position)          (Heading)

  Safety: GPIO4 (active LOW, Pull-Up)
  IMU_INT: GPIO48
  LogSwitch: GPIO47 (active LOW)
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

### ADS1118 Library ([`lib/ads1118/`](lib/ads1118/))

Lokale ADS1118 Treiberbibliothek mit folgenden Features:

| Feature | Beschreibung |
|---------|-------------|
| SPI-Protokoll | 16-Bit simultan Config+Data (korrekt lt. Datasheet SLASB73) |
| SPI-Mode | Auto-Detection (probiert Mode0 und Mode1) |
| Bit-Inversion | Auto-Detection für Module mit invertierenden Level-Shiftern |
| DOUT-Test | Crosstalk- und Floating-Erkennung |
| Shared-Bus | Deselect-Callback für andere CS-Pins am selben Bus |
| Non-blocking | `readLoop()` für 200 Hz Control-Loop (128 SPS = 7.8ms Konvertierung) |
| Temperatur | Interner Temperatursensor auslesbar |

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
3. Lenkwinkel lesen (SPI2/FSPI, CS=GPIO18)  → ADS1118 AIN0 → steer_angle_deg
4. PID berechnen  Fehler = desiredSteerAngleDeg − g_nav.steer_angle_deg
5. Aktuator ansteuern (SPI2/FSPI, CS=GPIO40)  → uint16_t Kommandowert
```

#### commTask – Core 0 – 100 Hz (10 ms)

```
1. GNSS MAIN pollen (UART1, RX=45)  → GGA, RMC → lat, lon, alt, sog, cog, fix
2. GNSS HEADING pollen (UART2, RX=43)  → RMC → heading_deg
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
| GNSS_MAIN | UART1 | RX=45, TX=46 | Hauptposition, RTK-Rover |
| GNSS_HEADING | UART2 | RX=43, TX=44 | Heading-Quelle |

NMEA-Parser: **GGA** (lat, lon, alt, fix), **RMC** (sog, cog). 460800 Baud.

### IMU ([`src/logic/imu.h`](src/logic/imu.h))

BNO085 über SPI2/FSPI (CS=GPIO38, INT=GPIO48). Liest `yaw_rate_dps` und `roll_deg`. SH-2 Protokoll TODO.

### Lenkwinkelsensor ([`src/logic/steer_angle.h`](src/logic/steer_angle.h))

ADS1118 16-Bit ADC über SPI2/FSPI (CS=GPIO18). Potentiometer an AIN0.
0-3.3V → 0 bis ~26880 raw → -45° bis +45°.
Automatische SPI-Mode und Bit-Inversion Detection.

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

> ⚠️ **Wichtig:** Nach dem Wechsel auf `partitions_ota.csv` muss die Firmware einmalig per USB geflasht werden.

---

## Bauen & Testen

### ESP32-Firmware (PlatformIO)

```bash
pio run                          # Kompilieren
pio run --target upload          # Flashen
pio device monitor -b 115200     # Serieller Monitor
```

### PC-Simulation

```bash
cd pc_sim
make          # kompilieren
make run      # ausführen (1s Simulation mit Frame-Verifikation)
make clean    # aufräumen
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

### PID-Parameter

In [`src/logic/control.cpp`](src/logic/control.cpp) – `controlInit()`:

```cpp
pidInit(&s_steer_pid, 1.0f, 0.0f, 0.01f, 0.0f, 65535.0f);
```

---

## Bekannte Probleme

| Problem | Status | Lösung |
|---------|--------|--------|
| PSRAM ID read error | ⚠️ Nicht kritisch | Board-Typvariante; `CONFIG_SPIRAM_IGNORE_NOTFOUND=y` unterdrückt die Meldung. |
| GNSS Main Warnung | ℹ️ Normal ohne GNSS | Verschwindet sobald ein GNSS-Modul an UART1 angeschlossen ist. |
| ADS1118 Bit-Inversion | ℹ️ Automatisch | Cheap Module mit Transistor-Level-Shiftern invertieren DOUT. Wird auto-kompensiert. |

---

## TODO

| Bereich | Status |
|---------|--------|
| BNO085 SH-2 SPI-Protokoll | 🔲 TODO |
| Echtes Aktuator-Protokoll | 🔲 TODO |
| Dual-Antenna Heading-Fusion | 🔲 TODO |
| SteerSettings In (PGN 252) empfangen | 🔲 TODO |
| SteerConfig In (PGN 251) empfangen | 🔲 TODO |
| HDOP/Satelliten aus GGA | 🔲 TODO |
| Subnet-Change aktualisiert auch die lokale IP | 🔲 TODO |
| OTA-Update (SD-Karte) | ✅ Implementiert |
| ADS1118 Auto-Detection | ✅ Implementiert |
