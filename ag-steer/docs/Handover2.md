# AgSteer ESP32-S3 Firmware — Projekt-Handover

> **Zuletzt aktualisiert:** 2026-04-17
> **Repo:** `github.com/Keule/ZAI_GPS` (Branch: `agent/zai-work-v2` oder neuer)
> **Firmware:** `ag-steer/steer-controller/`

---

## 1. Projektüberblick

Ziel: Eigenentwicklung eines **AgOpenGPS Autosteer-Moduls** basierend auf dem
**LilyGO T-ETH-Lite-S3** (ESP32-S3-WROOM-1 + W5500 Ethernet).

Die Firmware empfängt Lenkbefehle von AgOpenGPS (via AgIO) über Ethernet/UDP,
steuert einen Lenkaktuator über SPI an und sendet Sensorwerte (Lenkwinkel,
IMU, Schalter) zurück.

### Hardware

| Komponente | Chip/Modul | Interface |
|------------|-----------|-----------|
| MCU | ESP32-S3-WROOM-1 | — |
| Ethernet | W5500 | SPI (SPI2_HOST) |
| Lenkwinkelsensor | ADS1118 (16-Bit ADC) | SPI (SPI2_HOST, shared) |
| Lenkaktuator | PWM-Driver | SPI (SPI2_HOST, shared) |
| IMU | BNO085 (9-Achsen) | SPI (SPI2_HOST, shared) |
| SD-Card | — | SPI (SPI2_HOST, shared, temporär) |
| Safety-Schalter | GPIO | Digital Input |
| Logger-Schalter | GPIO 47 | Digital Input |
| USB | USB-OTG (CDC Serial) | Debug-Output |

Alle SPI-Geräte teilen sich **SPI2_HOST**. Die Sensor-Initialisierung (`hal_sensor_spi_init`)
konfiguriert alle Chip-Select-Leitungen. Die SD-Card benötigt temporär exklusiven
Zugriff auf den Bus (`hal_sensor_spi_deinit()` / `hal_sensor_spi_reinit()`).

---

## 2. Architektur

```
┌─────────────────────────────────────────────────────┐
│                    main.cpp                          │
│  setup(): HAL init → modules → calibration → tasks  │
│  loop(): watchdog reset, STAT logging (idle task)    │
├──────────────┬──────────────────────────────────────┤
│  Core 1      │  Core 0                               │
│  controlTask │  commTask                             │
│  200 Hz      │  100 Hz                               │
│              │                                       │
│  controlStep │  netPollReceive()                     │
│  sdLoggerRec │  netSendAogFrames()                   │
│              │  modulesUpdateStatus() @ 1 Hz         │
│              │  hwStatusUpdate()     @ 1 Hz         │
├──────────────┴──────────────────────────────────────┤
│              Logic Layer                             │
│  control.cpp │ net.cpp │ modules.cpp │ pgn_codec.cpp │
│  imu.cpp     │ hw_status.cpp │ sd_logger.cpp        │
├─────────────────────────────────────────────────────┤
│              HAL Layer                               │
│  hal.h (Interface) │ hal_impl.cpp (ESP32-S3)        │
├─────────────────────────────────────────────────────┤
│              Hardware                                │
│  W5500 │ ADS1118 │ Actuator │ BNO085 │ SD │ GPIO   │
└─────────────────────────────────────────────────────┘
```

### Dateistruktur

```
ag-steer/steer-controller/src/
├── main.cpp                          # Einstiegspunkt, Tasks, Loop
├── hal/
│   └── hal.h                         # HAL-Interface (C linkage)
├── hal_esp32/
│   ├── hal_impl.cpp                  # ESP32-S3 HAL-Implementierung
│   ├── hal_impl.h                    # HAL-spezifische Konstanten
│   ├── sd_ota_esp32.cpp              # OTA Firmware-Update via SD
│   └── sd_logger_esp32.cpp           # SD-Card Logger Task
└── logic/
    ├── pgn_types.h                   # PGN-Konstanten, Structs (single source of truth)
    ├── pgn_registry.h                # PGN-Tabelle, Lookup-Funktionen
    ├── pgn_codec.h / .cpp            # Frame-Builder/Validator, Encoder/Decoder
    ├── global_state.h / .cpp         # Gemeinsamer State mit Mutex-Schutz
    ├── control.h / .cpp              # PID-Regler, controlStep(), controlInit()
    ├── net.h / .cpp                  # UDP-Empfang/Sende, PGN-Dispatch
    ├── modules.h / .cpp              # Modul-Registry, Hardware-Detection, Hello/Reply
    ├── imu.h / .cpp                  # IMU-Abstraktion
    ├── steer_angle.h / .cpp          # Lenkwinkelsensor
    ├── actuator.h / .cpp             # Aktuatorsteuerung
    ├── hw_status.h / .cpp            # Hardware-Status-Monitoring
    ├── sd_logger.h / .cpp            # Data-Logger Ringbuffer
    ├── sd_ota.h                      # OTA-Firmware-Update Interface
    ├── sd_ota_version.cpp            # Firmware-Versionsverwaltung
    ├── log_config.h                  # Compile-time Log-Level pro Modul
    ├── log_ext.h / .cpp              # Erweiterte Logging-Hilfsfunktionen
    └── aog_udp_protocol.h            # Legacy-Header (wird nicht mehr aktiv genutzt)
```

### Task-Architektur

| Task | Core | Frequenz | Funktion |
|------|------|----------|----------|
| `controlTask` | 1 | 200 Hz | PID-Regler, Aktuator, Logger-Record |
| `commTask` | 0 | 100 Hz | UDP-Empfang/Sende, HW-Status @ 1 Hz |
| `loop()` | (main) | ~10 Hz | Watchdog-Reset, STAT-Ausgabe alle 5s |

Die Control-Task kompensierte Ausführungszeit (`elapsed < 5 ? delay(5-elapsed)`),
die Comm-Task und loop() nutzen einfache `vTaskDelay()`.

---

## 3. Bisher durchgeführte Arbeiten

### Session 1–3: Initialer Aufbau

- Aufbau der Firmware-Struktur mit HAL-Abstraktion
- PGN-Codec implementiert (Encoder/Decoder für alle relevanten PGNs)
- Modul-Registry mit Hardware-Detection
- Zwei FreeRTOS Tasks (control + comm)
- SD-Card Data-Logger (Ringbuffer → CSV, 10 Hz)
- OTA Firmware-Update via SD-Card
- Lenkwinkel-Kalibrierung (interaktiv über Serial, Werte in NVS persistent)

### Session 4–6: Bugfixes & Stabilität

| Fix | Problem | Lösung |
|-----|---------|--------|
| `hal_log()` ging zu USB CDC statt UART0 | `ESP_LOGI()` geht zu UART0, nicht USB Serial | Zurück zu `Serial.printf()` mit recursive Mutex-Schutz |
| `s_log_mutex` fehlte | USB CDC Panic bei parallelem Zugriff Core 0+1 | Recursive Mutex (`xSemaphoreCreateRecursiveMutexStatic`) wiederhergestellt |
| `controlInit()` nie aufgerufen | PID hatte kp=0, ki=0, kd=0 | `controlInit()` nach `modulesInit()` in `setup()` eingefügt |
| Verschachteltes Git-Repo blockierte Sandbox | `ZAI_GPS/` als Submodule im Index | `git rm --cached ZAI_GPS`, zu `.gitignore` hinzugefügt |
| Discovery-PGNs verworfen | `pgnValidateFrame()` prüfte CRC für PGN 200/201/202 | CRC-Check für Discovery-PGNs übersprungen |
| DBG-Ticks → Hz-Anzeige | Tick-Counter wenig aussagekräftig | Tatsächliche Frequenz in Hz gemessen über Zeitdelta |

### Session 7: CRC-Verhalten Discovery/Management-PGNs

AgIO sendet Discovery-PGNs (200, 201, 202) mit **statischen trailing bytes**
die NICHT dem additiven Checksum-Algorithmus entsprechen. Die AgIO- und ModSim-
Empfangspfade validieren die Checksum für diese PGNs nicht.

**Änderungen in `pgn_codec.cpp`:**
- Neue Funktion `pgnIsDiscovery(pgn, src)` identifiziert Discovery-PGNs
- `pgnValidateFrame()` überspringt CRC-Check für:
  - PGN 0xC8 (200) — Hello from AgIO
  - PGN 0xC9 (201) — Subnet Change
  - PGN 0xCA (202) — Scan Request
  - Hello Replies (PGN == Src): 0x7E, 0x7B, 0x79, 0x78
- **Core AOG PGNs (254, 253, 252, 251, etc.) bleiben vollständig CRC-validiert**
- Self-Test erweitert mit Testfällen für Discovery-PGNs

---

## 4. PGN-Protokoll

### Frame-Format

```
[0x80][0x81][Src][PGN][Len][Payload...][CRC]
```

- **CRC:** Additive 8-Bit Checksumme über Bytes[2..Len-2] (ohne Preamble)
- **Discovery-PGNs:** Keine CRC-Validierung (siehe oben)

### PGNs die wir **empfangen** (von AgIO, Src=0x7F)

| PGN | Name | CRC | Bemerkung |
|-----|------|-----|-----------|
| 0xC8 (200) | Hello from AgIO | ❌ skip | Löst Hello-Reply aus |
| 0xC9 (201) | Subnet Change | ❌ skip | Ändert Ziel-IP |
| 0xCA (202) | Scan Request | ❌ skip | Löst Subnet-Reply aus |
| 0xFE (254) | Steer Data In | ✅ validiert | Geschwindigkeit, Sollwinkel, Schalter |
| 0xFC (252) | Steer Settings In | ✅ validiert | PID-Parameter |
| 0xFB (251) | Steer Config In | ✅ validiert | Hardware-Konfigbits |
| 0xDD (221) | Hardware Message | ✅ validiert | Display-Nachricht |

### PGNs die wir **senden** (an AgIO, Port 9999)

| PGN | Name | Frequenz | CRC |
|-----|------|----------|-----|
| 0xFD (253) | Steer Status Out | 10 Hz | ✅ berechnet |
| 0xFA (250) | From Autosteer 2 | 10 Hz | ✅ berechnet |
| 0x7E (126) | Hello Reply Steer | Bei PGN 200 | ✅ berechnet |
| 0xCB (203) | Subnet Reply | Bei PGN 202 | ✅ berechnet |

### UDP-Ports

| Port | Richtung | Verwendung |
|------|----------|------------|
| 8888 | Empfang (von AgIO) | Alle PGNs von AgIO |
| 9999 | Senden (an AgIO) | PGN 253, 250, Hello, Reply |
| 5126 | Source Port (Steer) | Lokaler Port für ausgehende Pakete |

### AgOpenGPS Discovery-Protokoll

Das Discovery-Protokoll ist ein **eigenständiger Pfad** neben dem Core-AOG-PGN-Protokoll:

```
AgIO ---(PGN 200 Hello)---> Module
AgIO ---(PGN 202 Scan)---> Module
Module --(PGN 203 Reply)-> AgIO
AgIO ---(PGN 201 Subnet)-> Module
```

**Wichtig:** Das Discovery-Protokoll hat eine andere Checksum-Strictness als das
Core-AOG-Protokoll. Die statischen Paketdefinitionen in AgIO (helloFromAgIO,
sendIPToModules, scanModules) enthalten trailing bytes die nicht zum additiven
Checksum passen. Sowohl AgIO als auch ModSim validieren die Checksum auf dem
Discovery-Pfad nicht.

Referenz-Quellcode: `AgIO/Source/Forms/UDP.designer.cs`, `ModSim/Source/Forms/UDP.designer.cs`

---

## 5. Git-Workflow & Sandbox-Regeln

### ⚠️ KRITISCHE REGELN (aus schlechter Erfahrung gelernt)

1. **NIE auf `main` arbeiten.** Immer auf einem Feature-Branch (`agent/zai-work-*`).
2. **NIE cherry-pick, rebase oder stash-pop.** Die Sandbox blockiert bei merge-Konflikten
   komplett — kein Tool-Aufruf geht mehr. Stattdessen: **neuen Branch erstellen und pushen.**
3. **KEINE verschachtelten Git-Repos.** Ein `ZAI_GPS/`-Ordner im Workspace wurde als
   Submodule committed und blockierte die Sandbox.
4. **`.gitignore` prüfen** vor jedem Commit — sicherstellen dass keine temporären Dateien,
   Build-Artefakte oder verschachtelte Repos eingeschlossen werden.
5. Bei Branch-Schutz (force-push verboten): Einfach einen neuen Branch `agent/zai-work-vN`
   erstellen und pushen. Den alten Branch auf GitHub manuell löschen.

### Aktuelle Branch-Situation

| Branch | Status | Verwendung |
|--------|--------|------------|
| `main` / `master` | Geschützt, hat diverse Merge-Artefakte | Nicht als Arbeitsbranch nutzen |
| `agent/zai-work` | Divergiert, geschützt vor force-push | Alt, kann gelöscht werden |
| `agent/zai-work-v2` | ✅ Sauber, alle Fixes enthalten | Letzter sauberer Stand (vor CRC-Fix) |
| `agent/zai-work-v3` | **Aktuell** | Alle Fixes + CRC-Fix für Discovery-PGNs |

### Wenn die Sandbox blockiert ist

Wenn ein `git checkout`, `stash pop`, oder ähnliches zu einem Merge-Konflikt führt
und ALLE Tool-Aufrufe mit "error: you need to resolve your current index first"
fehlschlagen:

Im Sandbox-Terminal ausführen:
```bash
cd /home/z/my-project/ag-steer
git add -A
git reset --hard origin/<sauberer-branch>
```

Falls `reset --hard` auch blockiert:
```bash
rm -f .git/MERGE_HEAD .git/MERGE_MSG .git/MERGE_MODE
git add <konflikt-dateien>
git reset --hard origin/<sauberer-branch>
```

---

## 6. Entwicklungskonventionen

### Logging

| Situation | Funktion | Ausgabe |
|-----------|----------|---------|
| Normale Firmware-Logs | `hal_log(...)` | USB CDC Serial |
| PGN-Codec interne Logs | `LOGI("PGN", ...)` / `LOGE(...)` | Geht über `log_ext.h` → `hal_log` |
| Module-spezifische Logs | `LOGI("NET", ...)`, `LOGI("CTL", ...)` | Compile-time Level pro Modul |

**WICHTIG:** `ESP_LOGI()` geht zu UART0 (nicht sichtbar im Serial Monitor wenn USB CDC
genutzt wird). Immer `hal_log()` verwenden, nicht `ESP_LOGI()` direkt.

**WICHTIG:** `hal_log()` nutzt einen recursive Mutex (`s_log_mutex`) um USB CDC Panics
bei parallelem Zugriff von Core 0 und Core 1 zu verhindern.

### Log-Level (Compile-Time)

In `log_config.h` werden pro Modul Log-Levels definiert:
```cpp
#define LOG_LEVEL_MAIN   ESP_LOG_VERBOSE   // 5
#define LOG_LEVEL_NET    ESP_LOG_DEBUG     // 4
#define LOG_LEVEL_PGN    ESP_LOG_WARN      // 2
// etc.
```

### State-Zugriff

Globaler State liegt in `global_state.h` (`g_nav` struct). Bei Zugriff aus
mehreren Tasks MUSS der Mutex verwendet werden:
```cpp
{
    StateLock lock;  // RAII: lock in scope, unlock at end
    g_nav.steer_angle_deg = 42.0f;
}
```

### Code-Stil

- **HAL-Abstraktion:** Alle Hardware-Zugriffe über `hal.h`. Keine Arduino/ESP32-Headers
  in der Logic-Layer.
- **PGN-Codec:** Rein prozedurale C++-Funktionen, keine Klassen. Alle PGN-Structs in
  `pgn_types.h` mit `static_assert` Größenprüfung und `__attribute__((packed))`.
- **Modularität:** Neue PGNs → `pgn_types.h` (Struct), `pgn_codec.h/.cpp` (Encoder/Decoder),
  `pgn_registry.h` (Registrierung), `net.cpp` (Dispatch).
- **Fehlerbehandlung:** Return-Codes (`bool`, `size_t==0`), keine Exceptions.
- **Kommentare:** Jeder PGN-Encoder/Decoder referenziert die PGN-Nummer und die AgOpenGPS-
  Doku (`PGN.md` auf GitHub).

### Build-System

PlatformIO (`platformio.ini`). Build mit `pio run`, Flash mit `pio run --target upload`.
Monitor: `pio device monitor` oder Arduino Serial Monitor (USB CDC).

---

## 7. Noch fehlende Boot-Logs

Im letzten Boot-Log waren folgende Messages noch **nicht sichtbar** (Stand vor CRC-Fix):

| Message | Erwartet in | Vermutete Ursache |
|---------|-------------|-------------------|
| `MODULES: === Hardware Detection ===` | `modulesInit()` | Wurde vor Log-Fix nicht angezeigt |
| `CTL: initialised (PID Kp=...)` | `controlInit()` | Wurde vor Log-Fix nicht angezeigt |
| `STAT: hd=... st=...` | `loop()` alle 5s | Sollte funktionieren nach Log-Fix |
| `tasks created, entering main loop` | `setup()` nach Task-Erstellung | Sollte funktionieren nach Log-Fix |

**Mit dem CRC-Fix (Discovery-PGNs werden jetzt akzeptiert) und den vorherigen Log-Fixes
sollten diese Messages beim nächsten Flash-Versuch sichtbar sein.**

---

## 8. Offene Aufgaben & TODOs

### Hohe Priorität

- [ ] **Boot-Log validieren:** Flash mit neuestem Stand und prüfen ob alle erwarteten
  Messages erscheinen (Modules, CTL, STAT, Tasks created).
- [ ] **DBG-Marker entfernen:** Die `[DBG-CTRL]`, `[DBG-COMM]`, `[DBG-LOOP]` Hz-Meldungen
  sind temporär. Nach Validierung der Loop-Frequenzen entfernen oder auf DEBUG-Level setzen.
- [ ] **PGN 251 (Steer Config) anwenden:** Aktuell wird nur geloggt. Die Config-Bits
  (InvertWAS, RelayActiveHigh, MotorDriveDirection, etc.) müssen auf die Hardware
  angewendet werden.

### Mittlere Priorität

- [ ] **BNO085 IMU Integration:** Hardware ist noch nicht angeschlossen. `hal_imu_begin()`,
  `hal_imu_read()`, `hal_imu_detect()` existieren aber sind nicht produktiv getestet.
  Kalibrierungs-Procedure für Heading/_roll_offset fehlt.
- [ ] **Hardware-Watchdog:** ESP32 Task Watchdog ist aktiv (`esp_task_wdt_reset()`),
  aber ein externer Hardware-Watchdog fehlt.
- [ ] **PGN-Bibliothek auslagern:** PGN-Codec, Types, Registry sollen in eine separate
  Library ausgelagert werden, damit sie auch in anderen Firmware-Projekten (GPS Bridge,
  Machine Module) wiederverwendet werden können.

### Niedrige Priorität

- [ ] **GPS Bridge Firmware:** Zweites ESP32-Board das GPS-Daten empfängt und als
  PGN 214 (GPS Main Out) an AgIO weiterleitet.
- [ ] **Section Control:** PGN 182/183 Implementierung für Abschnittssteuerung.
- [ ] **NMEA-Output:** PGN 100 (Corrected Position) für GPS-Out Geräte.
- [ ] **Fehler-Reporting erweitern:** `modulesSendStartupErrors()` nutzen, aber
  Runtime-Errors an AgIO senden (PGN 221).
- [ ] **PlatformIO CI/CD:** Automatischer Build-Check bei Push.

---

## 9. Was bei der Weiterentwicklung dokumentiert werden muss

Um den Entwicklungsstil und die Code-Qualität zu halten, sollten folgende
Dinge bei jeder neuen Session/Änderung beachtet werden:

### Vor jeder Änderung

1. **Handover lesen:** Diese Datei als Einstiegspunkt nutzen um den aktuellen Stand
   zu verstehen.
2. **Branch prüfen:** Niemals auf `main` arbeiten. Aktuellen Branch mit
   `git branch --show-current` verifizieren.
3. **Vorherigen Stand verstehen:** `git log --oneline -10` um die letzten Änderungen
   zu sehen.

### Bei jedem Commit

4. **Commit-Message Format:** `fix: Beschreibung` oder `feat: Beschreibung` oder
   `refactor: Beschreibung`. Kurze, präzise Beschreibung was und warum.
5. **Keine temporären Dateien committen:** `.gitignore` prüfen.
6. **Keine verschachtelten Repos:** Niemals einen Repo-Ordner innerhalb des Workspace
   erstellen oder committen.

### Bei jeder Code-Änderung

7. **HAL-Trennung einhalten:** Hardware-Zugriff NUR über `hal.h`. Keine ESP32/Arduino-
   Headers in `logic/`-Dateien.
8. **Logging-Konvention:** `hal_log()` für alle User-sichtbaren Logs. `LOGI("MODUL", ...)`
   für modulinterne Logs. NIE `ESP_LOGI()` direkt.
9. **PGN-Änderungen:** Neue PGNs IMMER in `pgn_types.h` (Struct), `pgn_codec.h/.cpp`
   (Encoder/Decoder), `pgn_registry.h` (Registrierung) und `net.cpp` (Dispatch)
   anlegen. Alle Structs mit `static_assert` und `packed`.
10. **State-Zugriff mit Mutex:** Jeder Zugriff auf `g_nav` aus Tasks MUSS mit
    `StateLock lock;` geschützt werden.
11. **Self-Tests:** Bei Änderungen am PGN-Codec: `pgnChecksumSelfTest()` muss weiterhin
    bestehen. Bei neuen PGNs: Testfall hinzufügen.

### Bei Fehlern / Sandbox-Problemen

12. **Niemals cherry-pick oder stash-pop.** Bei Konflikten: Neuen Branch pushen.
13. **Bei Sandbox-Blockade:** Nutzer im Sandbox-Terminal eingreifen lassen.
14. **Kurzfristige DBG-Ausgaben:** Temporäre Debug-Meldungen klar markieren (`[DBG-...]`)
    und in der TODO-Liste vermerken.

### Dokumentation

15. **Bei neuen PGNs:** Referenz zur AgOpenGPS-Doku (`PGN.md`) im Kommentar angeben.
16. **Bei Hardware-Änderungen:** Pin-Zuordnung und SPI-Bus-Konfiguration in `hal_impl.h`
    aktualisieren.
17. **Bei neuen Modulen:** In `modules.cpp` registrieren, in `MODULES:` Detection-Log
    aufnehmen, in `pgn_registry.h` eintragen.
18. **ChatGPT-Analyse dokumentieren:** Wichtige Erkenntnisse aus AgOpenGPS-Quellcode-Analysen
    (wie die CRC-Discovery-Sache) sollen hier im Handover ergänzt werden.
