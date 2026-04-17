# TASK-004 BNO085-Integration

- **ID**: TASK-004
- **Titel**: BNO085-Pfad auf echter Hardware integrieren und kalibrieren
- **Status**: in_progress
- **Priorität**: medium
- **Komponenten**: `src/logic/imu.*`, `src/hal_esp32/hal_bno085.cpp`, Kalibrierlogik
- **Dependencies**: none
- **Kontext/Problem**:
  - BNO085 ist funktional vorhanden, aber für produktiven Betrieb fehlen abgesicherte Hardware-Validierung und belastbare Kalibrierabläufe.
- **Scope (in)**:
  - Robuste Initialisierung/Erkennung auf Zielhardware.
  - Stabilisierung des Read-Pfads inkl. Plausibilisierung.
  - Dokumentierter Kalibrierablauf für Heading und Roll-Offset.
- **Nicht-Scope (out)**:
  - Austausch des Sensors oder komplette Neuentwicklung des IMU-Stacks.
- **AC**:
  - IMU wird auf Zielhardware zuverlässig erkannt.
  - `hal_imu_read()` liefert stabile, plausibilisierte Werte.
  - Kalibrierungsablauf für Heading und Roll-Offset ist dokumentiert.
  - Fehlverhalten/Timeouts werden sauber geloggt.
- **Umgesetzt in dieser Session (Teilabschluss)**:
  - `hal_imu_detect_boot_qualified()` nutzt jetzt reproduzierbare Grenzwerte
    (`20` Samples, `min_ok=18`, klassifiziert nach `ok/ff/zero/other`) statt
    eines Einzelsamples.
  - `hal_imu_read()` nutzt einen strengeren Freshness-Grenzwert
    (`kImuFreshTimeoutUs = 300000`) zur Stabilisierung des Datenpfads.
  - Dokumentierte Heading-Kalibriersequenz implementiert:
    - **Startbedingungen:** Geo+Game verfügbar und stationär
      (`|yaw_rate| <= 12 dps`)
    - **Dauer:** min. `3000 ms`, max. `12000 ms` bis Neustart
    - **Erfolgskriterien:** mind. `30` Samples und Kreis-Konzentration `>= 0.80`
  - Fallback bei IMU-Read-Fehlern implementiert:
    - Enter degraded mode nach `5` Fehl-Lesungen in Folge
    - Exit nach `3` erfolgreichen Lesungen in Folge
    - Im degraded mode werden keine falschen IMU-Werte weitergereicht
      (nur Quality-Flags/Invalid-Sentinel im Protokollpfad).
- **Verifikation/Test**:
  - Hardware-Testlauf mit reproduzierbaren Kalibrierwerten.
  - Fehlerfälle (Sensor nicht vorhanden, Timeout) sind im Log nachvollziehbar.
- **Owner**: firmware-team
- **Links**:
  - `docs/Handover2.md#8-offene-aufgaben--todos`
  - `backlog/epics/EPIC-002-sensor-and-safety.md`
- **delivery_mode**: hardware_required
