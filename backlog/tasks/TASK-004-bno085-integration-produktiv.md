# TASK-004 BNO085 IMU produktiv integrieren

- **ID**: TASK-004
- **Titel**: BNO085-Pfad auf echter Hardware integrieren und kalibrieren
- **Status**: open
- **Priorität**: medium
- **Komponenten**: imu.cpp, hal_imu_begin/read/detect, Kalibrierlogik
- **Dependencies**: none
- **AC**:
  - IMU wird auf Zielhardware zuverlässig erkannt.
  - `hal_imu_read()` liefert stabile, plausibilisierte Werte.
  - Kalibrierungsablauf für Heading und Roll-Offset ist dokumentiert.
  - Fehlverhalten/Timeouts werden sauber geloggt.
- **Owner**: firmware-team
- **Links**:
  - docs/Handover2.md#8-offene-aufgaben--todos
  - backlog/epics/EPIC-002-sensor-and-safety.md
- **execution_mode**: hardware_required
