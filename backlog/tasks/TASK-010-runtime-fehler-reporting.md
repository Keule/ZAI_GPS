# TASK-010 Runtime-Fehlerreporting erweitern

- **ID**: TASK-010
- **Titel**: Laufzeitfehler zusätzlich via PGN 221 an AgIO senden
- **Status**: open
- **Priorität**: low
- **Komponenten**: modulesSendStartupErrors, net send path, error model
- **Dependencies**: TASK-003
- **AC**:
  - Startup- und Runtime-Fehler sind im gemeinsamen Fehlermodell definiert.
  - Runtime-Fehler werden über PGN 221 an AgIO übertragen.
  - Rate-Limiting/Entprellung verhindert Flooding.
- **Owner**: firmware-team
- **Links**:
  - docs/Handover2.md#8-offene-aufgaben--todos
  - backlog/epics/EPIC-004-feature-expansion.md
- **execution_mode**: firmware_only
