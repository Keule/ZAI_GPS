# TASK-011 PlatformIO CI/CD Build-Check

- **ID**: TASK-011
- **Titel**: Automatischen PlatformIO Build-Check bei Push etablieren
- **Status**: open
- **Priorität**: low
- **Komponenten**: .github/workflows, platformio build pipeline
- **Dependencies**: none
- **AC**:
  - CI-Workflow startet bei Push/PR.
  - Mindestens ein `pio run` Build wird automatisiert ausgeführt.
  - Build-Ergebnis ist in PR-Checks sichtbar.
- **Owner**: platform-team
- **Links**:
  - docs/Handover2.md#8-offene-aufgaben--todos
  - backlog/epics/EPIC-003-platform-and-reuse.md
- **execution_mode**: firmware_only
