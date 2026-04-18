# ZAI_GPS — AgSteer Firmware (AgOpenGPS)

Dieses Repository ist der **kanonische Einstieg** für das Projekt `ZAI_GPS`.

## Startpfade

### Für Menschen
1. **System/Architektur verstehen:**
   - [`docs/ADR-001-capability-cycle-freshness-output-gating.md`](docs/ADR-001-capability-cycle-freshness-output-gating.md)
   - [`docs/protocol/README.md`](docs/protocol/README.md)
2. **Projektstand/Handover lesen:**
   - [`docs/Handover2.md`](docs/Handover2.md) (aktuell)
   - [`docs/Handover1.md`](docs/Handover1.md) (historisch)
3. **Prozessregeln und Arbeitsprinzipien:**
   - [`docs/plans/P1-dependent-api-freeze-und-crc-grenze.md`](docs/plans/P1-dependent-api-freeze-und-crc-grenze.md)
   - [`docs/process/PLAN_AGENT.md`](docs/process/PLAN_AGENT.md)
   - [`docs/process/QUICKSTART_WORKFLOW.md`](docs/process/QUICKSTART_WORKFLOW.md)
4. **Backlog/Arbeitsverlauf:**
   - [`backlog/README.md`](backlog/README.md)
   - optional direkt: [`backlog/index.yaml`](backlog/index.yaml)
5. **Standard-Templates für Doku/Übergaben:**
   - [`templates/session-start.md`](templates/session-start.md)
   - [`templates/session-progress.md`](templates/session-progress.md)
   - [`templates/session-handover.md`](templates/session-handover.md)
   - [`templates/task.md`](templates/task.md)
   - [`templates/adr.md`](templates/adr.md)

### Für Agenten
1. **Zuerst dieses README lesen** (kanonischer Einstieg).
2. **Protokollgrenze verinnerlichen (verpflichtend):**
   - **Core-AOG-PGNs**: CRC/Checksum **strikt validieren**.
   - **Discovery/Management-PGNs**: definierte **CRC-Ausnahme** (toleranter Pfad).
   - Referenzen:
     - [`docs/Handover2.md`](docs/Handover2.md)
     - [`docs/plans/P1-dependent-api-freeze-und-crc-grenze.md`](docs/plans/P1-dependent-api-freeze-und-crc-grenze.md)
     - [`docs/protocol/README.md`](docs/protocol/README.md)
3. **Dann in dieser Reihenfolge arbeiten:**
   - Architektur (`ADR-001`) → Handover (`Handover2`) → Prozess (`P1-Plan`) → Backlog (`backlog/README.md`, optional `backlog/index.yaml`).

## Konsistente Einstiege nach Thema

- **Architektur:**
  - [`docs/ADR-001-capability-cycle-freshness-output-gating.md`](docs/ADR-001-capability-cycle-freshness-output-gating.md)
- **Handover:**
  - [`docs/Handover2.md`](docs/Handover2.md)
  - [`docs/Handover1.md`](docs/Handover1.md)
- **Prozess:**
  - [`docs/plans/P1-dependent-api-freeze-und-crc-grenze.md`](docs/plans/P1-dependent-api-freeze-und-crc-grenze.md)
  - [`docs/process/PLAN_AGENT.md`](docs/process/PLAN_AGENT.md)
  - [`docs/process/QUICKSTART_WORKFLOW.md`](docs/process/QUICKSTART_WORKFLOW.md)
- **Backlog:**
  - [`backlog/README.md`](backlog/README.md)
  - optional direkt: [`backlog/index.yaml`](backlog/index.yaml)
- **Templates:**
  - [`templates/session-start.md`](templates/session-start.md)
  - [`templates/session-progress.md`](templates/session-progress.md)
  - [`templates/session-handover.md`](templates/session-handover.md)
  - [`templates/task.md`](templates/task.md)
  - [`templates/adr.md`](templates/adr.md)

## Protokollbesonderheit (sichtbar und verbindlich)

> **Nicht verhandelbar:**
>
> - **Core-AOG-Pfad** (z. B. Steer Data/Status/Settings/Config): CRC strikt.
> - **Discovery/Management-Pfad** (z. B. PGN 200/201/202 inkl. Replies): CRC-Ausnahme gemäß AgIO-Kompatibilität.
>
> Änderungen dürfen diese Grenze nicht implizit verschieben.


## Prozess-Update (Stand: 2026-04-18)

- Capability-Steuerung für optionale SPI/UART-Pfade wird über Backlog-Tasks in der Kette **Compile-Time → Boot-Init → Pin-Claiming** geführt (TASK-022 bis TASK-024, abgeschlossen).
- Für über Chat gestartete KI-Entwickler-Aufgaben gilt: klickbare Codex-Buttons müssen den Pflicht-Onboarding-Block enthalten (`README.md`, `docs/process/PLAN_AGENT.md`, `docs/process/QUICKSTART_WORKFLOW.md`).

## GNSS-Buildup (kurz, normativ)

Für GNSS-Diagnose/Bringup ist das dedizierte PlatformIO-Profil `gnss_buildup` zu verwenden.

- Zweck: reduzierter Init-Pfad (Comm + GNSS), UART-Mirror für schnelle Diagnose.
- Verbindliche Doku: [`docs/process/GNSS_BUILDUP.md`](docs/process/GNSS_BUILDUP.md).
- Finale UART-Pins (Board): UART1 `TX48/RX45`, UART2 `TX2/RX1`.

## Build & Smoke lokal

```bash
pio run
python3 tools/run_test_matrix.py
```

Nur Host-Smoke ohne PlatformIO-Builds:

```bash
SKIP_PROFILE_BUILDS=1 python3 tools/run_test_matrix.py
```
