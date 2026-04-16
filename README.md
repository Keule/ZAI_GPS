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
4. **Backlog/Arbeitsverlauf:**
   - [`worklog.md`](worklog.md)

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
   - Architektur (`ADR-001`) → Handover (`Handover2`) → Prozess (`P1-Plan`) → Backlog (`worklog.md`).

## Konsistente Einstiege nach Thema

- **Architektur:**
  - [`docs/ADR-001-capability-cycle-freshness-output-gating.md`](docs/ADR-001-capability-cycle-freshness-output-gating.md)
- **Handover:**
  - [`docs/Handover2.md`](docs/Handover2.md)
  - [`docs/Handover1.md`](docs/Handover1.md)
- **Prozess:**
  - [`docs/plans/P1-dependent-api-freeze-und-crc-grenze.md`](docs/plans/P1-dependent-api-freeze-und-crc-grenze.md)
- **Backlog:**
  - [`worklog.md`](worklog.md)

## Protokollbesonderheit (sichtbar und verbindlich)

> **Nicht verhandelbar:**
>
> - **Core-AOG-Pfad** (z. B. Steer Data/Status/Settings/Config): CRC strikt.
> - **Discovery/Management-Pfad** (z. B. PGN 200/201/202 inkl. Replies): CRC-Ausnahme gemäß AgIO-Kompatibilität.
>
> Änderungen dürfen diese Grenze nicht implizit verschieben.

## Build & Smoke lokal

```bash
pio run
python3 tools/run_test_matrix.py
```

Nur Host-Smoke ohne PlatformIO-Builds:

```bash
SKIP_PROFILE_BUILDS=1 python3 tools/run_test_matrix.py
```
