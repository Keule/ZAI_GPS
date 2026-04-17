# Session Handover
- Datum/Zeit (UTC): 2026-04-17 08:12
- Status (Ampel): gelb

## Geänderter Stand / Ergebnisse
- Backlog rund um RTCM-Forwarder wurde um zusammenhängende Aufgaben erweitert bzw. im Branch aufgebaut:
  - `TASK-013` (zweimal vergeben, siehe Risiken):
    - Planungs-/Claim-Task: `backlog/tasks/TASK-013-hal-net-interface-claims-und-parallelisierung.md`
    - Design-Task RTCM-Forwarder: `backlog/tasks/TASK-013-rtcm-forwarder-design-und-schnittstellen.md`
  - `TASK-014` (zweimal vergeben, siehe Risiken):
    - Doku/CRC-Grenz-Task: `backlog/tasks/TASK-014-doku-rtcm-um980-und-crc-grenze.md`
    - Implementierungs-Task HAL-UART-Forwarding: `backlog/tasks/TASK-014-hal-gnss-rtcm-uart-forwarding.md`
  - `TASK-015`: UDP-RTCM-Empfang + Pufferung (`backlog/tasks/TASK-015-udp-rtcm-receiver-und-buffering.md`)
  - `TASK-016`: PGN-214 FixQuality/Age Integration (`backlog/tasks/TASK-016-pgn214-fixquality-age-integration.md`)
  - `TASK-017`: E2E-Validierung AgIO ↔ UM980 (`backlog/tasks/TASK-017-rtcm-validierung-agiou-m980.md`)
  - `TASK-018`: Betriebs-/Handover-Doku (`backlog/tasks/TASK-018-doku-rtcm-forwarder-und-handover.md`)
- `backlog/index.yaml` wurde auf die neuen Tasks erweitert, enthält aber strukturelle Konsistenzprobleme:
  - Doppelte IDs (`TASK-013`, `TASK-014`),
  - YAML-Strukturbruch in Task-Blöcken (zweite `title/file/...`-Sequenz innerhalb derselben List-Item-Struktur),
  - dadurch inkonsistente Epic-Referenzen und fehlerhafte Tool-Verarbeitung.
- HAL/Public-API wurde für RTCM erweitert (`src/hal/hal.h`):
  - GNSS-RTCM-UART-API (`hal_gnss_rtcm_begin`, `hal_gnss_rtcm_write`, `hal_gnss_rtcm_is_ready`, `hal_gnss_rtcm_drop_count`),
  - dedizierte Empfangs-API für RTCM-UDP (`hal_net_receive_rtcm`).
- ESP32-HAL-Implementierung (`src/hal_esp32/hal_impl.cpp`) ergänzt:
  - separater RTCM-UDP-Socket (`ethUDP_rtcm`) auf eigenem Listen-Port,
  - GNSS-RTCM-UART-Pfad mit Mutex-Schutz, init-/ready-Status und Drop-Counter,
  - non-blocking/partielle Write-Semantik zur Entkopplung von Netzwerk und UART.
- Netzwerklogik (`src/logic/net.cpp`) um RTCM-Datenpfad erweitert:
  - `netPollRtcmReceiveAndForward()` liest RTCM-Datagramme via `hal_net_receive_rtcm`,
  - Ringbuffer (`RTCM_RING_CAPACITY = 4096`) für Entkopplung RX/Forwarding,
  - Telemetrie für `rx_bytes`, `forwarded_bytes`, `dropped_packets`, `overflow_bytes`, `partial_writes`,
  - Weiterleitung in Chunks über `hal_gnss_rtcm_write`.

## Offene Punkte und Risiken
- **Backlog-Index defekt**:
  - Validator meldet aktuell `duplicate task id: TASK-014`.
  - Zusätzlich sind Task-Elemente durch verschachtelte Felder semantisch beschädigt (manuell im YAML sichtbar).
- **Doppelte/inkonsistente API-Deklaration in `hal.h`**:
  - `hal_gnss_rtcm_write` ist sowohl im Abschnitt „GNSS RTCM (UART)“ als auch erneut im Abschnitt „Network (W5500 Ethernet)“ deklariert.
  - Funktional kompilierbar, aber API-Ownership/Layering unsauber und wartungsriskant.
- **Unvollständig abgesicherter Datenfluss Net → HAL**:
  - Ringbuffer- und Partial-Write-Logik existiert, aber ohne belastbare Last-/Soak-Tests.
  - Kein klar dokumentierter Rückstau-/Timeout-Mechanismus über die bestehenden Zähler hinaus.
- **Testlücken**:
  - Keine automatisierten Unit-/Integrationstests für RTCM-Ringbuffer-Grenzfälle,
  - keine reproduzierbare Hardware-Testmatrix als versioniertes Artefakt im Repo,
  - unklar, ob AgIO/NTRIP-Szenarien mit hoher Burst-Rate stabil abgefangen werden.
- **Merge-Risiko hoch** in zentralen Dateien:
  - `backlog/index.yaml`, `src/hal/hal.h`, `src/hal_esp32/hal_impl.cpp`, `src/logic/net.cpp`, `src/logic/net.h`.

## Nächste konkrete Schritte
1. **Backlog reparieren (zuerst, blocker-fix)**
   - Eindeutige IDs herstellen (z. B. Planungs-Task vs. Forwarder-Design sauber trennen, keine Wiederverwendung von `TASK-013/014`).
   - `backlog/index.yaml` so korrigieren, dass jedes Task-List-Element genau einen konsistenten Feldsatz hat.
   - Danach Validator wieder grün laufen lassen.
2. **RTCM-Schnittstelle refactoren (API-Klarheit)**
   - `hal_gnss_rtcm_write` nur einmal in `hal.h` deklarieren (GNSS-Sektion),
   - Ownership dokumentieren: `net.cpp` konsumiert nur HAL-API, keine impliziten Layer-Übergriffe.
3. **Design-/Validierungs-Task als In-Progress starten**
   - Entweder `TASK-013-rtcm-forwarder-design-und-schnittstellen` oder `TASK-017` konkretisieren,
   - Testprotokoll für Burst, Dauerlast, Link-Flaps und UART-Backpressure versionieren.
4. **Akzeptanzkriterien (empfohlen für Folgesession)**
   - AC-1: `python3 tools/validate_backlog_index.py` ohne Fehler.
   - AC-2: Keine doppelte Prototyp-Deklaration in `src/hal/hal.h`.
   - AC-3: Dokumentierter Datenfluss UDP→Ringbuffer→UART inkl. Grenzverhalten (Overflow/Partial Write).
   - AC-4: Mindestens ein reproduzierbarer Testreport (HW oder Host-stub) mit Telemetrie-Auswertung.

## Referenzen auf betroffene Dateien und Tasks
- Backlog/Tasks:
  - `backlog/index.yaml`
  - `backlog/tasks/TASK-013-hal-net-interface-claims-und-parallelisierung.md`
  - `backlog/tasks/TASK-013-rtcm-forwarder-design-und-schnittstellen.md`
  - `backlog/tasks/TASK-014-doku-rtcm-um980-und-crc-grenze.md`
  - `backlog/tasks/TASK-014-hal-gnss-rtcm-uart-forwarding.md`
  - `backlog/tasks/TASK-015-udp-rtcm-receiver-und-buffering.md`
  - `backlog/tasks/TASK-016-pgn214-fixquality-age-integration.md`
  - `backlog/tasks/TASK-017-rtcm-validierung-agiou-m980.md`
  - `backlog/tasks/TASK-018-doku-rtcm-forwarder-und-handover.md`
- Implementierung:
  - `src/hal/hal.h`
  - `src/hal_esp32/hal_impl.cpp`
  - `src/logic/net.cpp`
  - `src/logic/net.h`

## Prozesshinweis (wichtig)
Diese Session-Handover-Notiz ist **nicht normativ**. Verbindliche Regeln und Definitionen stehen in den Projekt-Primärquellen (`README`, Architektur-/Prozessdokumente). Das Handover dient ausschließlich der Kontextübergabe für die nächste Session.
