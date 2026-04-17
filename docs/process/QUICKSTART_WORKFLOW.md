# Quickstart Workflow (Mensch + Agent)

Kurzleitfaden für sauberes Arbeiten über Chats/Agenten/Umgebungen hinweg.

## 1) Session-Start (immer gleich)
1. Einstieg lesen:
   - `README.md`
   - `docs/process/PLAN_AGENT.md`
   - `backlog/README.md`
2. Task auswählen oder anlegen:
   - bestehende `TASK-XXX` nutzen oder neue Task nach Schema erstellen.
3. Scope festziehen:
   - Was ist in scope / out of scope?
   - Welche Dateien werden voraussichtlich geändert?

## 2) Planung vor Umsetzung
1. Task klassifizieren:
   - `independent` oder `dependent`.
2. Abhängigkeiten prüfen:
   - `exclusive_before` und `parallelizable_after` festlegen (wenn nötig).
3. Konfliktflächen prüfen:
   - `merge_risk_files` identifizieren.
4. Claim/Lock setzen:
   - kritische Dateien exklusiv claimen, bevor implementiert wird.

## 3) Umsetzung
1. Kleine, nachvollziehbare Änderungen.
2. Architekturgrenzen respektieren:
   - Core-AOG-PGNs: CRC strikt.
   - Discovery/Management-Pfad: definierte CRC-Ausnahme.
3. Konventionen einhalten:
   - HAL-/Logic-Trennung,
   - Logging-Regeln,
   - State-Zugriffe mit Mutex.

## 4) Dokumentation während der Arbeit
1. Session-Templates nutzen:
   - `templates/session-start.md`
   - `templates/session-progress.md`
2. Bei Architektur-/Prozessentscheidungen:
   - ADR erfassen (`templates/adr.md`).

## 5) Session-Ende / Handover
1. `templates/session-handover.md` ausfüllen:
   - erledigt / offen / blocker / nächster konkreter Schritt.
2. Task-Status aktualisieren.
3. Referenzen dokumentieren:
   - betroffene Dateien, Commits/PR, offene Risiken.

## 6) Umgebungswechsel (Codex ↔ ChatGPT ↔ z.ai)
Beim Wechsel immer aktiv mitgeben:
- Task-ID
- aktueller Status
- geänderte Dateien
- offene Blocker
- nächster konkreter Schritt

Nie auf Chat-Kontext verlassen — der aktuelle Stand muss im Repo stehen (Task/Handover/ADR).