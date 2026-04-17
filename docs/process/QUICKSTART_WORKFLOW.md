# QUICKSTART_WORKFLOW — Schnellstart für den Arbeitsablauf

Dieses Dokument ist der **kompakte Einstieg** für neue Sessions und ergänzt den detaillierten Plan in `PLAN_AGENT.md`.

## Ziel

- In wenigen Minuten arbeitsfähig werden.
- Reihenfolge und Prüfpunkte standardisieren.
- Doku-, Code- und Backlog-Änderungen konsistent halten.

## 1) Vorbereitung (2–5 Minuten)

1. `README.md` lesen (Startpfade und verbindliche Prozessgrenzen).
2. `docs/Handover2.md` auf aktuellen Stand prüfen.
3. Relevante Backlog-Einträge in `backlog/index.yaml` und `backlog/tasks/` identifizieren.

## 2) Arbeitsauftrag schärfen

Für jede Aufgabe kurz festhalten:

- **Zielbild:** Was soll nachher messbar anders sein?
- **Scope:** Welche Dateien werden gelesen/geschrieben?
- **Risiko:** Gibt es Schnittstellen-, CRC- oder Merge-Risiken?
- **Abhängigkeiten:** Ist die Aufgabe independent oder dependent?

Wenn `README.md`, zentrale Header oder Build-Dateien betroffen sind, als **Merge-Risiko** markieren.

## 3) Umsetzung in kleinen Schritten

Empfohlener Ablauf:

1. Kleinsten sinnvollen Änderungsschnitt wählen.
2. Änderungen umsetzen.
3. Lokale Checks ausführen (`pio run`, `python3 tools/run_test_matrix.py`, falls anwendbar).
4. Doku/Backlog synchron halten (Task-Status, Handover-Hinweise, Referenzen).

## 4) Review-Checkliste vor Commit

- Passt die Änderung zum Zielbild?
- Sind CRC-/Protokollgrenzen unverändert korrekt?
- Sind betroffene Doku-Dateien mitgezogen?
- Sind Commit-Message und Diff verständlich und klein gehalten?

## 5) Abschluss

1. Kurzen, präzisen Commit erstellen.
2. Falls relevant: Backlog-/Handover-Notiz ergänzen.
3. Nächsten sinnvollen Schritt oder offene Risiken dokumentieren.

---

## Referenzen

- `docs/process/PLAN_AGENT.md` (Detailregeln für Parallelisierung, Locks, Abhängigkeiten)
- `README.md` (kanonischer Einstieg)
- `backlog/README.md` und `backlog/index.yaml` (Arbeitsplanung)
