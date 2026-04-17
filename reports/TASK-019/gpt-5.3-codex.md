Entwickler-Report für Task TASK-019

Entwickler: GPT-5.3-Codex
Task-ID: TASK-019

Zusammenfassung

Backlog um fünf neue Ausführungs-Tasks TASK-019A bis TASK-019E ergänzt (Pinbelegung, PlatformIO `gnss_buildup`, GNSS-Bringup-Modus, UART1/2→Console-Mirror, Smoke-Test-Reportstandard), den Index erweitert sowie eine bestehende ID-Kollision (`TASK-014`) im Index bereinigt, damit der Validator wieder grün läuft.

Geänderte Dateien
- backlog/tasks/TASK-019A-pinbelegung-um980-und-konsole.md
- backlog/tasks/TASK-019B-platformio-environment-gnss-buildup.md
- backlog/tasks/TASK-019C-gnss-bringup-modus.md
- backlog/tasks/TASK-019D-uart1-uart2-console-mirror.md
- backlog/tasks/TASK-019E-smoke-test-reportstandard.md
- backlog/index.yaml

Tests / Build
- `python3 tools/validate_backlog_index.py` zunächst fehlgeschlagen wegen fehlendem `PyYAML`.
- `python3 -m pip install --user pyyaml` ausgeführt.
- `python3 tools/validate_backlog_index.py` danach erfolgreich (`OK: backlog/index.yaml is valid`).

Offene Fragen / Probleme
- Keine inhaltlichen Blocker für den Backlog-Stand.
- Rollenhinweis: `backlog/README.md` weist Task-Neuanlage dem KI-Planer zu; Umsetzung erfolgte hier auf explizite Arbeitsanweisung.

---

Task-ID: TASK-019B
Titel: PlatformIO-Environment `gnss_buildup` für GNSS-Buildup
Datum: 2026-04-17

Umsetzung
- Neues PlatformIO-Environment `gnss_buildup` auf Basis `env:T-ETH-Lite-ESP32S3` in `platformio.ini` ergänzt.
- Für das Profil wurden ausschließlich Comm+GNSS-Flags gesetzt:
  - `-DFEAT_PROFILE_COMM_ONLY`
  - `-DFEAT_COMM`
  - `-DFEAT_GNSS`
- Damit bleiben Steering/IMU/Aktorik in diesem Profil deaktiviert (keine entsprechenden Flags gesetzt).
- Bestehende Profile blieben unverändert; es wurde nur ein zusätzlicher Profil-Eintrag ergänzt.

Build-Befehle und Ergebnis
- `pio run -e gnss_buildup`
  - Ergebnis: fehlgeschlagen, Tool nicht verfügbar (`pio: command not found`).
- `python3 -m platformio run -e gnss_buildup`
  - Ergebnis: fehlgeschlagen, Python-Modul nicht installiert (`No module named platformio`).

Blocker
- Lokale Umgebung enthält keine PlatformIO-CLI/kein PlatformIO-Python-Modul, daher kein vollständiger Build-Nachweis im Container möglich.
