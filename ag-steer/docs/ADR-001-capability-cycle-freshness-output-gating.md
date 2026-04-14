# ADR-001: Capability-Matrix, Zyklusfrequenzen, Freshness/Timeout/Fallback und Output-Gating

- **Status:** Accepted
- **Datum:** 2026-04-14
- **Kontext:** `steer-controller` Laufzeitverhalten (ESP32-S3, FreeRTOS, AgOpenGPS/AgIO)

## 1) Entscheidung (final)

Diese ADR fixiert die **finale Runtime-Architektur** für den Steering Controller:

1. Eine klare **Capability-Matrix** pro Core/Task/Subsystem.
2. Verbindliche **Zyklusfrequenzen** für Control-, Comm- und Telemetriepfade.
3. Einheitliche Regeln für **Input-Freshness**, **Timeouts** und **Fallbacks**.
4. Verbindliche Kriterien, wann **Outputs** (PGN/Aktorik) aktiv gesendet bzw. sicher gesperrt werden.

---

## 2) Finale Capability-Matrix

| Capability | Verantwortlich | Core / Kontext | Input | Output | Gate / Bedingung |
|---|---|---|---|---|---|
| Safety-Check & Aktorik-Freigabe | `controlStep()` | Core 1 (`controlTask`) | Safety-GPIO, Schalterstatus, GPS-Speed, Watchdog | Aktuatorbefehl | Nur bei `safety_ok && work_switch && steer_switch && !watchdog_triggered && gps_speed_kmh >= 0.1` |
| PID-Regelung Lenkung | `controlStep()` + PID | Core 1 | Sollwinkel aus PGN 254, Istwinkel (WAS) | PWM/Command an Aktuator | Nur wenn Aktorik freigegeben; sonst Reset + Command 0 |
| IMU/WAS Einlesen | `imuUpdate()`, `steerAngleReadDeg()` | Core 1 | SPI-Sensoren | State-Update (Heading/Roll/WAS raw) | Bei jeder Control-Iteration |
| UDP Receive/Dispatch | `netPollReceive()` + `netProcessFrame()` | Core 0 (`commTask`) | UDP-Frames | State-Update, Trigger für Module-Replies | Nur Frames mit `Src=AgIO (0x7F)` werden verarbeitet |
| Periodische PGN-Telemetrie | `netSendAogFrames()` | Core 0 | globaler Zustand (`g_nav`) | PGN 253 + PGN 250 | Nur wenn Ethernet verbunden und 100 ms Sendeintervall erreicht |
| Discovery/Management Replies | `modulesSendHellos()`, `modulesSendSubnetReplies()` | Core 0 | PGN 200 / PGN 202 | Hello-/Subnet-Replies | Event-getrieben durch AgIO-Anfrage |
| HW-Status-Monitoring | `hwStatusUpdate()` | Core 0 | Ethernet/Safety/IMU/WAS/Steer-HW-Status | PGN 221 (HW Message) | Debounce, Rate-Limit, Resend-Intervall |
| Startup-HW-Fehlerreport | `modulesSendStartupErrors()` | Setup + Comm-Pfad | Hardware-Detektion | PGN 221 oder Serial-Log | UDP nur bei Netzwerk; sonst Serial-Fallback |

---

## 3) Zyklusfrequenzen pro Core

### Core 1 – `controlTask`
- **Task-Startverzögerung:** 500 ms (Stabilisierung).
- **Loop-Ziel:** **200 Hz** (`5 ms` Intervall).
- **Teilschritte je Zyklus:** Safety → IMU → WAS → Freigabe-/Watchdog-/Speed-Gates → PID → Aktuator.
- **Nebenpfad:** SD-Logger Record pro Zyklus (intern auf 10 Hz subsampled).

### Core 0 – `commTask`
- **Task-Startverzögerung:** 2000 ms (Netzwerk settle).
- **Poll-Loop:** **100 Hz** (`10 ms` Intervall).
- **Telemetrie-Senden:** **10 Hz** (`SEND_INTERVAL_MS = 100`).
- **HW-Status-Update:** ~**1 Hz** (`HW_STATUS_INTERVAL_MS = 1000`).

### HW-Status interne Taktung (zusätzlich)
- **Minimum zwischen HW-Messages:** 2 s.
- **Fehler-Debounce vor erstem Report:** 3 s.
- **Resend persistenter Fehler:** 10 s.

---

## 4) Regeln für Input-Freshness / Timeout / Fallback

### 4.1 Input-Freshness (AgIO Command-Pfad)
- Frische-Quelle ist **PGN 254 (Steer Data In)**.
- Bei gültigem PGN 254 werden aktualisiert:
  - Sollwinkel,
  - Schalterbits (`work_switch`, `steer_switch`),
  - Geschwindigkeit (`gps_speed_kmh`),
  - Watchdog-Timestamp (`watchdog_timer_ms = now`).

### 4.2 Timeout-Regel
- **Watchdog Timeout:** `2500 ms` ohne frisches PGN 254.
- Bei Timeout: `watchdog_triggered = true` und **Aktorik hart sperren** (PID Reset + Command 0).

### 4.3 Weitere Freshness-/Validitätsregeln
- Nur Pakete mit **Source `0x7F` (AgIO)** werden verarbeitet; alles andere wird ignoriert.
- Ungültige/unklare Frames werden verworfen; Logging ist rate-limitiert.
- Discovery-PGNs (200/201/202) folgen dem bestehenden AgIO-kompatiblen Parsing-Pfad.

### 4.4 Fallback-Regeln
- **Netzwerk down:**
  - Periodische PGNs (253/250) werden nicht gesendet.
  - Fehlerkommunikation fällt auf **Serial Logging** zurück (insb. Startup-Errors).
- **Safety nicht OK / Timeout / zu geringe Geschwindigkeit / Auto-Steer off:**
  - Aktuatorbefehl wird auf **0** gesetzt.
  - PID wird zurückgesetzt.

---

## 5) Kriterien für Output-Freigabe vs. Sperre

## 5.1 PGN-Outputs (UDP)

### A) PGN 253 + PGN 250 (periodisch)
**Senden wenn:**
1. Ethernet verbunden (`hal_net_is_connected() == true`), und
2. 100-ms-Intervall erreicht.

**Sperren wenn:**
- Ethernet nicht verbunden, oder
- Intervall noch nicht erreicht.

> Hinweis: Diese Status-PGNs transportieren den internen Sicherheits-/Schalterzustand; die eigentliche Aktorik-Freigabe erfolgt separat in der Control-Logik.

### B) Hello/Subnet-Replies
**Senden wenn:**
- AgIO Discovery-Request eingegangen ist (PGN 200/202).

**Sperren wenn:**
- Kein Trigger-Ereignis vorliegt.

### C) PGN 221 Hardware-Message
**Senden wenn:**
- Hardwarefehler aktiv, Debounce erfüllt, Rate-Limit erfüllt.

**Sperren wenn:**
- Debounce/Rate-Limit noch nicht erfüllt oder kein aktiver Fehler.

**Fallback:**
- Bei Startup-Fehlern ohne Netz: Serial-Ausgabe statt UDP.

## 5.2 Aktorik-Output (Steuerbefehl)

**Aktorik senden (PID-Command >0 möglich) nur wenn alle Bedingungen gleichzeitig erfüllt sind:**
1. `safety_ok == true`
2. `work_switch == true`
3. `steer_switch == true`
4. `watchdog_triggered == false` (PGN-254 frisch genug)
5. `gps_speed_kmh >= 0.1`

**Aktorik sperren (Command = 0) sobald eine Bedingung verletzt ist.**

Zusätzlich gilt:
- Bei Sperre wird der PID-Zustand zurückgesetzt (kein Windup-Übertrag in den nächsten Freigabezeitraum).

---

## 6) Konsequenzen

### Positiv
- Deterministische Trennung von **Control (Core 1)** und **Kommunikation (Core 0)**.
- Safety- und Freshness-Regeln sind explizit, messbar und testbar.
- Definierte Fallbacks verhindern „silent failure“ bei Netz-/Subsystemproblemen.

### Trade-offs
- Kein Telemetrieversand ohne Netz (gewollt, aber Diagnosekanal auf Serial begrenzt).
- Harte Aktorik-Sperre bei Timeout priorisiert Sicherheit vor Komfort.

---

## 7) Nicht-Ziele in dieser ADR

- Keine Änderung am Protokollformat (PGN-Layouts bleiben unverändert).
- Keine Einführung neuer Tasks/Cores.
- Keine inhaltliche PID-Tuning-Strategie.

