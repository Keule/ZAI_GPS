
Komplettes Projekt-Handover: ESP32-S3 AgSteer Autosteer-Controller
1. Projektstruktur & Repository
Repo & Branch
GitHub Repo: https://github.com/Keule/ZAI_GPS
Branch: feature/restructure-multi-device
Arbeitsverzeichnis auf Sandbox: /home/z/my-project/ag-steer/steer-controller/
WICHTIG: Am Anfang jeder Session cd /home/z/my-project/ag-steer und den Branch prüfen!
Sandbox-Verhalten
Die Sandbox resettet sich zwischen jeder Nachricht auf /home/z/my-project
Git-Branch bleibt erhalten, aber man muss in das richtige Verzeichnis wechseln
Die Session war previously durch einen korrupten Git-Merge-State gelockt → neue Session starten
Tipp: Lokalen main so setzen, dass er origin/feature/restructure-multi-device trackt:
bash

cd /home/z/my-project/ag-steer
git branch --set-upstream-to=origin/feature/restructure-multi-device main
git checkout main
git pull
So geht jeder git push automatisch zum richtigen Branch, egal ob die Sandbox resettet.
Projekt-Verzeichnisstruktur
text

ag-steer/
├── steer-controller/
│   ├── src/
│   │   ├── main.cpp           # Hauptschleife, Setup, Logging, Steuerungs-Loop
│   │   ├── net.cpp            # UDP Kommunikation, PGN Empfang/Sende, Frame-Verarbeitung
│   │   ├── net.h
│   │   ├── aog_udp_protocol.cpp  # AOG Frame-Validierung, CRC-Berechnung, Encode/Decode
│   │   ├── aog_udp_protocol.h    # PGN-Konstanten, Struct-Definitionen, Funktionsdeklarationen
│   │   ├── control.cpp        # ADC-Lesung, Winkelberechnung, PID-Regler
│   │   ├── control.h
│   │   ├── global_state.h     # Globaler State (NavState-Struct mit allen Steuerungsdaten)
│   │   ├── hal.h              # Hardware-Abstraktions-Layer Interface
│   │   ├── hal_impl.cpp       # HAL-Implementierung (ADS1118, W5500, GPIO)
│   │   └── hal_impl.h
│   ├── platformio.ini         # PlatformIO Konfiguration
│   └── docs/                  # Dokumentation, PGN-Beschreibungen
├── gps-bridge/                # (zukünftig) GPS Bridge Firmware
└── shared/                    # Gemeinsamer Code zwischen Modulen
2. Hardware
MCU: ESP32-S3 auf T-ETH-Lite-S3 (LilyGo Board)
ADC: ADS1118 (16-bit, SPI) — liest das Lenkwinkel-Potentiometer
Ethernet: W5500 (SPI) — UDP Kommunikation mit AgOpenGPS
IMU: BNO085 (I2C) — aktuell nur ein Stub (nicht implementiert)
Winkelbereich: ±45° (angepasst von ursprünglich anderem Bereich)
GPIO: Diverse Pins für Status-LEDs, PWM-Ausgang für Lenkaktor, Schalter-Inputs
Hardware-Konfiguration (wesentliche Pins)
ADS1118: SPI-Bus (shared oder dediziert)
W5500: SPI-Bus mit eigenem CS-Pin
PWM-Ausgang für Hydraulikventil / Elektromotor
Work-Switch und Steer-Switch als GPIO-Inputs
3. Software-Architektur
Module und Verantwortlichkeiten
main.cpp:

setup(): Initialisiert Serial, HAL (W5500, ADS1118, GPIO), Net (UDP Socket), startet Steuerungsschleife
loop(): Zyklischer Aufruf von netPollReceive(), controlStep(), netSendStatus() (ca. alle 10-50ms)
CRASH-FIX an Zeile 266 (siehe Abschnitt 6)
Log-Ausgabe über Serial mit Format-Strings
net.cpp:

netInit(): UDP-Socket auf Port 5126 erstellen (Broadcast)
netPollReceive(): Prüft auf eingehende UDP-Pakete (Port 5126 + Port 8888)
Filter 1: Eigene Broadcasts ignorieren (if (src == AOG_SRC_STEER) return;)
Filter 2: Nur Src=0x7F (AgIO) akzeptieren
Filter 3: NMEA-Frames ignorieren (if (buf[0] == 0x24) return;) — $-Prefix = GPS-NMEA Daten von AgIO
Rate-Limiting für Fehler-Logs (alle 10 Sekunden via static unsigned long lastLog)
netProcessFrame(): Decodiert validierte Frames und dispatcht nach PGN:
PGN 200 (Hello): Verbindungs-Setup
PGN 252 (Steer Settings): ACK + Konfiguration (PID-Parameter, Winkel-Einstellungen)
PGN 254 (Steer Data In): Zielwinkel und Geschwindigkeit von AOG
PGN 250 (Autosteer2): Zusätzliche Steuerungsdaten
netSendStatus() (PGN 253): Sendet aktuellen Lenkwinkel + Status alle 100ms
netSendHello() (PGN 200): Sendet Hello-Paket
aog_udp_protocol.cpp:

aogValidateFrame(): Validiert eingehende Frames:
Mindestlänge prüfen (≥ 14 Bytes: 2 Preamble + 1 Src + 1 PGN + 1 Len + min 1 payload + 1 CRC + 2 Postamble)
Preamble prüfen: buf[0] == 0x80 && buf[1] == 0x81
Postamble prüfen: buf[n-2] == 0x80 && buf[n-1] == 0x81
Längenkonsistenz: buf[4] muss mit n - 9 übereinstimmen
CRC-Validierung: CRC8 über buf[2..n-3] muss mit buf[n-3] übereinstimmen
DIES IST DIE STELLE DES BUGS — CRC-Berechnung stimmt vermutlich nicht mit AgOpenGPS überein
aogEncodeFrame(): Baut einen AOG-Frame zusammen
PGN-spezifische Encode/Decode-Funktionen
aog_udp_protocol.h:

Konstanten:
AOG_SRC_AGIO = 0x7F (127)
AOG_SRC_STEER = 0x7E (126) — eigene Source-Adresse
PGN-Nummern: 200, 250, 252, 253, 254
Ports: 5126 (Broadcast), 8888 (AgIO Antwort)
Structs für die verschiedenen PGN-Daten
control.cpp:

controlInit(): ADS1118 Kalibrierung, Nullpunkt bestimmen
controlStep():
ADC auslesen (ADS1118) → Rohwert
Rohwert in Winkel umrechnen (Linearisierung: 0–65535 → ±45°)
Wenn Work-Switch UND Steer-Switch aktiv: PID-Regler läuft
PID berechnet Ausgang (PWM-Wert) basierend auf Soll-Ist-Differenz
PWM ausgeben
controlUpdateSettings(): Aktualisiert PID-Parameter aus PGN 252 (Kp, Ki, Kd, minPWM, etc.)
global_state.h:

NavState-Struct (global g_nav):
Steer-Einstellungen (Kp, Ki, Kd, minPWM, maxPWM, etc.)
Switches (workSwitch, steerSwitch)
Aktueller Winkel, Sollwinkel
PID-Ausgang
ADC-Rohwerte
Verbindungsstatus (AOG verbunden ja/nein, letztes Hello)
4. AgOpenGPS UDP Protokoll — Detailliert
Frame-Format
text

Byte 0:     0x80          (Preamble Byte 1)
Byte 1:     0x81          (Preamble Byte 2)
Byte 2:     Src           (Source Address: 0x7F=AgIO, 0x7E=Steer, etc.)
Byte 3:     PGN           (PGN Number, 1 Byte)
Byte 4:     Length         (Payload Length in Bytes)
Byte 5..5+L-1: Payload    (PGN-spezifische Daten)
Byte 5+L:   CRC           (CRC8 über Bytes 2 bis 5+L-1)
Byte 5+L+1: 0x80          (Postamble Byte 1)
Byte 5+L+2: 0x81          (Postamble Byte 2)
Gesamt: 2 + 1 + 1 + 1 + Length + 1 + 2 = Length + 8 Bytes

CRC-Berechnung (unserer aktueller Code — MÖGLICHER FEHLER)
CRC8 über alle Bytes von Index 2 bis (n-3), also Src + PGN + Length + Payload
Der CRC-Wert steht an Position (n-3), also direkt vor dem Postamble
Problem: Die berechnete CRC stimmt nicht mit der im Frame empfangenen CRC überein
Symptom: Bei 11-Byte-Frames von Port 8888 immer got 0x47 (ASCII 'G')
UDP-Kommunikation
Broadcast Port: 5126 — alle Module hören hier, AgIO sendet hierauf
AgIO Port: 8888 — AgIO antwortet auf Hello-Pakete hierauf
Steer-Modul sendet auf: Broadcast Port 5126
Wichtige PGNs
PGN 200 (0xC8) — Hello

Wird vom Steer-Modul gesendet um sich bei AgIO anzumelden
AgIO antwortet mit eigenem Hello auf Port 8888
Payload: Versionsnummer, Modul-Typ, etc.
PGN 252 (0xFC) — Steer Settings / ACK

Von AgIO zum Steer-Modul
Enthält: PID-Parameter (Kp, Ki, Kd), Winkel-Einstellungen, minPWM, maxPWM, etc.
Wird als ACK auf Hello-Pakete verwendet
PGN 253 (0xFD) — Steer Status (OUT)

Vom Steer-Modul zu AgIO
Enthält: aktueller Lenkwinkel (×100 als int16), Switch-Status, Fehler-Status
Wird ca. alle 100ms gesendet
In unserem Code: steer_angle_deg × 100 → little-endian int16
PGN 254 (0xFE) — Steer Data (IN)

Von AgIO zum Steer-Modul
Enthält: Soll-Lenkwinkel (×100 als int16), Geschwindigkeit, Umschalter-Status
Dies ist der Haupt-Steuer-PGN von AgOpenGPS
PGN 250 (0xFA) — Autosteer2

Zusätzliche Steuerungsdaten
In unserem Code wird der ADC-Rohwert (low byte) gesendet
Unidentifizierte PGNs aus den Logs
PGN 0xFD (253) von Src 0x7E: Eigene Broadcasts die zurückkommen → gefiltert
PGN 0xCB (203) von Src 0x7E: Eigene Subnet-Antworten → gefiltert
PGN 0x7E (126) von Src 0x7E: 11-Byte-Frame, CRC=0x47 vs expected=0x08
PGN 0x7B (123) von Src 0x7B: 11-Byte-Frame, CRC=0x47 vs expected=0xFB
PGN 0x79 (121) von Src 0x79: 11-Byte-Frame, CRC=0x47 vs expected=0xF7
Hinweis: Diese Frames haben konstant CRC=0x47 → das ist ASCII 'G', was darauf hindeutet dass die CRC-Position falsch berechnet wird und stattdessen ein Datenbyte gelesen wird (möglicherweise Teil einer "$GPGGA..." NMEA-Zeichenkette)
5. Aktuelle Aufgabe — PGN-Analyse des Reference-Codes
Was gemacht werden muss:
Reference-Code herunterladen und analysieren:
URL: https://github.com/AgOpenGPS-Official/Boards/blob/main/ArduinoModules/UDP/Autosteer_UDP_v5/Autosteer_UDP_v5.ino
curl -sL "https://raw.githubusercontent.com/AgOpenGPS-Official/Boards/main/ArduinoModules/UDP/Autosteer_UDP_v5/Autosteer_UDP_v5.ino" -o /tmp/Autosteer_UDP_v5.ino
Alle verarbeiteten PGNs identifizieren: Welche PGNs empfängt der Reference-Code? Welche sendet er?
Frame-Format vergleichen: Wie werden Frames aufgebaut? Preamble, Src, PGN, Length, CRC, Postamble — identisch zu unserem Code?
CRC-Berechnung analysieren:
Welcher Algorithmus wird verwendet? (Polynom, Initialwert, Lookup-Tabelle?)
Über welche Bytes wird die CRC berechnet?
Stimmt das mit unserer aogValidateFrame() überein?
Das ist der vermutete Bug!
Ergebnis: Zusammenfassung der Unterschiede und Fix-Vorschläge für unsere Implementierung
Spezifische Fragen die zu klären sind:
Warum bekommt unsere CRC immer 0x47 bei 11-Byte-Frames?
Ist 0x47 tatsächlich ein Datenbyte und unsere CRC-Positions-Berechnung ist falsch?
Oder ist unser CRC-Algorithmus (Polynom/Init) falsch?
Sind PGN 0x7E, 0x7B, 0x79 gültige AgIO-Subnet-Adressen/PGNs?
6. Bereits durchgeführte Fixes (Commit-Historie)
Commit 32923e6 — Crash-Fix in main.cpp:266
Problem: Guru Meditation LoadProhibited (NULL Dereference in vsnprintf)
Ursache: 9 Format-Specifier im printf aber nur 8 Argumente. net=%s hatte kein zugehöriges Argument → vsnprintf las NULL vom Stack → Crash
Fix:

Fehlendes Argument ergänzt: (int)g_nav.pid_output
Format-String korrigiert: pid=%.1f → pid=%d (int, nicht float)
Argument-Reihenfolge korrigiert: desiredSteerAngleDeg auf richtige Position für tgt=%.1f
Zeile: Etwa Zeile 266 in main.cpp
Commit nach Crash-Fix — Log-Spam Filter (net.cpp)
Problem: Eigene UDP-Broadcasts wurden empfangen und verarbeitet → Endlosschleife an Logs
Fix: if (src == AOG_SRC_STEER) return; am Anfang von netProcessFrame()

Commit danach — NMEA + Port 8888 Filter
Problem: AgIO sendet NMEA-Daten ($GP...) auf Port 8888, unser Code versuchte diese als AOG-Frames zu parsen
Fix:

if (buf[0] == 0x24) return; in netPollReceive() — filtert alle $-prfixierten Frames
Nur src == AOG_SRC_AGIO (0x7F) akzeptieren in netProcessFrame()
Rate-Limiting für Fehler-Logs (alle 10s)
Commit danach — CRC-Rate-Limit
Problem: CRC-Mismatch Logs spammten das Serial-Monitor
Fix: Rate-Limiting in aogValidateFrame() — CRC-Fehler nur alle 10s loggen

7. Bekannte Probleme & Offene Punkte
KRITISCH — CRC-Validierung fehlerhaft
Status: UNBEHOBEN
Symptom: CRC mismatch: got 0x47 expected 0x08 (und ähnlich) bei Frames von Port 8888
Verdacht: CRC-Algorithmus in aogValidateFrame() stimmt nicht mit AgOpenGPS überein
Nächster Schritt: Reference-Code analysieren (siehe Abschnitt 5)
Offen — BNO085 IMU Integration
aktuell nur ein Stub
Wird für Heading/Gyro-Daten benötigt
I2C-Verbindung muss implementiert werden
Offen — GPS Bridge Firmware
Zweites Modul (gps-bridge/) ist geplant
Bridge zwischen GPS-Empfänger und Ethernet
Offen — Stabilität
Timeout-Handling wenn AgIO nicht antwortet
Watchdog implementieren
Fallback wenn Verbindung abbricht
8. Entwicklungs-Tipps für die neue Session
Immer zuerst: cd /home/z/my-project/ag-steer && git status && git log --oneline -5
Branch-Tracking: git branch --set-upstream-to=origin/feature/restructure-multi-device main
Commits: Aussagekräftige Commit-Messages, auf main pushen (trackt feature branch)
Serial Monitor Logs: Immer die letzten Zeilen von dev.log lesen nach Änderungen
PlatformIO: pio run -t upload zum Flashen, pio device monitor für Serial
CRC-Fix zuerst: Das ist die aktuell wichtigste Aufgabe
Dann: Alle PGNs aus dem Reference-Code mit unserer Implementierung abgleichen