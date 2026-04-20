Entwickler-Report für Task TASK-028

Entwickler: codex
Task-ID: TASK-028

Checkliste (Pflichtfelder)

- [x] Zusammenfassung ausgefüllt
- [x] Geänderte Dateien vollständig aufgelistet
- [x] Tests / Build dokumentiert (inkl. Ergebnis)
- [x] Offene Fragen / Probleme dokumentiert (oder explizit „keine“)

Zusammenfassung

Umsetzung als kombinierter Change-Set für Tasks 026-030 (Config-Umstellung, Module/NTRIP/maintTask, Soft-Config Grundgerüst).

Geänderte Dateien

- include/fw_config.h
- include/soft_config.h
- src/main.cpp
- src/logic/modules.h
- src/logic/modules.cpp
- src/logic/ntrip.h
- src/logic/ntrip.cpp
- src/logic/sd_logger.h
- src/logic/runtime_config.h
- src/logic/runtime_config.cpp
- src/hal_esp32/hal_impl.cpp
- src/hal_esp32/hal_bno085.cpp
- src/hal_esp32/sd_ota_esp32.cpp
- src/hal_esp32/sd_logger_esp32.cpp

Tests / Build

- `rg '"hardware_pins\\.h"' include src` -> keine Treffer.
- `pio run -e gnss_buildup` -> fehlgeschlagen, da `pio` im Container nicht installiert.

Offene Fragen / Probleme

- Vollständige AC-Abdeckung für TASK-027/029/030 benötigt Hardware-/Integrationstest.
- Backlog-Statusänderungen (TASK-025/TASK-030 Scope) wurden hier nicht geändert.
