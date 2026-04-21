# TASK-036 – Entwicklerreport (codex)

## Umsetzung
- `MOD_SD` als neues Feature-Modul ergänzt (`FirmwareFeatureId`, `MOD_COUNT`, Feature-Tabelle, Owner-Tag, Default-State über `cfg::MOD_DEFAULT_SD`).
- Boot-Detect-Gating umgesetzt:
  - Neue HAL-API `hal_sd_card_present()`.
  - SD-Präsenz in `modulesInit()` erfasst und in `ModuleHwStatus::sd_present` + `g_features[MOD_SD].hw_detected` gespiegelt.
  - Aktivierung in `setup()` nur bei erkanntem SD-Medium; sonst explizite Deaktivierung + Log.
- Hartes Gating über `moduleIsActive(MOD_SD)` umgesetzt für:
  - SD-Logger (Record-Pfad + Task-Init + Laufzeit-Loop).
  - SD-OTA (Availability-Check + Update-Entry).
  - SD-runtime_config-Overrides (`softConfigLoadOverrides`).
- Board-Profile erweitert um SD-Detect/Pin-Gruppe:
  - `SD_DETECT_PIN`, `SD_DETECT_ACTIVE_LOW`.
  - `FEAT_PINS_SD`, `FEAT_PINS_SD_COUNT`.
- Boot-Reihenfolge in `main.cpp` angepasst:
  - `modulesInit()` + Modulaktivierung (inkl. SD-Gating) vor SD-OTA/runtime_config-SD-Zugriffen.

## Verifikation / Checks
- `pio run -e gnss_buildup`
- `pio run -e gnss_bringup_ntrip`
- `rg "MOD_SD|SD_DETECT_PIN|hal_sd_card_present|moduleIsActive\(MOD_SD\)" src include`

## Hinweise / Risiken
- `SD_DETECT_PIN` wurde boardseitig auf GPIO 39 mit active-low modelliert; das setzt passende Verdrahtung des Card-Detect-Signals voraus.
- Für Boards ohne Detect-Pin ist in der HAL ein expliziter Fallback (`assume PRESENT`) vorhanden, um Verhalten klar zu halten.

## Offene Punkte
- Hardware-Validierung mit SD gesteckt/gezogen (Boot-Logs + OTA/Logger/RuntimeConfig-Gating) steht noch aus.
