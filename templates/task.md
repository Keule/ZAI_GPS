# Task Template

## Zweck
Einheitliche Beschreibung einer umsetzbaren Arbeitseinheit.

## Minimalpflichtfelder
- **Titel:**
- **Kontext/Problem:**
- **Akzeptanzkriterien:**
- **Scope (in):**
- **Nicht-Scope (out):**
- **Verifikation/Test:**
- **Owner:**
- **Status:** `todo | in_progress | done`

## Vorlage
```md
# Task: <Titel>
- Kontext/Problem:
- Akzeptanzkriterien:
- Scope (in):
- Nicht-Scope (out):
- Verifikation/Test:
- Owner:
- Status: todo
```

## Beispiel
```md
# Task: Discovery/Core CRC-Tests trennen
- Kontext/Problem: Discovery- und Core-Pfade sind in einem Test vermischt.
- Akzeptanzkriterien: Zwei getrennte Testgruppen; beide grün in der Matrix.
- Scope (in): `src/logic/pgn_codec.*`, Host-Smoke-Tests.
- Nicht-Scope (out): API-Redesign von `pgn_types.h`.
- Verifikation/Test: `python3 tools/run_test_matrix.py`.
- Owner: Firmware-Team
- Status: in_progress
```


## Template-Snippet: Nacharbeits-Task
```md
## Nacharbeit (bei Prozessabweichung)
- Ursache:
- Betroffene Task-ID/PR:
- Korrekturmaßnahme:
- Präventionsmaßnahme:
```
