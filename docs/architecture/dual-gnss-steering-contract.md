# Dual-GNSS Bridge → Steering-Core Vertrag (UM980/UM982)

## 1) Firmware-Ziel (klar getrennt)

**Bridge-Firmware Ziel:**
- Nimmt zwei unabhängige GNSS-Quellen (2x UM980/UM982) entgegen.
- Validiert und bewertet beide Streams robust.
- Liefert **nur steering-relevante, bereits validierte Navigationsdaten** an den Steering-Core.
- Kümmert sich um Parser-/Transport-/Receiver-spezifische Details.

**Steering-Core Ziel:**
- Verwendet ausschließlich den minimalen GNSS-Vertrag (`SteerCoreGnssSample`).
- Enthält **keine Kenntnis** über UART-Ports, NMEA-Sätze, UBX/Unicore oder Failover-Strategien.
- Entscheidet nur über Lenklogik/Safety auf Basis des Vertrags.

Damit bleibt die Abhängigkeit strikt einseitig: **Bridge → Contract → Steering-Core**.

## 2) Minimaler Schnittstellenvertrag zum Steering-Core

Datei: `src/logic/gnss_contract.h`

`SteerCoreGnssSample` ist der einzige notwendige Übergabedatensatz:
- Zeitstempel (Bridge-Monotonic)
- Position (`lat/lon`)
- Heading, Speed, Roll
- Qualitätsindikatoren (`hdop`, `sat_count`, `quality_code`)
- selektierte Quelle (`source_index`)
- Steering-Freigabe (`valid_for_steering`)

Keine bridge-internen Felder sind Teil dieses Vertrags.

## 3) Robustes Datenvalidierungsmodell für Dual-GNSS

Dateien: `src/logic/gnss_dual_input.h/.cpp`

Validierung pro Receiver-Observation:
1. **Finite-Check**: Keine NaN/Inf-Werte.
2. **Range-Check**:
   - Latitude `[-90, 90]`
   - Longitude `[-180, 180]`
   - Heading `[0, 360)`
   - Speed `[0, 60] m/s`
   - Roll `|roll| <= 45°`
3. **Geometrie/Qualität**:
   - Frame-CRC muss gültig sein
   - Fix darf nicht `NONE` sein
   - `sat_count >= 6`
   - `hdop <= 3.5`
4. **Freshness**:
   - Sample-Alter `<= 250 ms`

Nur wenn alle Gates erfüllt sind: `valid_for_steering = true`.

### Source-Selektion / Failover

Beide validen Quellen werden gescored:
- höherer Fix-Typ (RTK fixed > float > single)
- mehr Satelliten
- niedrigerer HDOP
- niedrigeres Differentialalter

Die beste Quelle wird in den Steering-Vertrag gemappt (`source_index`).
Bei Ausfall/Ungültigkeit der führenden Quelle erfolgt automatischer Fallback auf die zweite Quelle.

## 4) Welche Daten in welcher Frequenz an Steering gehen

Aus `gnss_contract.h`:
- Soll-Frequenz an Steering-Core: **20 Hz** (`STEERING_GNSS_OUTPUT_HZ`)
- Maximal zulässiges Alter für verwendete Daten: **250 ms** (`STEERING_GNSS_MAX_AGE_MS`)

Übertragene Felder (pro Zyklus):
- `timestamp_ms`
- `latitude_deg`, `longitude_deg`
- `heading_deg`, `speed_mps`, `roll_deg`
- `hdop`, `sat_count`, `quality_code`
- `source_index`
- `valid_for_steering`

## 5) Unabhängigkeit des Steering-Core von Bridge-Details

Technische Maßnahmen:
- Steering-Core sieht nur `gnss_contract::SteerCoreGnssSample`.
- Dual-GNSS-spezifische Heuristiken leben vollständig in `gnss_dual`.
- Keine UM980/UM982-Typen im Vertragsobjekt.
- Austausch der Bridge-Implementierung ist ohne Änderung am Steering-Core möglich,
  solange `SteerCoreGnssSample` semantisch stabil bleibt.
