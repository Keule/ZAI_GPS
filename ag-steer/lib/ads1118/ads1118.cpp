/**
 * @file ads1118.cpp
 * @brief ADS1118 16-Bit ADC driver – implementation.
 *
 * Handles the ADS1118's unique SPI protocol:
 *   - 16-bit simultaneous config+data per CS cycle
 *   - Auto-detects SPI mode (Mode0 / Mode1)
 *   - Auto-detects bit-inverted DOUT (cheap level-shifters)
 *   - DOUT connectivity test (crosstalk / floating detection)
 */

#include "ads1118.h"
#include <cstdarg>

// ===================================================================
// Constructor
// ===================================================================

ADS1118::ADS1118(SPIClass& spi)
    : _spi(spi)
{
    // Build default config: AIN0, PGA 4.096V, single-shot, 128 SPS
    _config = (1u << 15)   // OS: start single-shot
            | (0u << 12)   // MUX: AIN0 vs GND
            | (1u <<  9)   // PGA: +/-4.096V
            | (1u <<  8)   // MODE: single-shot
            | (4u <<  5)   // DR: 128 SPS
            | (0u <<  4)   // CM: standard
            | (0u <<  3)   // TS: ADC mode
            | (0u <<  2)   // POL: MSB first
            | (3u <<  0);  // CQ: comparator disabled
    // = 0x8383
}

// ===================================================================
// begin()
// ===================================================================

void ADS1118::begin(int cs_pin,
                     const int* deselect_pins,
                     int num_deselect,
                     void (*deselect_all_fn)(void))
{
    _cs_pin = cs_pin;
    _deselect_all_fn = deselect_all_fn;

    // Store deselect pins
    _num_deselect = 0;
    if (deselect_pins && num_deselect > 0) {
        int count = (num_deselect < MAX_DESELECT) ? num_deselect : MAX_DESELECT;
        for (int i = 0; i < count; i++) {
            _deselect_pins[_num_deselect++] = deselect_pins[i];
        }
    }

    // Configure CS pin
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);

    // Deselect all other SPI devices immediately
    deselectOthers();

    // Small delay for bus settling
    delay(1);
}

// ===================================================================
// detect() – auto-detect SPI mode and bit-inversion
// ===================================================================

bool ADS1118::detect(void (*log_fn)(const char*, ...))
{
    if (_cs_pin < 0) return false;

    deselectOthers();

    // === DOUT CONNECTIVITY TEST ===
    checkDoutConnection();

    deselectOthers();

    // === Try both SPI modes, check normal and inverted values ===

    struct ModeResult {
        uint8_t mode;
        const char* name;
        int16_t raw;
        bool valid;
        bool inverted;
    };

    ModeResult results[2] = {
        { SPI_MODE0, "Mode0", 0, false, false },
        { SPI_MODE1, "Mode1", 0, false, false },
    };

    for (int i = 0; i < 2; i++) {
        uint8_t mode = results[i].mode;

        // Step 1: send config, start conversion (ignore garbage result)
        _spi.beginTransaction(SPISettings(SPI_SPEED_SLOW, MSBFIRST, mode));
        digitalWrite(_cs_pin, LOW);
        _spi.transfer(static_cast<uint8_t>((_config >> 8) & 0xFF));
        _spi.transfer(static_cast<uint8_t>(_config & 0xFF));
        digitalWrite(_cs_pin, HIGH);
        _spi.endTransaction();

        // Step 2: wait for conversion
        delay(_conv_ms + 3);

        // Step 3: read conversion result
        _spi.beginTransaction(SPISettings(SPI_SPEED_SLOW, MSBFIRST, mode));
        digitalWrite(_cs_pin, LOW);
        uint8_t msb = _spi.transfer(static_cast<uint8_t>((_config >> 8) & 0xFF));
        uint8_t lsb = _spi.transfer(static_cast<uint8_t>(_config & 0xFF));
        digitalWrite(_cs_pin, HIGH);
        _spi.endTransaction();

        int16_t raw     = static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
        int16_t raw_inv = static_cast<int16_t>(~raw);

        // Choose: normal or bit-inverted?
        bool raw_crosstalk = isCrosstalk(raw);
        bool inv_crosstalk = isCrosstalk(raw_inv);
        bool raw_ok = (raw != static_cast<int16_t>(0xFFFF)) && !raw_crosstalk;
        bool inv_ok = (raw_inv != static_cast<int16_t>(0xFFFF)) && !inv_crosstalk;

        // Prefer inverted if raw is crosstalk but inverted is clean,
        // OR if inverted is positive while raw is negative (0-3.3V poti).
        bool use_inverted = false;
        if (inv_ok && !raw_ok) {
            use_inverted = true;
        } else if (raw_ok && inv_ok) {
            if (raw < 0 && raw_inv >= 0) use_inverted = true;
        }

        results[i].raw      = use_inverted ? raw_inv : raw;
        results[i].inverted = use_inverted;
        results[i].valid    = (results[i].raw != static_cast<int16_t>(0xFFFF));

        float voltage = static_cast<float>(results[i].raw) * _lsb_v;

        // Log result
        if (log_fn) {
            log_fn("ADS1118 %s%s: raw=%d (0x%04X), voltage=%.4fV %s",
                   results[i].name,
                   use_inverted ? " (inv)" : "",
                   results[i].raw,
                   static_cast<uint16_t>(results[i].raw),
                   voltage,
                   results[i].valid ? "OK" : "BAD");
        } else {
            Serial.printf("ADS1118 %s%s: raw=%d (0x%04X), voltage=%.4fV %s\n",
                          results[i].name,
                          use_inverted ? " (inv)" : "",
                          results[i].raw,
                          static_cast<uint16_t>(results[i].raw),
                          voltage,
                          results[i].valid ? "OK" : "BAD");
        }
    }

    // === Pick the best result (Mode0 preferred) ===
    for (int i = 0; i < 2; i++) {
        if (results[i].valid) {
            _spi_mode     = results[i].mode;
            _last_raw     = results[i].raw;
            _invert_dout  = results[i].inverted;
            _detected     = true;
            _last_conv_start = millis();

            float voltage = static_cast<float>(_last_raw) * _lsb_v;
            const char* mode_name = (_spi_mode == SPI_MODE0) ? "Mode0" : "Mode1";
            const char* inv_str   = _invert_dout ? ", invert-DOUT" : "";

            if (log_fn) {
                log_fn("ADS1118 DETECTED (%s%s, raw=%d, %.4fV)",
                       mode_name, inv_str, _last_raw, voltage);
            } else {
                Serial.printf("ADS1118 DETECTED (%s%s, raw=%d, %.4fV)\n",
                              mode_name, inv_str, _last_raw, voltage);
            }
            return true;
        }
    }

    // === DETECTION FAILED ===
    _detected = false;

    bool any_crosstalk = false;
    bool any_floating  = false;
    for (int i = 0; i < 2; i++) {
        uint16_t u = static_cast<uint16_t>(results[i].raw);
        if (u == 0xFFFF) any_floating = true;
        if (isCrosstalk(results[i].raw)) any_crosstalk = true;
    }

    if (log_fn) {
        if (any_crosstalk) {
            log_fn("ADS1118 DETECT FAILED: crosstalk – check DIN/DOUT wiring");
        } else if (any_floating) {
            log_fn("ADS1118 DETECT FAILED: no response (0xFFFF) – check connections");
        } else {
            log_fn("ADS1118 DETECT FAILED: unknown response");
        }
    } else {
        if (any_crosstalk) {
            Serial.println("ADS1118 DETECT FAILED: crosstalk – check DIN/DOUT wiring");
        } else if (any_floating) {
            Serial.println("ADS1118 DETECT FAILED: no response (0xFFFF) – check connections");
        } else {
            Serial.println("ADS1118 DETECT FAILED: unknown response");
        }
    }

    return false;
}

// ===================================================================
// Configuration
// ===================================================================

void ADS1118::setPGA(uint16_t pga) {
    _config = (_config & ~0x0E00u) | pga;

    // Update FSR and LSB based on PGA setting
    // PGA is shifted left by 9, extract the 3-bit value
    uint16_t pga_bits = (pga >> 9) & 0x07;
    switch (pga_bits) {
        case 0: _fsr = 6.144f; break;   // +/-6.144V
        case 1: _fsr = 4.096f; break;   // +/-4.096V
        case 2: _fsr = 2.048f; break;   // +/-2.048V
        case 3: _fsr = 1.024f; break;   // +/-1.024V
        case 4: _fsr = 0.512f; break;   // +/-0.512V
        case 5: _fsr = 0.256f; break;   // +/-0.256V
        default: _fsr = 4.096f; break;
    }
    _lsb_v = _fsr / 32768.0f;
}

void ADS1118::setTempSensorMode(bool enable) {
    if (enable) {
        _config |= (1u << 3);   // TS = 1 (temperature sensor)
    } else {
        _config &= ~(1u << 3);  // TS = 0 (ADC mode)
    }
}

// ===================================================================
// Reading – blocking
// ===================================================================

float ADS1118::readSingleEnded(uint8_t channel) {
    if (!_detected) return 0.0f;
    int16_t raw = readRaw(channel);
    return static_cast<float>(raw) * _lsb_v;
}

float ADS1118::readDifferential(uint8_t positive, uint8_t negative) {
    if (!_detected) return 0.0f;

    // Differential mux encoding:
    // 100: AIN0-AIN1, 101: AIN0-AIN3, 110: AIN1-AIN3, 111: AIN2-AIN3
    uint16_t mux;
    if (positive == 0 && negative == 1) mux = Ads1118Reg::MUX_AIN0_AIN1;
    else if (positive == 0 && negative == 3) mux = Ads1118Reg::MUX_AIN0_AIN3;
    else if (positive == 1 && negative == 3) mux = Ads1118Reg::MUX_AIN1_AIN3;
    else if (positive == 2 && negative == 3) mux = Ads1118Reg::MUX_AIN2_AIN3;
    else return 0.0f;  // invalid differential pair

    deselectOthers();

    // Build config with differential mux
    uint16_t cfg = (_config & ~0x7000u) | mux;

    // Ensure TS = 0 (ADC mode)
    cfg &= ~(1u << 3);

    // Send config, start conversion
    transaction(cfg);
    delay(_conv_ms + 1);

    // Read result
    int16_t result = transaction(cfg);
    if (result != static_cast<int16_t>(0xFFFF)) {
        _last_raw = result;
    }

    return static_cast<float>(_last_raw) * _lsb_v;
}

int16_t ADS1118::readRaw(uint8_t channel) {
    if (!_detected) return 0;

    deselectOthers();

    // Build config for this channel
    uint16_t cfg = buildConfig(channel);

    // Send config, start conversion
    transaction(cfg);

    // Wait for conversion to complete
    delay(_conv_ms + 1);

    // Read result
    int16_t result = transaction(cfg);

    if (result != static_cast<int16_t>(0xFFFF)) {
        _last_raw = result;
    }
    return _last_raw;
}

float ADS1118::readTemperature() {
    if (!_detected) return 0.0f;

    deselectOthers();

    uint16_t cfg = buildTempConfig();

    // Send config
    transaction(cfg);
    delay(_conv_ms + 1);

    // Read temperature result
    int16_t raw = transaction(cfg);
    if (raw == static_cast<int16_t>(0xFFFF)) return 0.0f;

    // Temperature conversion (from datasheet):
    // 0°C = 0x0000, each LSB = 0.03125°C
    // Note: DOUT inversion applies to temperature too!
    return static_cast<float>(raw) * 0.03125f;
}

// ===================================================================
// Reading – non-blocking
// ===================================================================

bool ADS1118::conversionReady() const {
    return (millis() - _last_conv_start) >= _conv_ms;
}

void ADS1118::startConversion(uint8_t channel) {
    if (!_detected || _cs_pin < 0) return;

    deselectOthers();

    uint16_t cfg = buildConfig(channel);
    transaction(cfg);
}

int16_t ADS1118::readConversionResult() {
    if (!_detected) return 0;

    // Wait if conversion not ready
    while (!conversionReady()) {
        delay(1);
    }

    deselectOthers();

    uint16_t cfg = buildConfig(0);  // channel already set by startConversion
    int16_t result = transaction(cfg);

    if (result != static_cast<int16_t>(0xFFFF)) {
        _last_raw = result;
    }
    return _last_raw;
}

int16_t ADS1118::readLoop(uint8_t channel) {
    if (!_detected) return _last_raw;

    if (conversionReady()) {
        deselectOthers();

        uint16_t cfg = buildConfig(channel);
        int16_t result = transaction(cfg);

        if (result != static_cast<int16_t>(0xFFFF)) {
            _last_raw = result;
        }
    }
    return _last_raw;
}

// ===================================================================
// Internal: transaction()
// ===================================================================

int16_t ADS1118::transaction(uint16_t config) {
    if (_cs_pin < 0) return static_cast<int16_t>(0xFFFF);

    _spi.beginTransaction(SPISettings(SPI_SPEED_NORMAL, MSBFIRST, _spi_mode));
    digitalWrite(_cs_pin, LOW);

    // 16-bit simultaneous transfer: config OUT, result IN
    uint8_t msb = _spi.transfer(static_cast<uint8_t>((config >> 8) & 0xFF));
    uint8_t lsb = _spi.transfer(static_cast<uint8_t>(config & 0xFF));

    digitalWrite(_cs_pin, HIGH);
    _spi.endTransaction();

    _last_conv_start = millis();

    // Apply bit inversion if the module's DOUT line is inverted
    if (_invert_dout) {
        msb = ~msb;
        lsb = ~lsb;
    }

    return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
}

// ===================================================================
// Internal: buildConfig()
// ===================================================================

uint16_t ADS1118::buildConfig(uint8_t channel) const {
    uint16_t cfg = _config;

    // Set MUX bits for single-ended channel
    uint16_t mux = 0;
    switch (channel & 0x03) {
        case 0: mux = Ads1118Reg::MUX_AIN0_GND; break;
        case 1: mux = Ads1118Reg::MUX_AIN1_GND; break;
        case 2: mux = Ads1118Reg::MUX_AIN2_GND; break;
        case 3: mux = Ads1118Reg::MUX_AIN3_GND; break;
    }

    cfg = (cfg & ~0x7000u) | mux;

    // Ensure TS = 0 (ADC mode)
    cfg &= ~(1u << 3);

    return cfg;
}

uint16_t ADS1118::buildTempConfig() const {
    uint16_t cfg = _config;
    // Set TS = 1 (temperature mode)
    cfg |= (1u << 3);
    return cfg;
}

// ===================================================================
// Internal: checkDoutConnection()
// ===================================================================

bool ADS1118::checkDoutConnection() {
    // Send 0x55 pattern on MOSI.
    // Correct wiring: ADS1118 drives its own data on DOUT (not 0x55).
    // Swapped wiring: MISO picks up MOSI via crosstalk (~0x55).
    // No device: MISO reads 0xFF (floating).

    _spi.beginTransaction(SPISettings(SPI_SPEED_SLOW, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs_pin, LOW);
    uint8_t b1 = _spi.transfer(0x55);
    uint8_t b2 = _spi.transfer(0x55);
    digitalWrite(_cs_pin, HIGH);
    _spi.endTransaction();

    bool crosstalk = (b1 == 0x55 || b2 == 0x55);
    bool floating  = (b1 == 0xFF && b2 == 0xFF);

    if (crosstalk) {
        Serial.printf("ADS1118 DOUT test: CROSSTALK (read 0x%02X 0x%02X) – DIN/DOUT swapped!\n",
                      b1, b2);
        return false;
    } else if (floating) {
        Serial.printf("ADS1118 DOUT test: FLOATING (0xFF 0xFF) – no device on DOUT\n");
        return false;
    } else {
        Serial.printf("ADS1118 DOUT test: OK (read 0x%02X 0x%02X)\n", b1, b2);
        return true;
    }
}

// ===================================================================
// Internal: isCrosstalk()
// ===================================================================

bool ADS1118::isCrosstalk(int16_t raw) {
    uint16_t uval = static_cast<uint16_t>(raw);
    uint16_t lo = uval & 0x00FFu;
    uint16_t hi = (uval >> 8) & 0x00FFu;

    // Config word bytes appearing in the reading (0x83 = low byte of default config)
    if (lo == 0x83 || hi == 0x83) return true;

    // Continuous-mode config echo (0x03 in high byte)
    if (hi == 0x03) return true;

    // Test pattern echo
    if (uval == 0x5555 || uval == 0x0000) return true;

    return false;
}

// ===================================================================
// Internal: deselectOthers()
// ===================================================================

void ADS1118::deselectOthers() {
    // Use callback if provided (handles all devices at once)
    if (_deselect_all_fn) {
        _deselect_all_fn();
        return;
    }

    // Otherwise deselect stored pin list
    for (int i = 0; i < _num_deselect; i++) {
        digitalWrite(_deselect_pins[i], HIGH);
    }
}
