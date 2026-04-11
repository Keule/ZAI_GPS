/**
 * @file ads1118.h
 * @brief ADS1118 16-Bit ADC driver library for Arduino / ESP32.
 *
 * Supports the ADS1118's unique SPI protocol:
 *   - 16-bit simultaneous config+data per CS cycle
 *   - Config sent to DIN while previous conversion result received from DOUT
 *   - CS HIGH starts new conversion
 *
 * Features:
 *   - Automatic SPI mode detection (Mode 0 / Mode 1)
 *   - Automatic bit-inversion detection for modules with inverting
 *     level-shifters on the DOUT line
 *   - DOUT connectivity test (crosstalk / floating detection)
 *   - Shared SPI bus support (deselect other devices before transactions)
 *   - Single-shot and continuous conversion modes
 *   - Internal temperature sensor reading
 *
 * Pin mapping:
 *   ADS1118 DOUT  -> MCU MISO
 *   ADS1118 DIN   -> MCU MOSI
 *   ADS1118 SCLK  -> MCU SCK
 *   ADS1118 CS    -> MCU GPIO (active LOW)
 *
 * Based on TI ADS1118 datasheet: SLASB73
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>

// ===================================================================
// ADS1118 Register & Config constants
// ===================================================================

namespace Ads1118Reg {

    /// Input multiplexer configuration
    enum Mux : uint16_t {
        MUX_AIN0_GND   = 0b000 << 12,
        MUX_AIN1_GND   = 0b001 << 12,
        MUX_AIN2_GND   = 0b010 << 12,
        MUX_AIN3_GND   = 0b011 << 12,
        MUX_AIN0_AIN1  = 0b100 << 12,
        MUX_AIN0_AIN3  = 0b101 << 12,
        MUX_AIN1_AIN3  = 0b110 << 12,
        MUX_AIN2_AIN3  = 0b111 << 12,
    };

    /// Programmable Gain Amplifier (full-scale range)
    enum PGA : uint16_t {
        PGA_6144   = 0b000 << 9,   ///< +/-6.144V
        PGA_4096   = 0b001 << 9,   ///< +/-4.096V
        PGA_2048   = 0b010 << 9,   ///< +/-2.048V
        PGA_1024   = 0b011 << 9,   ///< +/-1.024V
        PGA_0512   = 0b100 << 9,   ///< +/-0.512V
        PGA_0256   = 0b101 << 9,   ///< +/-0.256V
    };

    /// Data rate (samples per second)
    enum DataRate : uint16_t {
        DR_8SPS    = 0b000 << 5,   ///<   8 SPS
        DR_16SPS   = 0b001 << 5,   ///<  16 SPS
        DR_32SPS   = 0b010 << 5,   ///<  32 SPS
        DR_64SPS   = 0b011 << 5,   ///<  64 SPS
        DR_128SPS  = 0b100 << 5,   ///< 128 SPS (default)
        DR_250SPS  = 0b101 << 5,   ///< 250 SPS
        DR_475SPS  = 0b110 << 5,   ///< 475 SPS
        DR_860SPS  = 0b111 << 5,   ///< 860 SPS
    };

    /// Operating mode
    enum Mode : uint16_t {
        MODE_CONTINUOUS   = 0 << 8,
        MODE_SINGLE_SHOT  = 1 << 8,
    };

    /// Conversion status
    enum Status : uint16_t {
        STATUS_BUSY    = 0xFFFF,    ///< Conversion not ready / busy
        STATUS_VALID   = 0x0000,    ///< Conversion complete
    };

} // namespace Ads1118Reg


class ADS1118 {
public:
    /**
     * Construct ADS1118 driver for a specific SPI bus.
     * @param spi  Reference to the shared SPIClass (e.g. FSPI).
     */
    explicit ADS1118(SPIClass& spi);

    /**
     * Initialise the ADS1118 on a specific CS pin.
     *
     * Sets up the CS pin, deselects other SPI devices, and runs
     * a DOUT connectivity test.
     *
     * @param cs_pin            Chip-select GPIO (active LOW).
     * @param deselect_pins     Array of other CS pins to hold HIGH.
     * @param num_deselect      Number of entries in deselect_pins.
     * @param deselect_all_fn   Optional callback to deselect ALL other
     *                          devices (called before every transaction).
     */
    void begin(int cs_pin,
               const int* deselect_pins = nullptr,
               int num_deselect = 0,
               void (*deselect_all_fn)(void) = nullptr);

    /**
     * Auto-detect the ADS1118: SPI mode and DOUT bit-inversion.
     *
     * Tries SPI_MODE0 and SPI_MODE1, checks raw and bit-inverted values.
     * Stores the working mode and inversion flag internally.
     *
     * @param log_fn  Optional logging function (printf-style with va_list).
     *               If nullptr, uses Serial.printf.
     * @return true if ADS1118 was detected successfully.
     */
    bool detect(void (*log_fn)(const char*, ...) = nullptr);

    /// Returns true if ADS1118 was detected by detect().
    bool isDetected() const { return _detected; }

    /// Returns true if DOUT bit-inversion is active (auto-compensated).
    bool isDoutInverted() const { return _invert_dout; }

    /// Returns the auto-detected SPI mode (SPI_MODE0 or SPI_MODE1).
    uint8_t getSPIMode() const { return _spi_mode; }

    // ===================================================================
    // Configuration
    // ===================================================================

    /// Set input multiplexer (see ADS1118::Mux enum).
    void setMux(uint16_t mux) { _config = (_config & ~0x7000u) | mux; }

    /// Set PGA gain (see ADS1118::PGA enum). Updates _fsr and _lsb_v.
    void setPGA(uint16_t pga);

    /// Set data rate (see ADS1118::DataRate enum). Updates _conv_ms.
    void setDataRate(uint16_t dr) { _config = (_config & ~0x00E0u) | dr; }

    /// Set operating mode (single-shot or continuous).
    void setMode(uint16_t mode) { _config = (_config & ~0x0100u) | mode; }

    /// Enable temperature sensor mode.
    void setTempSensorMode(bool enable);

    /// Manually set DOUT inversion (overrides auto-detection).
    void setDoutInvert(bool invert) { _invert_dout = invert; }

    /// Get the current full-scale range in volts.
    float getFSR() const { return _fsr; }

    // ===================================================================
    // Reading
    // ===================================================================

    /**
     * Read a single-ended channel (AIN0-AIN3 vs GND).
     * @param channel  AIN channel (0-3).
     * @return Voltage in volts. Returns 0.0 if not detected.
     */
    float readSingleEnded(uint8_t channel);

    /**
     * Read a differential channel.
     * @param positive  Positive input (0-3).
     * @param negative  Negative input (0-3).
     * @return Voltage in volts.
     */
    float readDifferential(uint8_t positive, uint8_t negative);

    /**
     * Read raw 16-bit ADC value.
     * @param channel  AIN channel (0-3).
     * @return Raw signed 16-bit value, or 0 if not detected.
     */
    int16_t readRaw(uint8_t channel);

    /**
     * Read the internal temperature sensor.
     * @return Temperature in degrees Celsius.
     */
    float readTemperature();

    /**
     * Check if a new conversion result is ready.
     * @return true if conversion result is available.
     */
    bool conversionReady() const;

    /**
     * Start a single conversion on a channel (non-blocking).
     * @param channel  AIN channel (0-3).
     */
    void startConversion(uint8_t channel);

    /**
     * Read the result of the last conversion (blocking).
     * Waits for conversion to complete if necessary.
     * @return Raw signed 16-bit conversion result.
     */
    int16_t readConversionResult();

    /**
     * Non-blocking read for use in control loops.
     *
     * If enough time has passed since the last conversion start,
     * reads the result and starts a new conversion.
     * Otherwise returns the last known value.
     *
     * @param channel  AIN channel (0-3).
     * @return Raw signed 16-bit value.
     */
    int16_t readLoop(uint8_t channel);

    /// Get the last raw value.
    int16_t getLastRaw() const { return _last_raw; }

private:
    // ===================================================================
    // Internal methods
    // ===================================================================

    /**
     * Perform one 16-bit SPI transaction.
     *
     * Sends config to DIN, reads previous conversion result from DOUT.
     * CS HIGH starts a new conversion.
     *
     * @param config  Config word to send (16-bit).
     * @return 16-bit raw ADC value (0xFFFF = not ready).
     */
    int16_t transaction(uint16_t config);

    /**
     * Build a config word for ADC reading on a channel.
     * @param channel  AIN channel (0-3).
     * @return Complete 16-bit config word.
     */
    uint16_t buildConfig(uint8_t channel) const;

    /**
     * Build a config word for temperature sensor mode.
     * @return Complete 16-bit config word (TS bit = 1).
     */
    uint16_t buildTempConfig() const;

    /**
     * DOUT connectivity test.
     *
     * Sends 0x55 0x55 pattern on MOSI.
     * Correct wiring: ADS1118 drives its own data (not 0x55).
     * Swapped wiring: MISO picks up MOSI (reads ~0x55 0x55).
     * No device: MISO reads 0xFF 0xFF (floating).
     *
     * @return true if DOUT appears correctly connected.
     */
    bool checkDoutConnection();

    /**
     * Check if a raw ADC value looks like MOSI→MISO crosstalk.
     */
    static bool isCrosstalk(int16_t raw);

    /**
     * Deselect all other SPI devices.
     */
    void deselectOthers();

    // ===================================================================
    // Member variables
    // ===================================================================

    SPIClass& _spi;
    int _cs_pin = -1;

    static constexpr int MAX_DESELECT = 8;
    int _deselect_pins[MAX_DESELECT] = {};
    int _num_deselect = 0;
    void (*_deselect_all_fn)(void) = nullptr;

    uint16_t _config = 0;
    float _fsr = 4.096f;
    float _lsb_v = 0.000125f;
    uint32_t _conv_ms = 9;

    uint32_t _last_conv_start = 0;
    int16_t _last_raw = 0;

    uint8_t _spi_mode = SPI_MODE0;
    bool _detected = false;
    bool _invert_dout = false;

    static constexpr uint32_t SPI_SPEED_NORMAL = 1000000;
    static constexpr uint32_t SPI_SPEED_SLOW   = 200000;
};
