/**
 * @file sen6x_core.h
 * @brief Core definitions and API for Sensirion SEN6x environmental sensor family.
 *
 * Provides type definitions, constants, and class declarations for communicating
 * with SEN6x devices (SEN63C / SEN65 / SEN66 / SEN68) over I²C. SEN60 may not be supported as it
 * hasa differnet I2C address and may not play well with the meshtastic system
 *
 * Based on: Sensirion SEN6x Datasheet – Version 0.91 (August 2025)
 *           https://sensirion.com/media/documents/FAFC548D/68C12881/Sensirion_Datasheet_SEN6x.pdf
 *
 * Defines:
 *   - CRC parameters and helper functions
 *   - Device status flags and parsing structures
 *   - Core class for I²C command execution
 *
 * @author  Arvind S.A.
 * @date    2025-10-05
 */

/*
 * TODO:
 * - Move all commands to central Definition
 * - Central tracking of when next command can be made
 * - Family based splitting. Currently. Keep the API same . For
 */

#pragma once
#include <Arduino.h>
#include <Wire.h>

namespace sen6x
{

    struct Reading
    {
        // Populate with what your variant provides; placeholders included:
        // Typical fields for air-quality class devices:
        float voc_index = NAN;   // e.g., VOC index
        float nox_index = NAN;   // e.g., NOx index
        float humidity = NAN;    // %RH
        float temperature = NAN; // °C
        uint32_t timestamp_ms = 0;
        bool valid = false;
        uint8_t last_error = 0; // 0 = OK; else I2C/parse error
    };

    enum class Status : uint8_t
    {
        OK = 0,
        I2C_ERROR,
        CRC_ERROR,
        TIMEOUT,
        INVALID_ARG,
        NOT_INITIALIZED
    };

    struct DeviceStatus
    {
        bool fan_speed_warning = false;
        bool fan_error = false;
        bool rht_error = false;
        bool gas_error = false;
        bool co2_2_error = false;
        bool hcho_error = false;
        bool pm_error = false;
        bool co2_1_error = false;

        // optional: store the raw 32-bit value
        uint32_t raw = 0;
    };

    struct SEN66Measured
    {
        float pm1_0;       ///< µg/m³
        float pm2_5;       ///< µg/m³
        float pm4_0;       ///< µg/m³
        float pm10_0;      ///< µg/m³
        float humidity;    ///< %RH
        float temperature; ///< °C
        float voc_index;   ///< unitless index
        float nox_index;   ///< unitless index
        float co2;         ///< ppm
    };

    class Core
    {
    public:
        explicit Core(TwoWire &w = Wire, uint8_t addr = 0x6B) // default addr placeholder
            : wire_(w), address_(addr)
        {
        }

        /**
         * @brief Initialize I²C communication with the SEN6x sensor.
         *
         * Probes the configured I²C address to verify sensor presence and
         * marks the driver as initialized if communication succeeds.
         *
         * @return Status::OK on success,
         *         Status::I2C_ERROR if the device did not acknowledge.
         */
        Status begin();

        /**
         * @brief Read the 32-bit Device Status Register (DSR) from the sensor.
         *
         * Sends command 0xD206 to retrieve the DSR, waits for execution,
         * and reads two 16-bit words (MSW + LSW), each protected by a CRC.
         * The combined 32-bit result contains all warning and error flags.
         *
         * Bit mapping reference: SEN6x Datasheet Section 4.3.
         *
         * @param[out] out_status 32-bit raw DSR value (MSW << 16 | LSW).
         * @return Status::OK on success,
         *         Status::I2C_ERROR or CRC_ERROR on communication failure,
         *         Status::NOT_INITIALIZED if begin() has not been called.
         */
        Status readDeviceStatus(uint32_t &out_status);

        /**
         * @brief Retrieve and decode the Device Status Register into structured flags.
         *
         * Reads the raw 32-bit DSR from the sensor and converts it into
         * a DeviceStatus structure with individual warning and error fields.
         *
         * @param[out] out Parsed DeviceStatus structure with decoded flags.
         * @return Status::OK on success,
         *         Status::I2C_ERROR or CRC_ERROR on communication failure.
         *
         * @see Core::readDeviceStatus()
         * @see decodeDSR()
         */
        Status getDeviceStatus(DeviceStatus &out);

        /**
         * @brief Read and then clear the Device Status Register (DSR).
         *
         * Issues command 0xD210 (“Read And Clear Device Status”) to retrieve the
         * current 32-bit status flags and then clear them on the device.
         *
         * Applies to: SEN63C, SEN65, SEN66, SEN68
         * Available in: Idle and Measurement modes
         * Execution time: ~20 ms
         * RX: 2 words (MSW, LSW), each with CRC
         *
         * @param[out] out_status 32-bit DSR value *before* clearing (MSW<<16 | LSW).
         * @return Status::OK on success,
         *         Status::I2C_ERROR or CRC_ERROR on failure,
         *         Status::NOT_INITIALIZED if begin() wasn’t called.
         *
         * @note Flags are cleared on the device after this read. If you need to keep
         *       them, store @p out_status locally before the next status operation.
         */
        Status readAndClearDeviceStatus(uint32_t &out_status);

        /**
         * @brief Convenience API: read+clear DSR and decode into structured flags.
         *
         * @param[out] out Parsed DeviceStatus (represents DSR *before* clearing).
         * @return Status::OK on success or an error status on failure.
         * @see readAndClearDeviceStatus(), decodeDSR()
         */
        Status getAndClearDeviceStatus(DeviceStatus &out);

        // Soft reset (device specific command id to be filled)
        Status softReset();

        /**
         * @brief Start continuous measurement (SEN6x).
         *
         * Sends command 0x0021 (“Start Continuous Measurement”) to begin periodic
         * sampling. The device needs approximately 1.1 seconds before the first
         * valid results become available. This time is stored in @ref dataReadyBy_
         * for later checks via isDataReadyByTime().
         *
         * Applies to: SEN63C, SEN65, SEN66, SEN68
         * Available in: Idle mode only
         * Execution time: ~50 ms
         *
         * @note For SEN63C, the CO₂ sensor performs a 24 s conditioning phase
         *       after startup; avoid restarting or recalibrating during this period.
         *
         * @return Status::OK on success,
         *         Status::NOT_INITIALIZED if begin() was not called,
         *         or I²C error status from sendCommand().
         */
        Status startMeasurement();

        /**
         * @brief Stop continuous measurement and return the device to idle mode.
         *
         * Sends command 0x0104 (“Stop Measurement”) to halt continuous measurement.
         * After issuing this command, the device requires at least 1000 ms
         * before a new measurement can be started.
         *
         * Applies to: SEN63C, SEN65, SEN66, SEN68
         * Available in: Measurement mode
         * Execution time: ~1000 ms
         *
         * @return Status::OK on success,
         *         Status::NOT_INITIALIZED if begin() was not called,
         *         or an I²C communication error code from sendCommand().
         *
         * @note Always wait ≥1 s after this command before calling startMeasurement()
         *       again to avoid CO₂-1 sensor errors on SEN63C devices.
         */
        Status stopMeasurement();

        /**
         * @brief Query whether new measurement data is ready.
         *
         * Sends command 0x0202 (“Get Data Ready”) and returns a one-byte flag:
         *  - 0x01 → data ready
         *  - 0x00 → not ready (or no measurement running)
         *
         * Applies to: SEN63C, SEN65, SEN66, SEN68
         * Available in: Measurement mode
         * Execution time: ~20 ms
         * RX: 2 data bytes (padding=0x00, ready flag) + CRC
         *
         * @param[out] out_ready 0x01 if data is ready; 0x00 otherwise.
         * @return Status::OK on success,
         *         Status::NOT_INITIALIZED if begin() wasn’t called,
         *         Status::I2C_ERROR / CRC_ERROR on communication or checksum failure.
         */
        Status getDataReady(uint8_t &out_ready);

        /**
         * @brief Read measured values from SEN66 (PM, RH, T, VOC index, NOx index, CO₂).
         *
         * Issues command 0x0300 (“Read Measured Values SEN66”), waits ~20 ms, then reads
         * 9 words (each with CRC) in this order:
         *   0: PM1.0   (uint16, scale /10 → µg/m³; 0xFFFF = unknown)
         *   1: PM2.5   (uint16, scale /10 → µg/m³; 0xFFFF = unknown)
         *   2: PM4.0   (uint16, scale /10 → µg/m³; 0xFFFF = unknown)
         *   3: PM10.0  (uint16, scale /10 → µg/m³; 0xFFFF = unknown)
         *   4: Humidity(int16,  scale /100 → %RH;   0x7FFF = unknown)
         *   5: Temp    (int16,  scale /200 → °C;    0x7FFF = unknown)
         *   6: VOC idx (int16,  scale /10;          0x7FFF = unknown)
         *   7: NOx idx (int16,  scale /10;          0x7FFF = unknown; first 10–11 s after power/reset it is 0x7FFF)
         *   8: CO₂     (uint16, ppm;                0xFFFF = unknown; first 5–6 s after start it is 0xFFFF)
         *
         * Applies to: SEN66
         * Available in: Measurement mode
         * Execution time: ~20 ms
         * RX: 27 bytes total (9 × (MSB, LSB, CRC))
         *
         * @param[out] out Struct of scaled float values; unknowns are set to NAN.
         * @return Status::OK on success,
         *         Status::NOT_INITIALIZED if begin() wasn’t called,
         *         Status::I2C_ERROR / CRC_ERROR on communication/checksum failure.
         */
        Status readMeasurement(SEN66Measured &out);

        // Optional helpers you may need:
        Status readSerialNumber(String &out_serial);
        Status readProductName(String &out_name);
        Status setAveraging(uint8_t level); // device dependent
        Status setHeater(bool enable);      // if your variant has it

        // Low-level: send a 16-bit command + optional 16-bit data words (each with CRC)
        Status sendCommand(uint16_t cmd, const uint16_t *words = nullptr, size_t word_count = 0);

        // Low-level: read N 16-bit words (each followed by CRC byte) into buffer
        Status readWords(uint16_t *out_words, size_t word_count, uint32_t wait_ms = 0);

        uint8_t address() const { return address_; }
        void setAddress(uint8_t addr) { address_ = addr; }
        void setDebug(Print *p) { dbg_ = p; }

        /**
         * @brief Check whether the first data sample should be ready by time.
         *
         * Returns true once the current time has reached or passed the timestamp
         * recorded when startMeasurement() was called (≈1.1 s after start).
         *
         * @return true if data should be ready by time, false otherwise.
         */
        bool isDataReadyByTime() const; ///< Returns true if current time >= dataReadyBy_.

        /**
         * @brief Trigger fan-cleaning cycle (max speed for ~10 s).
         *
         * Sends command 0x5607 (“Start Fan Cleaning”). The device must be in Idle.
         * After issuing this command, wait at least 10 seconds before starting
         * a new measurement.
         *
         * Applies to: SEN63C, SEN65, SEN66, SEN68
         * Available in: Idle mode
         * Execution time: ~20 ms
         * TX/RX: none
         *
         * @return Status::OK on success,
         *         Status::NOT_INITIALIZED if begin() wasn’t called,
         *         or an I²C error from sendCommand().
         */
        Status startFanCleaning();

    private:
        TwoWire &wire_;
        uint8_t address_;
        bool initialized_ = false;
        Print *dbg_ = nullptr;

        /**
         * @brief  Timestamp (ms) when data will first be ready after startMeasurement().
         */
        unsigned long dataReadyBy_ = 0;

        /**
         * @brief Calculate 8-bit CRC (Sensirion standard, poly 0x31, init 0xFF).
         *
         * Computes the CRC used by SEN6x sensors for data integrity over I²C.
         * Each 16-bit word is checked against this CRC.
         *
         * @param data Pointer to input bytes.
         * @param len  Number of bytes.
         * @return Computed CRC byte.
         *
         * @see Reference: Datasheet Section 4.9 Checksum Calulation
         */
        static uint8_t crc8(const uint8_t *data, size_t len); // Sensirion CRC: poly 0x31, init 0xFF
        Status tx_(const uint8_t *bytes, size_t len, bool sendStop = true);
        Status rx_(uint8_t *bytes, size_t len, uint32_t wait_ms = 0);
    };

} // namespace sen6x

namespace sen6x_bits
{
    constexpr uint32_t WARN_SPEED = (1u << 21); // “Warning SPEED” (from your figure)
    constexpr uint32_t ERR_FAN = (1u << 5);
    constexpr uint32_t ERR_RHT = (1u << 6);
    constexpr uint32_t ERR_GAS = (1u << 7);
    constexpr uint32_t ERR_CO2_2 = (1u << 8);
    constexpr uint32_t ERR_HCHO = (1u << 10);
    constexpr uint32_t ERR_PM = (1u << 11);
    constexpr uint32_t ERR_CO2_1 = (1u << 12);
    // bits 0..4, 9, 13..20, 22..31 look reserved in your image
}