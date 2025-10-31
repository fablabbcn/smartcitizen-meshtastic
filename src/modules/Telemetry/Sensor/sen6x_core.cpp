/**
 * @file sen6x_core.cpp
 * @brief Core I²C driver implementation for Sensirion SEN6x environmental sensor family.
 *
 * This driver provides low-level communication, CRC checking, and command handling
 * for SEN6x devices (SEN63C / SEN65 / SEN66 / SEN68) over the I²C interface.
 *
 * Based on: Sensirion SEN6x Datasheet – Version 0.91 (August 2025)
 *           https://sensirion.com/media/documents/FAFC548D/68C12881/Sensirion_Datasheet_SEN6x.pdf
 *
 * Implements:
 *   - Standard I²C command protocol
 *   - 8-bit CRC (polynomial 0x31, init 0xFF)
 *   - Device Status Register parsing
 *   - Basic measurement and configuration commands
 *
 * @author  Arvind S.A.
 * @date    2025-10-05
 */

#include "sen6x_core.h"

namespace sen6x
{

    static constexpr uint8_t CRC8_POLY = 0x31; // x^8 + x^5 + x^4 + 1
    static constexpr uint8_t CRC8_INIT = 0xFF;

    uint8_t Core::crc8(const uint8_t *data, size_t len)
    {
        uint8_t crc = CRC8_INIT;
        for (size_t i = 0; i < len; ++i)
        {
            crc ^= data[i];
            for (uint8_t b = 0; b < 8; ++b)
            {
                crc = (crc & 0x80) ? ((crc << 1) ^ CRC8_POLY) : (crc << 1);
            }
        }
        return crc;
    }

    /**
     * @brief Decode the 32-bit Device Status Register (DSR) into structured flags.
     *
     * Parses the raw DSR value from the SEN6x sensor into individual
     * warning and error flags defined in the DeviceStatus struct.
     *
     * Bit mapping
     *   - Bit 21 : Fan speed warning
     *   - Bit 12 : CO₂-1 error
     *   - Bit 11 : PM error
     *   - Bit 10 : HCHO error
     *   - Bit  9 : CO₂-2 error
     *   - Bit  7 : GAS error
     *   - Bit  6 : RH&T error
     *   - Bit  4 : FAN error
     *
     * @param raw 32-bit raw DSR value read from the sensor.
     * @return Parsed DeviceStatus struct with decoded flags.
     */
    DeviceStatus decodeDSR(uint32_t raw)
    {
        DeviceStatus d;
        d.raw = raw;
        d.fan_speed_warning = raw & (1UL << 21);
        d.fan_error = raw & (1UL << 4);
        d.rht_error = raw & (1UL << 6);
        d.gas_error = raw & (1UL << 7);
        d.co2_2_error = raw & (1UL << 9);
        d.hcho_error = raw & (1UL << 10);
        d.pm_error = raw & (1UL << 11);
        d.co2_1_error = raw & (1UL << 12);
        return d;
    }

    Status Core::begin()
    {
        wire_.beginTransmission(address_);
        uint8_t err = wire_.endTransmission(true);
        if (err != 0)
        {
            if (dbg_)
                dbg_->printf("[sen6x_core] Probe 0x%02X failed, err=%u\n", address_, err);
            return Status::I2C_ERROR;
        }
        initialized_ = true;
        return Status::OK;
    }

    Status Core::readDeviceStatus(uint32_t &out_status)
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;

        // Device Status Register command for SEN6x family (confirm in your datasheet)
        constexpr uint16_t CMD_READ_STATUS = 0xD206;

        Status s = sendCommand(CMD_READ_STATUS);
        if (s != Status::OK)
            return s;

        delay(50); // wait execution time

        // Two 16-bit words (MSW, LSW), each with its own CRC
        uint16_t words[2];
        readWords(words, 2, 0); // your readWords should verify CRC per word
        uint32_t dsr = (uint32_t(words[0]) << 16) | words[1];

        // Combine to 32-bit (Sensirion returns MSW first)
        out_status = (static_cast<uint32_t>(words[0]) << 16) | words[1];

        if (dbg_)
            dbg_->printf("[sen6x_core] DSR=0x%08lX\n", (unsigned long)out_status);
        return Status::OK;
    }

    Status Core::getDeviceStatus(DeviceStatus &out)
    {
        uint32_t raw = 0;
        Status s = readDeviceStatus(raw);
        if (s != Status::OK)
            return s;
        out = decodeDSR(raw);
        return Status::OK;
    }

    // ── sen6x_core.cpp ───────────────────────────────────────────────────────────

    Status Core::readAndClearDeviceStatus(uint32_t &out_status)
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;

        constexpr uint16_t CMD_READ_AND_CLEAR_STATUS = 0xD210;
        Status s = sendCommand(CMD_READ_AND_CLEAR_STATUS);
        if (s != Status::OK)
            return s;

        delay(20); // execution time per datasheet

        // Two 16-bit words (MSW, LSW), each followed by a CRC byte (handled by readWords)
        uint16_t words[2] = {0, 0};
        s = readWords(words, 2, 0);
        if (s != Status::OK)
            return s;

        out_status = (static_cast<uint32_t>(words[0]) << 16) | words[1];

        if (dbg_)
            dbg_->printf("[sen6x_core] Read&Clear DSR=0x%08lX\n",
                         (unsigned long)out_status);
        return Status::OK;
    }

    Status Core::getAndClearDeviceStatus(DeviceStatus &out)
    {
        uint32_t raw = 0;
        Status s = readAndClearDeviceStatus(raw);
        if (s != Status::OK)
            return s;
        out = decodeDSR(raw);
        return Status::OK;
    }

    /**
     * @brief Perform a full device reset (equivalent to a power cycle).
     *
     * Sends command 0xD304 (“Device Reset”) to the SEN6x sensor.
     * This resets all internal modules (fan, PM, gas, humidity, etc.)
     * and has the same effect as a power cycle.
     *
     * Applies to: SEN63C, SEN65, SEN66, SEN68
     *
     *
     * @return Status::OK if the command was accepted,
     *         Status::I2C_ERROR if communication failed,
     *         Status::NOT_INITIALIZED if begin() was not called.
     *
     * @note After reset, the device requires re-initialization and
     *       should be idle for at least 1.2 s before new commands.
     */
    Status Core::softReset()
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;

        constexpr uint16_t CMD_DEVICE_RESET = 0xD304;

        Status s = sendCommand(CMD_DEVICE_RESET);
        if (s != Status::OK)
            return s;

        // Wait for the full reset period (datasheet: 1200 ms)
        delay(1200);

        // After reset, device will lose internal state
        initialized_ = false;

        if (dbg_)
            dbg_->println("[sen6x_core] Device reset completed");

        return Status::OK;
    }

    Status Core::startMeasurement()
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;

        constexpr uint16_t CMD_START_MEAS_SEN6X = 0x0021;
        Status s = sendCommand(CMD_START_MEAS_SEN6X);
        if (s != Status::OK)
            return s;

        delay(50); // execution time per datasheet

        // Record when first data will be available (~1.1 s later)
        dataReadyBy_ = millis() + 1100;

        if (dbg_)
            dbg_->printf("[sen6x_core] Measurement started, data ready by t=%lu ms\n", dataReadyBy_);

        return Status::OK;
    }

    Status Core::stopMeasurement()
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;

        constexpr uint16_t CMD_STOP_MEAS_SEN6X = 0x0104;
        Status s = sendCommand(CMD_STOP_MEAS_SEN6X);
        if (s != Status::OK)
            return s;

        delay(1000); // per datasheet: wait 1000 ms before restarting

        // TODO:Change the delay into a flag which can be used for start measurement also

        if (dbg_)
            dbg_->println("[sen6x_core] Measurement stopped, device in idle mode");

        // Reset readiness timestamp since we’re back in idle
        dataReadyBy_ = 0;

        return Status::OK;
    }

    bool Core::isDataReadyByTime() const
    {
        return millis() >= dataReadyBy_;
    }

    /**
     * @brief Reads the device’s serial number string from the SEN6x sensor.
     *
     * This function issues the Get Serial Number command (0xD033), waits 20 ms for
     * execution, and retrieves up to 32 ASCII characters plus CRC bytes.
     * The serial number is returned as a null-terminated string.
     *
     * Command reference: Sensirion Datasheet SEN6x v0.9 (Section 4.8.19)
     * Applies to SEN63C, SEN65, SEN66, SEN68
     *
     * @param[out] out_serial  Reference to an Arduino String to store the serial number.
     * @return Status::OK on success, or I2C_ERROR / CRC_ERROR / TIMEOUT as appropriate.
     */
    Status Core::readSerialNumber(String &out_serial)
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;

        constexpr uint16_t CMD_GET_SN = 0xD033; // Get Serial Number SEN6x
        Status s = sendCommand(CMD_GET_SN);
        if (s != Status::OK)
            return s;

        delay(20); // exec time per datasheet

        // 32 ASCII chars -> 16 words; readWords() verifies CRC per word
        constexpr size_t WORDS = 16;
        uint16_t words[WORDS] = {0};
        s = readWords(words, WORDS, 0);
        if (s != Status::OK)
            return s;

        // Assemble into a C string (max 32 chars), stop at first '\0'
        char buf[33];
        size_t idx = 0;
        for (size_t i = 0; i < WORDS && idx < 32; ++i)
        {
            uint8_t hi = static_cast<uint8_t>(words[i] >> 8);
            uint8_t lo = static_cast<uint8_t>(words[i] & 0xFF);

            if (hi == '\0')
                break;
            buf[idx++] = static_cast<char>(hi);
            if (idx >= 32)
                break;

            if (lo == '\0')
                break;
            buf[idx++] = static_cast<char>(lo);
        }
        buf[idx] = '\0';

        out_serial = String(buf);
        return Status::OK;
    }

    /**
     * @brief Read the product name string from the SEN6x sensor.
     *
     * Sends command 0xD014 (“Get Product Name”) to retrieve the device’s
     * product name as a null-terminated ASCII string (up to 32 characters).
     * Each 16-bit word read includes its own CRC, verified by readWords().
     *
     * Applies to: SEN63C, SEN65, SEN66, SEN68
     * Execution time: ~20 ms
     * Available in: Idle and Measurement modes
     *
     * @param[out] out_name Arduino String to store the product name.
     * @return Status::OK on success,
     *         Status::I2C_ERROR or CRC_ERROR on failure,
     *         Status::NOT_INITIALIZED if begin() was not called.
     *
     * @see Core::readSerialNumber()
     */

    Status Core::readProductName(String &out_name)
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;

        constexpr uint16_t CMD_GET_PRODUCT_NAME = 0xD014; // Get Product Name
        Status s = sendCommand(CMD_GET_PRODUCT_NAME);
        if (s != Status::OK)
            return s;

        delay(20); // execution time per datasheet

        constexpr size_t WORDS = 16; // 32 bytes = 16 words
        uint16_t words[WORDS] = {0};
        s = readWords(words, WORDS, 0);
        if (s != Status::OK)
            return s;

        char buf[33];
        size_t idx = 0;
        for (size_t i = 0; i < WORDS && idx < 32; ++i)
        {
            uint8_t hi = static_cast<uint8_t>(words[i] >> 8);
            uint8_t lo = static_cast<uint8_t>(words[i] & 0xFF);

            if (hi == '\0')
                break;
            buf[idx++] = static_cast<char>(hi);
            if (idx >= 32)
                break;

            if (lo == '\0')
                break;
            buf[idx++] = static_cast<char>(lo);
        }
        buf[idx] = '\0';

        out_name = String(buf);
        return Status::OK;
    }

    Status Core::getDataReady(uint8_t &out_ready)
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;

        if (millis() < dataReadyBy_)
        {
            out_ready = 0x00;
            if (dbg_)
                dbg_->printf("[sen6x_core] Not enough time since start\n");
            return Status::OK;
        }

        constexpr uint16_t CMD_GET_DATA_READY = 0x0202;
        Status s = sendCommand(CMD_GET_DATA_READY);
        if (s != Status::OK)
            return s;

        delay(20);

        uint8_t bytes[2];
        s = rx_(bytes, 2, 0);
        if (s != Status::OK)
            return s;

        uint8_t crc;
        s = rx_(&crc, 1, 0);
        if (s != Status::OK)
            return s;

        if (crc8(bytes, 2) != crc)
            return Status::CRC_ERROR;

        out_ready = bytes[1];

        if (dbg_)
            dbg_->printf("[sen6x_core] GetDataReady: 0x%02X\n", out_ready);
        return Status::OK;
    }

    Status Core::setAveraging(uint8_t level)
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;
        uint16_t param = static_cast<uint16_t>(level);
        // TODO: real command
        return sendCommand(0x0201, &param, 1);
    }

    Status Core::setHeater(bool enable)
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;
        uint16_t param = enable ? 1 : 0;
        // TODO: real command
        return sendCommand(0x0210, &param, 1);
    }

    Status Core::readMeasurement(SEN66Measured &out)
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;

        constexpr uint16_t CMD_READ_MEAS_SEN66 = 0x0300;
        Status s = sendCommand(CMD_READ_MEAS_SEN66);
        if (s != Status::OK)
            return s;

        delay(20);

        uint16_t w[9] = {0};
        s = readWords(w, 9, 0);
        if (s != Status::OK)
            return s;

        auto sc_u10 = [](uint16_t v) -> float
        { return (v == 0xFFFF) ? NAN : (float)v / 10.0f; };
        auto sc_i100 = [](int16_t v) -> float
        { return (v == (int16_t)0x7FFF) ? NAN : (float)v / 100.0f; };
        auto sc_i200 = [](int16_t v) -> float
        { return (v == (int16_t)0x7FFF) ? NAN : (float)v / 200.0f; };
        auto sc_i10 = [](int16_t v) -> float
        { return (v == (int16_t)0x7FFF) ? NAN : (float)v / 10.0f; };
        auto sc_co2 = [](uint16_t v) -> float
        { return (v == 0xFFFF) ? NAN : (float)v; };

        out.pm1_0 = sc_u10(w[0]);
        out.pm2_5 = sc_u10(w[1]);
        out.pm4_0 = sc_u10(w[2]);
        out.pm10_0 = sc_u10(w[3]);
        out.humidity = sc_i100((int16_t)w[4]);
        out.temperature = sc_i200((int16_t)w[5]);
        out.voc_index = sc_i10((int16_t)w[6]);
        out.nox_index = sc_i10((int16_t)w[7]);
        out.co2 = sc_co2(w[8]);

        return Status::OK;
    }

    Status Core::sendCommand(uint16_t cmd, const uint16_t *words, size_t word_count)
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;

        uint8_t buf[2 + (word_count * 3)];
        buf[0] = static_cast<uint8_t>((cmd >> 8) & 0xFF);
        buf[1] = static_cast<uint8_t>(cmd & 0xFF);

        for (size_t i = 0; i < word_count; ++i)
        {
            uint8_t hi = static_cast<uint8_t>((words[i] >> 8) & 0xFF);
            uint8_t lo = static_cast<uint8_t>(words[i] & 0xFF);
            uint8_t pair[2] = {hi, lo};
            uint8_t c = crc8(pair, 2); // CRC over {hi, lo}
            size_t base = 2 + i * 3;
            buf[base + 0] = hi;
            buf[base + 1] = lo;
            buf[base + 2] = c;
        }

        return tx_(buf, sizeof(buf), true);
    }

    Status Core::readWords(uint16_t *out_words, size_t word_count, uint32_t wait_ms)
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;

        if (wait_ms)
            delay(wait_ms);
        const size_t bytes_expected = word_count * 3;

        int r = wire_.requestFrom((int)address_, (int)bytes_expected);
        if (r != (int)bytes_expected)
        {
            if (dbg_)
                dbg_->printf("[sen6x_core] requestFrom %u -> %d bytes\n",
                             (unsigned)bytes_expected, r);
            return Status::I2C_ERROR;
        }

        for (size_t i = 0; i < word_count; ++i)
        {
            uint8_t hi = wire_.read();
            uint8_t lo = wire_.read();
            uint8_t crc = wire_.read();
            uint8_t pair[2] = {hi, lo};
            uint8_t calc = crc8(pair, 2);
            if (calc != crc)
            {
                if (dbg_)
                    dbg_->printf("[sen6x_core] CRC mismatch on word %u (got 0x%02X, calc 0x%02X)\n",
                                 (unsigned)i, crc, calc);
                return Status::CRC_ERROR;
            }
            out_words[i] = (static_cast<uint16_t>(hi) << 8) | lo;
        }
        return Status::OK;
    }

    Status Core::tx_(const uint8_t *bytes, size_t len, bool sendStop)
    {
        wire_.beginTransmission(address_);
        size_t w = wire_.write(bytes, len);
        uint8_t err = wire_.endTransmission(sendStop);
        if (w != len || err != 0)
            return Status::I2C_ERROR;
        return Status::OK;
    }

    Status Core::rx_(uint8_t *bytes, size_t len, uint32_t wait_ms)
    {
        if (wait_ms)
            delay(wait_ms);
        int r = wire_.requestFrom((int)address_, (int)len);
        if (r != (int)len)
            return Status::I2C_ERROR;
        for (size_t i = 0; i < len; ++i)
            bytes[i] = wire_.read();
        return Status::OK;
    }

    Status Core::startFanCleaning()
    {
        if (!initialized_)
            return Status::NOT_INITIALIZED;

        constexpr uint16_t CMD_START_FAN_CLEAN = 0x5607;
        Status s = sendCommand(CMD_START_FAN_CLEAN);
        if (s != Status::OK)
            return s;

        if (dbg_)
            dbg_->println("[sen6x_core] Fan cleaning started (run ~10s)");

        delay(1100);

        if (dbg_)
            dbg_->println("[sen6x_core] Fan cleaning completed (run ~10s)");
        return Status::OK;
    }

} // namespace sen6x