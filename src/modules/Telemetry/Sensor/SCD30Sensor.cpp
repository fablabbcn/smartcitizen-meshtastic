#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include(<SensirionI2cScd30.h>)

#include "SCD30Sensor.h"
#include "../mesh/generated/meshtastic/telemetry.pb.h"

#define SCD30_NO_ERROR 0

SCD30Sensor::SCD30Sensor() : TelemetrySensor(meshtastic_TelemetrySensorType_SCD30, "SCD30") {}

void SCD30Sensor::setup() {}

int32_t SCD30Sensor::runOnce() {
    LOG_INFO("Init sensor: %s", sensorName);
    if (!hasSensor()) {
        return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
    }

    bus = nodeTelemetrySensorsMap[sensorType].second;
    address = (uint8_t)nodeTelemetrySensorsMap[sensorType].first;

#ifdef SCD30_I2C_CLOCK_SPEED
    uint32_t currentClock = bus->getClock();
    if (currentClock != SCD30_I2C_CLOCK_SPEED) {
        bus->setClock(SCD30_I2C_CLOCK_SPEED);
    }
#endif

    scd30.begin(*bus, address);

    if (scd30.startPeriodicMeasurement(0) != SCD30_NO_ERROR) {
        LOG_ERROR("SCD30: Failed to start periodic measurement.");
        return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
    }

    if (!getASC(ascActive)) {
        LOG_WARN("SCD30: Could not determine ASC state.");
    } else {
        LOG_INFO("SCD30: ASC is %s", ascActive ? "enabled" : "disabled");
    }

    bool test_scd_func;
    uint16_t FRC_CO2 = 450;
    float TempOffset = 1.0f;
    uint16_t Altitude = 10;

    factoryReset();
    performFRC(FRC_CO2);  // Perform Forced Recalibration to 400 ppm CO₂
    // setASC(true);  // Enable Auto-Calibration
    // test_scd_func = getASC(ascActive);
    // LOG_INFO("SCD30: ASC test function returned %d", test_scd_func);
    // LOG_INFO("SCD30: ASC is %s", ascActive);
    setTemperatureOffset((TempOffset+50));  // Set temperature offset to 0
    test_scd_func = getTemperatureOffset(TempOffset);
    LOG_INFO("SCD30: Temperature offset test function returned %d", test_scd_func);
    LOG_INFO("SCD30: Temperature offset set to %.2f", TempOffset);
    setAltitude((Altitude+1));  // Set altitude compensation to 0
    test_scd_func = getAltitude(Altitude);
    LOG_INFO("SCD30: Altitude compensation test function returned %d", test_scd_func);
    LOG_INFO("SCD30: Altitude compensation set to %u", Altitude);
    test_scd_func = factoryReset();
    LOG_INFO("SCD30: Factory reset test function returned %d", test_scd_func);

    status = 1;
    return initI2CSensor();
}

bool SCD30Sensor::getMetrics(meshtastic_Telemetry *measurement) {
    float co2, temperature, humidity;

    uint16_t dataReady = 0;
    if (scd30.getDataReady(dataReady) != SCD30_NO_ERROR || dataReady == 0) {
        LOG_WARN("SCD30: No data available.");
        return false;
    }

    if (scd30.readMeasurementData(co2, temperature, humidity) != SCD30_NO_ERROR) {
        LOG_ERROR("SCD30: Failed to read measurement data.");
        return false;
    }

    if (co2 == 0) {
        LOG_ERROR("SCD30: Invalid CO₂ reading.");
        return false;
    }

    measurement->variant.air_quality_metrics.has_co2 = true;
    measurement->variant.air_quality_metrics.has_co2_temperature = true;
    measurement->variant.air_quality_metrics.has_co2_humidity = true;
    measurement->variant.air_quality_metrics.co2 = co2;
    measurement->variant.air_quality_metrics.co2_temperature = temperature;
    measurement->variant.air_quality_metrics.co2_humidity = humidity;

    return true;
}

bool SCD30Sensor::performFRC(uint16_t targetCO2) {
    if (scd30.stopPeriodicMeasurement() != SCD30_NO_ERROR) {
        LOG_ERROR("SCD30: Failed to stop periodic measurement before FRC.");
        return false;
    }

    if (scd30.forceRecalibration(targetCO2) != SCD30_NO_ERROR) {
        LOG_ERROR("SCD30: Failed to perform forced recalibration.");
        return false;
    }

    return true;
}

bool SCD30Sensor::setASC(bool ascEnabled) {
    if (scd30.activateAutoCalibration(ascEnabled) != SCD30_NO_ERROR) {
        LOG_ERROR("SCD30: Failed to set ASC.");
        return false;
    }

    ascActive = ascEnabled;
    LOG_INFO("SCD30: ASC is now %s", ascActive ? "enabled" : "disabled");
    return true;
}

bool SCD30Sensor::getASC(bool &ascEnabled) {
    uint16_t active;
    if (scd30.getAutoCalibrationStatus(active) != SCD30_NO_ERROR) {
        return false;
    }
    ascEnabled = (active != 0);
    return true;
}

bool SCD30Sensor::setTemperatureOffset(float offset) {
    if (scd30.setTemperatureOffset((uint16_t)offset) != SCD30_NO_ERROR) {
        LOG_ERROR("SCD30: Failed to set temperature offset.");
        return false;
    }
    return true;
}

bool SCD30Sensor::getTemperatureOffset(float &offset) {
    uint16_t tempOffset;
    if (scd30.getTemperatureOffset(tempOffset) != SCD30_NO_ERROR) {
        return false;
    }
    offset = (float)tempOffset;
    return true;
}

bool SCD30Sensor::setAltitude(uint16_t altitude) {
    if (scd30.setAltitudeCompensation(altitude) != SCD30_NO_ERROR) {
        LOG_ERROR("SCD30: Failed to set altitude.");
        return false;
    }
    return true;
}

bool SCD30Sensor::getAltitude(uint16_t &altitude) {
    if (scd30.getAltitudeCompensation(altitude) != SCD30_NO_ERROR) {
        return false;
    }
    return true;
}

bool SCD30Sensor::factoryReset() {
    if (scd30.softReset() != SCD30_NO_ERROR) {
        LOG_ERROR("SCD30: Failed to perform factory reset.");
        return false;
    }
    delay(2000);  // Required delay after softReset
    return true;
}

AdminMessageHandleResult SCD30Sensor::handleAdminMessage(const meshtastic_MeshPacket &mp, meshtastic_AdminMessage *request,
                                                          meshtastic_AdminMessage *response) {
    AdminMessageHandleResult result;

    switch (request->which_payload_variant) {
        case meshtastic_AdminMessage_sensor_config_tag:
            if (!request->sensor_config.has_scdxx_config) {
                result = AdminMessageHandleResult::NOT_HANDLED;
                break;
            }

            if (request->sensor_config.scdxx_config.has_factory_reset) {
                this->factoryReset();
            }

            if (request->sensor_config.scdxx_config.has_set_asc) {
                bool asc = request->sensor_config.scdxx_config.set_asc;
                this->setASC(asc);
                if (!asc && request->sensor_config.scdxx_config.has_target_co2_conc) {
                    this->performFRC(request->sensor_config.scdxx_config.target_co2_conc);
                }
            }

            if (request->sensor_config.scdxx_config.has_temperature) {
                this->setTemperatureOffset(request->sensor_config.scdxx_config.temperature);
            }

            if (request->sensor_config.scdxx_config.has_altitude) {
                this->setAltitude(request->sensor_config.scdxx_config.altitude);
            }

            result = AdminMessageHandleResult::HANDLED;
            break;

        default:
            result = AdminMessageHandleResult::NOT_HANDLED;
    }

    return result;
}

#endif
