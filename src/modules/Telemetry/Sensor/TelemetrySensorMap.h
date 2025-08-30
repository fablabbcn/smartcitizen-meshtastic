#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#pragma once

// TODO make an struct for holding an SensorArray inside
#include "TelemetrySensor.h"

#if __has_include(<Adafruit_SHT31.h>)
#include "SHT31Sensor.h"
#endif

struct SensorInfo {
    uint8_t address = 0;
    TwoWire * bus;
    meshtastic_TelemetrySensorType type = meshtastic_TelemetrySensorType_SENSOR_UNSET;
    TelemetrySensor *sensor;
};

struct SensorInitInfo {
    uint8_t address = 0;
    TwoWire * bus;
};

class TelemetrySensorMap
{
    public:

        static const uint8_t MAX_SENSOR_COUNT = 64;
        // TODO - Rename
        SensorInfo nodeTelemetrySensorsMap[MAX_SENSOR_COUNT];
        // NOTE
        // nodeTelemetrySensorsMap was a pair of:
        // found.address.address
        // i2cScanner->fetchI2CBus(found.address)
        uint8_t sensorCount = 0;

        bool newSensor(uint8_t index) {
            meshtastic_TelemetrySensorType sensorType = nodeTelemetrySensorsMap[index].type;

            TelemetrySensor *sensor;

            switch (sensorType) {
                case meshtastic_TelemetrySensorType_SHT31:
                    {
                        // TODO
                        sensor = new SHT31Sensor(nodeTelemetrySensorsMap[index].address, nodeTelemetrySensorsMap[index].bus);
                        nodeTelemetrySensorsMap[index].sensor = sensor;
                        break;
                    }
                default:
                    return false;
                    break;
            }
        }

        int32_t initSensor(uint8_t index) {
            uint8_t address = nodeTelemetrySensorsMap[index].address;
            TwoWire * bus = nodeTelemetrySensorsMap[index].bus;
            if (!address) {
                return 0;
            }
            return nodeTelemetrySensorsMap[index].sensor->runOnce();
        }

        void removeSensor(uint8_t index){
            nodeTelemetrySensorsMap[index].address = 0;
        }

        uint8_t countSensors(meshtastic_TelemetrySensorType sensorType){
            uint8_t n_sensors = 0;
            LOG_INFO("Counting sensors...");
            LOG_INFO("Sensor count: %d", n_sensors);
            for (int i = 0; i<MAX_SENSOR_COUNT;i+=1) {
                if (nodeTelemetrySensorsMap[i].type == sensorType) {
                    n_sensors +=1;
                    LOG_INFO("Sensor count: %d", n_sensors);
                }
            }

            LOG_INFO("Final sensor count: %d", n_sensors);

            return n_sensors;
        }

        SensorInitInfo findDevice(meshtastic_TelemetrySensorType sensorType) {
            SensorInitInfo info;
            for (int i = 0; i<MAX_SENSOR_COUNT;i+=1) {
                if (nodeTelemetrySensorsMap[i].type == sensorType) {
                    info.address = nodeTelemetrySensorsMap[i].address;
                    info.bus = nodeTelemetrySensorsMap[i].bus;
                    if (info.address) {
                        return info;
                    }
                }
            }
            return info;
        }
};

#endif

