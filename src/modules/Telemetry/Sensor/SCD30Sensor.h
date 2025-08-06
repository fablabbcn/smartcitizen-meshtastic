#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include(<SensirionI2cScd30.h>)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"
#include <SensirionI2cScd30.h>

#define SCD30_I2C_CLOCK_SPEED 100000

class SCD30Sensor : public TelemetrySensor
{
  private:
    SensirionI2cScd30 scd30;
    TwoWire* bus;
    uint8_t address;

    bool performFRC(uint16_t targetCO2);
    bool setASC(bool ascEnabled);
    bool getASC(bool &ascEnabled);
    bool setTemperatureOffset(float offset);
    bool getTemperatureOffset(float &offset);
    bool getAltitude(uint16_t &altitude);
    bool setAltitude(uint16_t altitude);
    bool factoryReset();

    // Parameters
    bool ascActive;

  protected:
    virtual void setup() override;

  public:
    SCD30Sensor();
    virtual int32_t runOnce() override;
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    AdminMessageHandleResult handleAdminMessage(const meshtastic_MeshPacket &mp, meshtastic_AdminMessage *request,
                                                meshtastic_AdminMessage *response) override;
};

#endif
