#pragma once
#ifdef PMMODULE
#ifdef HF_SENSOR_LD2410
#include "Sensor.h"
#include <ld2410.h>

#define LD2410_PRESENCE 0x01
#define LD2410_MOTION   0x02

class SensorLD2410 : public Sensor
{
  private:
    ld2410 radar;
    uint32_t lastReading = 0;
    uint8_t currentPresence = 0;

  protected:
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    bool checkSensorConnection() override;
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorLD2410(uint16_t iMeasureTypes);
    SensorLD2410(uint16_t iMeasureTypes, uint8_t iAddress);
    virtual ~SensorLD2410() {}

    bool begin() override;
};
#endif
#endif
