// #include "IncludeManager.h"
#ifdef HF_SENSOR_LD2410
#include "SensorLD2410.h"
#include <Arduino.h>
#include "hardware.h"

SensorLD2410::SensorLD2410(uint16_t iMeasureTypes)
    : SensorLD2410(iMeasureTypes, 0){};

SensorLD2410::SensorLD2410(uint16_t iMeasureTypes, uint8_t iAddress)
    : Sensor(iMeasureTypes, iAddress)
{
    gMeasureTypes |= Pres;
};

uint8_t SensorLD2410::getSensorClass()
{
    return SENS_LD2410;
}

void SensorLD2410::sensorLoopInternal()
{
    if (gSensorState == Running)
    {
        radar.read();

        if (radar.isConnected())
        {
            if (millis() - lastReading > 1000) // Report every 1000ms
            {
                uint8_t nextPresence = 0;

                lastReading = millis();

                if (radar.presenceDetected())
                {
                    if (radar.stationaryTargetDetected())
                    {
                        printDebug("LD2410: Stationary target: %d cm, energy: %d\n", radar.stationaryTargetDistance(), radar.stationaryTargetEnergy());
                        nextPresence |= LD2410_PRESENCE;
                    }

                    if (radar.movingTargetDetected())
                    {
                        printDebug("LD2410: Moving target: %d cm, energy: %d\n", radar.movingTargetDistance(), radar.movingTargetEnergy());
                        nextPresence |= LD2410_MOTION | LD2410_PRESENCE;
                    }
                }
                else
                {
                    printDebug("LD2410: No target detected\n");
                }

                currentPresence = nextPresence;
            }
        // } else {
            // printDebug("Lost connection to LD2410 - setting state to Off\n");
            // gSensorState = Off;
        }
    }
    else
    {
        Sensor::sensorLoopInternal();
    }
}

float SensorLD2410::measureValue(MeasureType iMeasureType)
{
    if (gSensorState == Running && radar.isConnected())
    {
        switch (iMeasureType)
        {
            case Pres:
                return (float) currentPresence;
            default:
                break;
        }
    }

    return NO_NUM;
}

bool SensorLD2410::checkSensorConnection()
{
    bool result = gSensorState == Running && radar.isConnected();
    return result;
}

bool SensorLD2410::begin()
{
    printDebug("Starting LD2410 mmWave presence sensor...\n");

    SERIAL_HF.setRX(HF_UART_RX_PIN);
    SERIAL_HF.setTX(HF_UART_TX_PIN);
    SERIAL_HF.begin(256000);

    printDebug("SERIAL_HF is %srunnning\n", SERIAL_HF ? "" : "NOT ");

#if defined(LD2410_DEBUG_DATA) || defined(LD2410_DEBUG_COMMANDS)
    radar.debug(SERIAL_DEBUG);
#endif

    if (radar.begin(HF_LD2410_SERIAL, true))
    {
        printDebug("LD2410 connected\n");
        printDebug("LD2410 firmware version: %d.%d.%x\n", radar.firmware_major_version, radar.firmware_minor_version, radar.firmware_bugfix_version);
        return true;
    }
    else
    {
        printDebug("LD2410 NOT connected\n");
        return false;
    }
}
#endif
