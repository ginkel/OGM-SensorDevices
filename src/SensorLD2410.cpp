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
    processPowercycle();

    if (gSensorState == Running)
    {
        radar.read();

        const bool connected = radar.isConnected();
        if (connected)
        {
            if (!isConnected) {
                logDebugP("LD2410 connected");
                radar.requestFirmwareVersion();
                logDebugP("LD2410 firmware version: %d.%d.%x", radar.firmware_major_version, radar.firmware_minor_version, radar.firmware_bugfix_version);
            }

            if (millis() - lastReading > 1000) // Report every 1000ms
            {
                uint8_t nextPresence = 0;

                lastReading = millis();

                if (radar.presenceDetected())
                {
                    if (radar.stationaryTargetDetected())
                    {
                        logDebugP("LD2410: Stationary target: %d cm, energy: %d", radar.stationaryTargetDistance(), radar.stationaryTargetEnergy());
                        nextPresence |= LD2410_PRESENCE;
                    }

                    if (radar.movingTargetDetected())
                    {
                        logDebugP("LD2410: Moving target: %d cm, energy: %d", radar.movingTargetDistance(), radar.movingTargetEnergy());
                        nextPresence |= LD2410_MOTION | LD2410_PRESENCE;
                    }
                }
                else
                {
                    logDebugP("LD2410: No target detected");
                }

                currentPresence = nextPresence;
            }
        }
        else
        {
            currentPresence = 0;
        }

        isConnected |= connected;
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
    logDebugP("Starting LD2410 mmWave presence sensor...");

    if (!serialInitialized)
    {
        logDebugP("Initializing serial communication (Rx_Pin = %i, Tx_Pin = %i)", HF_UART_RX_PIN, HF_UART_TX_PIN);

        SERIAL_HF.setRX(HF_UART_RX_PIN);
        SERIAL_HF.setTX(HF_UART_TX_PIN);
        SERIAL_HF.begin(256000);
        serialInitialized = true;
    }

    logDebugP("SERIAL_HF is %srunnning", SERIAL_HF ? "" : "NOT ");

#if defined(LD2410_DEBUG_DATA) || defined(LD2410_DEBUG_COMMANDS)
    Stream *virtualSerial = new OpenKNX::Log::VirtualSerial("LD2410");
    radar.debug(*virtualSerial);
#endif

    radar.begin(HF_LD2410_SERIAL, false);

    restartSensor();

    return true;
}

void SensorLD2410::restartSensor()
{
    powerCycleStartedAt = delayTimerInit();
    enableSensorHardware(false);
}

void SensorLD2410::processPowercycle()
{
    if (powerCycleStartedAt > 0 && delayCheck(powerCycleStartedAt, LD2410_POWERCYCLE_DURATION_MILLIS))
    {
        enableSensorHardware(true);
        powerCycleStartedAt = 0;
    }
}

void SensorLD2410::enableSensorHardware(bool enabled)
{
    uint8_t newPinState = HF_POWER_PIN_ACTIVE_ON == LOW ? !enabled : enabled;
    logDebugP("enableHardwareSensor: HF_POWER_PIN (%d) will be set to: %i", HF_POWER_PIN, newPinState);
    digitalWrite(HF_POWER_PIN, newPinState);
}
#endif
