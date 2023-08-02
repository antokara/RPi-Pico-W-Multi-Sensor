#include <ArduinoHA.h>
#include "device.h"
#include "switches.h"
#include "lxSensor.h"

float rawInputToLuxMultiplier = (LUX_MAX - LUX_MIN) / (PHOTORESISTOR_DARKEST_INPUT - PHOTORESISTOR_BRIGHTEST_INPUT);

/**
 * @brief frequency in milliseconds,
 * to allow sending of the lx value to the controller,
 * even if the delta is greater.
 *
 * note: this may change on the mode (e.g. if debug is enabled)
 * @see src/switches.cpp
 */
unsigned int LxSensor::sendLxFrequency = LX_SENSOR_SEND_FREQUENCY;

/**
 * @brief frequency in milliseconds,
 * to allow sending of the lx value to the controller,
 * even if the delta is greater.
 *
 * note: this may change on the mode (e.g. if debug is enabled)
 * @see src/switches.cpp
 */
float LxSensor::sendLxDelta = LX_SENSOR_SEND_DELTA;

// current lx
float LxSensor::lx = 0.0;

// previous lx (so we only send changes)
float LxSensor::prevLx = 0.0;

// last time we sent it
unsigned long LxSensor::lastLxSendTime = 0;

// the Lx sensor
HASensorNumber LxSensor::lxSensor(LX_SENSOR_ENTITY_ID, HASensorNumber::PrecisionP2);

/**
 * @brief convers the raw photoresistor input to lx
 *
 * @param rawInput
 * @return float
 */
float rawPhotoresistorInputToLx(int rawInput)
{
    float lx = abs((rawInput - PHOTORESISTOR_BRIGHTEST_INPUT) * rawInputToLuxMultiplier - LUX_MAX);
    if (lx < LUX_MIN)
    {
        return LUX_MIN;
    }
    if (lx > LUX_MAX)
    {
        return LUX_MAX;
    }
    return lx;
}

void LxSensor::setup()
{
    LxSensor::lxSensor.setName(LX_SENSOR_ENTITY_FRIENDLY_NAME);
    LxSensor::lxSensor.setDeviceClass(LX_SENSOR_ENTITY_DEVICE_CLASS);
}

/**
 * @brief if we should send the pressure to the controller.
 * a separate throttler to keep the unsolicited updates
 *
 * @return true
 * @return false
 */
bool LxSensor::shouldSendLx()
{
    return abs(long(millis() - LxSensor::lastLxSendTime)) > LxSensor::sendLxFrequency;
}

void LxSensor::loop()
{
    int rawLxSensorInputValue = analogRead(PHOTORESISTOR_PIN); // read the input pin
    LxSensor::lx = rawPhotoresistorInputToLx(rawLxSensorInputValue);
    if (abs(LxSensor::lx - LxSensor::prevLx) >= LxSensor::sendLxDelta && LxSensor::shouldSendLx())
    {
        LxSensor::prevLx = LxSensor::lx;
        LxSensor::lastLxSendTime = millis();

#ifdef SERIAL_DEBUG
        Serial.print("raw: ");
        Serial.println(rawLxSensorInputValue);
        Serial.print("lx: ");
        Serial.println(LxSensor::lx);
#endif
        if (Switches::isDebugActive)
        {
            Device::mqtt.publish(LX_SENSOR_DEBUG_MQTT_TOPIC, String("raw lx input: " + String(rawLxSensorInputValue) + ", lx: " + String(LxSensor::lx)).c_str());
        }

        LxSensor::lxSensor.setValue(LxSensor::lx);
    }
    else if (Device::reconnected)
    {
        /**
         * @brief only upon reconnection (the reconnect flag lasts only one loop)
         * send the current value to the controller, in case for example, the lx changed
         * while we were disconnected, so that the controller gets this value "update"...
         */
        LxSensor::lxSensor.setValue(LxSensor::lx, true);
    }
}