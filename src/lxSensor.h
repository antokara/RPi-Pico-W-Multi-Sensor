#ifndef LX_SENSOR
#define LX_SENSOR

#include <ArduinoHA.h>

/**
 * @brief the unique (across all HA) entity id
 * Important: it can't match the entity friendly name
 *
 */
#define LX_SENSOR_ENTITY_ID "multi_sensor_illuminance"
/**
 * @brief the entity's friendly name
 * Important: it can't match the entity id
 */
#define LX_SENSOR_ENTITY_FRIENDLY_NAME "illuminance"
/**
 * @brief the class name in HA
 * @see https://www.home-assistant.io/integrations/sensor/
 */
#define LX_SENSOR_ENTITY_DEVICE_CLASS "illuminance"

/**
 * @brief the MQTT topic for debugging this sensor
 *
 */
#define LX_SENSOR_DEBUG_MQTT_TOPIC "debug:multiSensor:lxSensor"

// the (analog) pin that we connect the light sensor output
// you may use A0-A2
#define PHOTORESISTOR_PIN A0

// the raw input value we get from the photoresistor at brightest
#define PHOTORESISTOR_BRIGHTEST_INPUT 24
// the raw input value we get from the photoresistor at darkest
#define PHOTORESISTOR_DARKEST_INPUT 992

// Darkest LX (Moonless, overcast night sky)
#define LUX_MIN 0.0001
// Lightest LX (direct sunlight)
#define LUX_MAX 100000

/**
 * @brief the time frequency in milliseconds that needs to pass from the last time
 * we sent the value to the controller, in order to qualify for sending the new value,
 * during normal operation mode.
 */
#define LX_SENSOR_SEND_FREQUENCY 15000

/**
 * @brief the lx delta that needs to be, in order to qualify for sending the new value
 */
#define LX_SENSOR_SEND_DELTA 1500

class LxSensor
{
public:
    static float rawInputToLuxMultiplier;
    static unsigned int sendLxFrequency;
    static float sendLxDelta;
    static float lx;
    static float prevLx;
    static unsigned long lastLxSendTime;
    static HASensorNumber lxSensor;

    // methods
    static bool shouldSendLx();
    static void setup();
    static void loop();
};

#endif // LX_SENSOR