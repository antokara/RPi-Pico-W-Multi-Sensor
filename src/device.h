#ifndef DEVICE
#define DEVICE

#include <ArduinoHA.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define DEVICE_ID "multiSensor"
#define DEVICE_NAME "Multi Sensor"
#define FIRMWARE_VERSION "1.0.3"

// uncomment to enable serial.print debug messages
// #define SERIAL_DEBUG

/**
 * @brief information related to the status sensor
 *
 */
#define STATUS_SENSOR_ID "multiSensorStatus"
#define STATUS_SENSOR_NAME "Status"
#define STATUS_SENSOR_ICON "mdi:check-circle"

/**
 * @brief the bits to use for analog pin resolution
 * @see https://arduino-pico.readthedocs.io/en/latest/analog.html#void-analogreadresolution-int-bits
 */
#define ANALOG_READ_RESOLUTION 10

/**
 * @brief the max value (range) an analog pin can achieve.
 *        this is directly related to the ANALOG_READ_RESOLUTION defined above.
 *        it must be the result of 2^ANALOG_READ_RESOLUTION-1
 * @see https://arduino-pico.readthedocs.io/en/latest/analog.html#int-analogread-pin-size-t-pin-a0-a3
 */
#define MAX_ANALOG_PIN_RANGE 1023

/**
 * @brief the max voltage an analog pin will need,
 *        to reach the MAX_ANALOG_PIN_RANGE
 */
#define MAX_ANALOG_PIN_RANGE_VOLTAGE 3.3

/**
 * @brief time in milliseconds to wait for the WiFi to connect
 *
 */
#define WAIT_FOR_WIFI 5000

/**
 * @brief time in milliseconds to wait for the MQTT to connect/reconnect
 *
 */
#define WAIT_FOR_MQTT 2500

/**
 * @brief frequence in milliseconds,
 * to check for the Wifi connection status
 */
#define WIFI_CHECK_FREQUENCY 10000

/**
 * @brief frequence in milliseconds,
 * to send a heartbit status update to the controller
 */
#define HEARTBIT_FREQUENCY 15000

/**
 * @brief the status we send when we first connect to the controller
 * at the very first loop iteration
 */
#define STATUS_CONNECTED "connected"

/**
 * @brief the status we send when we reconnect to the controller
 * after a WiFi disconnection
 */
#define STATUS_RECONNECTED "reconnected"

/**
 * @brief the status we send to to controller, after the first loop iteration
 * to signify we are ready and also as a heartbit
 */
#define STATUS_READY "ready"

class Device
{
public:
    // properties
    static const float analogInputValueMultiplier;
    static int wifiStatus;
    static WiFiClient client;
    static HADevice device;
    static HAMqtt mqtt;
    static HASensor statusSensor;
    static bool firstLoop;
    static bool reconnected;
    static unsigned long lastWifiCheck;
    static unsigned long lastHeartbit;

    // methods
    static void connectToMQTT();
    static void connectToWifi();
    static bool isConnected();
    static void wifiLoop();
    static void heartbitLoop();
    static void setupOTA();
    static void setup();
    static void loop();
};

#endif // DEVICE