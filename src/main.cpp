#include <ArduinoHA.h>
#include "device.h"
#include "lxSensor.h"
#include "switches.h"

void setup()
{
    Device::setup();
    Switches::setup();
    LxSensor::setup();
    // after everything is setup...
    Device::connectToMQTT();
}

void loop()
{
    Device::loop();
    Switches::loop();
    LxSensor::loop();
}
