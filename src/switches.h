#ifndef SWITCHES
#define SWITCHES
#include <ArduinoHA.h>

#define DEBUG_SWITCH_ENTITY_ID "multi_sensor_debug"
#define DEBUG_SWITCH_ENTITY_FRIENDLY_NAME "Debug"
#define DEBUG_SWITCH_ENTITY_ICON "mdi:test-tube"

class Switches
{
public:
    // properties
    static bool isDebugActive;
    static HASwitch debugSwitch;
    static bool firstLoop;

    // methods
    static void setup();
    static void loop();
    static void onSwitchCommand(bool state, HASwitch *sender);

private:
    static void setIsDebugActive(bool state);
};

#endif // SWITCHES