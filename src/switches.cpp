#include <ArduinoHA.h>
#include "switches.h"

HASwitch Switches::debugSwitch(DEBUG_SWITCH_ENTITY_ID);

/**
 * @brief controls the water monitor debug mode on MQTT
 */
bool Switches::isDebugActive = false;

/**
 * @brief to keep track of the first loop iteration
 */
bool Switches::firstLoop = true;

void Switches::setIsDebugActive(bool state)
{
    // keep our local state
    Switches::isDebugActive = state;
}

/**
 * @brief called when state changes remotely, from the controller
 *
 * @param state
 */
void Switches::onSwitchCommand(bool state, HASwitch *sender)
{
    if (sender == &Switches::debugSwitch)
    {
        Switches::setIsDebugActive(state);
    }

    // report state back to the Home Assistant
    sender->setState(state);
}

void Switches::setup()
{
    Switches::debugSwitch.setName(DEBUG_SWITCH_ENTITY_FRIENDLY_NAME);
    Switches::debugSwitch.setIcon(DEBUG_SWITCH_ENTITY_ICON);
    Switches::debugSwitch.onCommand(Switches::onSwitchCommand);
}

void Switches::loop()
{
    /**
     * @brief only on the first loop, reset the state,
     * in case there was a previous state that is now invalid.
     */
    if (Switches::firstLoop)
    {
        Switches::firstLoop = false;
        Switches::debugSwitch.setState(Switches::isDebugActive);
    }
}