#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>

enum class eStates : uint8_t
{
    HOME = 1,
    RUNNING,
    REBOOT,
    eLast
};

extern eStates controllerStateMachine;

#endif  // define STATE_MACHINE_H