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

enum class eDebugStates : uint8_t
{
    JOG_SERVO = 1U,
    JOG_MULTIPLE,
    VALID_DEMO
};

extern eStates controllerStateMachine;
extern eDebugStates debugStateMachine;

#endif  // STATE_MACHINE_H