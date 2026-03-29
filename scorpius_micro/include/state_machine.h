#include <Arduino.h>

enum class eStates : uint8_t
{
    HOME = 1,
    RUNNING,
    REBOOT,
    eLast
};

eStates controllerStateMachine = eStates::HOME;