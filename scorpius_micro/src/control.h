#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>

#define SERVOMIN  102    // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  522    // this is the 'maximum' pulse length count (out of 4096)

enum class eServo : uint8_t {

    A_0 = 0,
    A_1,
    B_0,
    B_1,
    C_0,
    C_1,
    D_0,
    D_1,
    E_0,
    E_1,
    F_0,
    F_1
};

void controlInit();
void servoGoTo(eServo servoId, int angle);

#endif // CONTROL_H