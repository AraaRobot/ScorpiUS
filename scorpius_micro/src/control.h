#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "comm.h"

constexpr uint16_t SERVOMIN = 102;  // this is the 'minimum' pulse length count (out of 4096)
constexpr uint16_t SERVOMAX = 522;  // this is the 'maximum' pulse length count (out of 4096)

constexpr int8_t MAX_ANGLE_VERTICAL = 90;
constexpr int8_t MIN_ANGLE_VERTICAL = -90;
constexpr int8_t MAX_ANGLE_HORIZONTAL = 45;
constexpr int8_t MIN_ANGLE_HORIZONTAL = -45;
constexpr int8_t HOME_ANGLE = 0;

enum class eServo : uint8_t
{
    VERT_A = 0,
    HORIZ_A,
    VERT_B,
    HORIZ_B,
    VERT_C,
    HORIZ_C,
    VERT_D,
    HORIZ_D,
    VERT_E,
    HORIZ_E,
    VERT_F,
    HORIZ_F,
    // Add servos here...
    NUM_SERVOS
};

void controlInit();
void processAngles(sAngles& angles);
void updatePosition();
void servoGoTo(eServo servoId, int angle);
void goHome();

#endif  // CONTROL_H