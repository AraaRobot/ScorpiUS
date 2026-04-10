#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "comm.h"

constexpr uint16_t SERVOMIN = 100;  // this is the 'minimum' pulse length count (out of 4096)
constexpr uint16_t SERVOMAX = 500;  // this is the 'maximum' pulse length count (out of 4096)

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
void processAngles(const sAngles& angles_);
void updatePosition();
// Forces immediate position updates by stepping the internal update state machine
// without waiting for the normal time-based gating. Each step updates one servo group
// (vertical or horizontal), so use 2 steps to update both groups.
void updatePositionForce(uint8_t steps_);
void servoGoTo(eServo servoId_, int angle_);
void goHome();

#endif  // CONTROL_H