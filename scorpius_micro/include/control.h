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

enum class eServo1 : uint8_t
{
    // Servos on first driver
    VERT_A = 0U,
    HORIZ_A,
    VERT_B = 4U,
    HORIZ_B,
    VERT_F = 8U,
    HORIZ_F,
    // Add servos here...
    NUM_SERVOS = 6U
};

// Servos on second driver
enum class eServo2 : uint8_t
{
    VERT_D = 0U,
    HORIZ_D,
    VERT_E = 4U,
    HORIZ_E,
    VERT_C = 8U,
    HORIZ_C,
    TAIL = 12U,
    NUM_SERVOS = 7U
};

void controlInit();
void processAngles(const sAngles& angles_);
void updatePosition();
// Forces immediate position updates by stepping the internal update state machine
// without waiting for the normal time-based gating. Each step updates one servo group
// (vertical or horizontal), so use 2 steps to update both groups.
void updatePositionForce(uint8_t steps_);
void servoGoTo1(eServo1 servoId_, int angle_);
void servoGoTo2(eServo2 servoId_, int angle_);
void goHome();
void controlReset();

#endif  // CONTROL_H