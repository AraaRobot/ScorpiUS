#include "control.h"
#include <Adafruit_PWMServoDriver.h>

// PCA9685 default address: 0x40
static Adafruit_PWMServoDriver driverModule;

static constexpr eServo allServos[] = {eServo::VERT_A,
                                       eServo::VERT_B,
                                       eServo::VERT_C,
                                       eServo::VERT_D,
                                       eServo::VERT_E,
                                       eServo::VERT_F,
                                       eServo::HORIZ_A,
                                       eServo::HORIZ_B,
                                       eServo::HORIZ_C,
                                       eServo::HORIZ_D,
                                       eServo::HORIZ_E,
                                       eServo::HORIZ_F};

void controlInit()
{
    driverModule = Adafruit_PWMServoDriver(0x40);
    driverModule.begin();
    delay(100);

    driverModule.setPWMFreq(50);
    delay(200);

    goHome();
}

void processAngles(sAngles& angles)
{
    servoGoTo(eServo::VERT_A, angles.vert_a);
    servoGoTo(eServo::VERT_B, angles.vert_b);
    servoGoTo(eServo::VERT_C, angles.vert_c);
    servoGoTo(eServo::VERT_D, angles.vert_d);
    servoGoTo(eServo::VERT_E, angles.vert_e);
    servoGoTo(eServo::VERT_F, angles.vert_f);
    servoGoTo(eServo::HORIZ_A, angles.hori_a);
    servoGoTo(eServo::HORIZ_B, angles.hori_b);
    servoGoTo(eServo::HORIZ_C, angles.hori_c);
    servoGoTo(eServo::HORIZ_D, angles.hori_d);
    servoGoTo(eServo::HORIZ_E, angles.hori_e);
    servoGoTo(eServo::HORIZ_F, angles.hori_f);
    delay(50);  // Delay to allow servos to reach the position. Might be useless?
}

static int angleToPulse(int ang)  // gets the angle in degree and returns the pulse width
{
    int pulse = map(ang, -90, 90, SERVOMIN, SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max
    return pulse;
}

void servoGoTo(eServo servoId, int angle)
{
    int desiredAngle = 0;

    switch (servoId)
    {
        case eServo::VERT_A:
            [[fallthrough]];
        case eServo::VERT_B:
            [[fallthrough]];
        case eServo::VERT_C:
            [[fallthrough]];
        case eServo::VERT_D:
            [[fallthrough]];
        case eServo::VERT_E:
            [[fallthrough]];
        case eServo::VERT_F:
            desiredAngle = constrain(angle, MIN_ANGLE_VERTICAL, MAX_ANGLE_VERTICAL);
            driverModule.setPWM(static_cast<uint8_t>(servoId), 0, angleToPulse(desiredAngle));
            break;
        case eServo::HORIZ_A:
            [[fallthrough]];
        case eServo::HORIZ_B:
            [[fallthrough]];
        case eServo::HORIZ_C:
            [[fallthrough]];
        case eServo::HORIZ_D:
            [[fallthrough]];
        case eServo::HORIZ_E:
            [[fallthrough]];
        case eServo::HORIZ_F:
            desiredAngle = constrain(angle, MIN_ANGLE_HORIZONTAL, MAX_ANGLE_HORIZONTAL);
            driverModule.setPWM(static_cast<uint8_t>(servoId), 0, angleToPulse(desiredAngle));
            break;
        default:
            Serial.println("Invalid servo ID");
            break;
    }
}

void goHome()
{
    for (eServo s : allServos)
    {
        servoGoTo(s, HOME_ANGLE);
        delay(50);
    }
}