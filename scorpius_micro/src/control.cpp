#include "control.h"
#include <Adafruit_PWMServoDriver.h>

// PCA9685 default address: 0x40
static Adafruit_PWMServoDriver driverModule;

static sAngles _lastAngles;

static constexpr eServo servos[] = {eServo::VERT_A,
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

void processAngles(sAngles& angles_)
{
    _lastAngles.vert_a = angles_.vert_a;
    _lastAngles.vert_b = angles_.vert_b;
    _lastAngles.vert_c = angles_.vert_c;
    _lastAngles.vert_d = angles_.vert_d;
    _lastAngles.vert_e = angles_.vert_e;
    _lastAngles.vert_f = angles_.vert_f;
    _lastAngles.hori_a = angles_.hori_a;
    _lastAngles.hori_b = angles_.hori_b;
    _lastAngles.hori_c = angles_.hori_c;
    _lastAngles.hori_d = angles_.hori_d;
    _lastAngles.hori_e = angles_.hori_e;
    _lastAngles.hori_f = angles_.hori_f;
}

void updatePosition()
{
    servoGoTo(eServo::VERT_A, _lastAngles.vert_a);
    servoGoTo(eServo::VERT_B, _lastAngles.vert_b);
    servoGoTo(eServo::VERT_C, _lastAngles.vert_c);
    servoGoTo(eServo::VERT_D, _lastAngles.vert_d);
    servoGoTo(eServo::VERT_E, _lastAngles.vert_e);
    servoGoTo(eServo::VERT_F, _lastAngles.vert_f);
    servoGoTo(eServo::HORIZ_A, _lastAngles.hori_a);
    servoGoTo(eServo::HORIZ_B, _lastAngles.hori_b);
    servoGoTo(eServo::HORIZ_C, _lastAngles.hori_c);
    servoGoTo(eServo::HORIZ_D, _lastAngles.hori_d);
    servoGoTo(eServo::HORIZ_E, _lastAngles.hori_e);
    servoGoTo(eServo::HORIZ_F, _lastAngles.hori_f);
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
    for (eServo s : servos)
    {
        servoGoTo(s, HOME_ANGLE);
        delay(50);
    }
}