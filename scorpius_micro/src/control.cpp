#include "control.h"
#include <Adafruit_PWMServoDriver.h>

// PCA9685 default address: 0x40
static Adafruit_PWMServoDriver driverModule;

static sAngles _lastAngles;

static constexpr eServo servos[static_cast<uint8_t>(eServo::NUM_SERVOS)] = {eServo::VERT_A,
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

static int8_t& getAngleForServo(sAngles& _lastAngles, eServo servoId)
{
    switch (servoId)
    {
        case eServo::VERT_A:
            return _lastAngles.vert_a;
        case eServo::VERT_B:
            return _lastAngles.vert_b;
        case eServo::VERT_C:
            return _lastAngles.vert_c;
        case eServo::VERT_D:
            return _lastAngles.vert_d;
        case eServo::VERT_E:
            return _lastAngles.vert_e;
        case eServo::VERT_F:
            return _lastAngles.vert_f;
        case eServo::HORIZ_A:
            return _lastAngles.hori_a;
        case eServo::HORIZ_B:
            return _lastAngles.hori_b;
        case eServo::HORIZ_C:
            return _lastAngles.hori_c;
        case eServo::HORIZ_D:
            return _lastAngles.hori_d;
        case eServo::HORIZ_E:
            return _lastAngles.hori_e;
        case eServo::HORIZ_F:
            return _lastAngles.hori_f;
        default:
            return _lastAngles.vert_a;
    }
}
void processAngles(const sAngles& angles)
{
    _lastAngles = angles;
}

void updatePosition()
{
    for (eServo servo : servos)
    {
        servoGoTo(servo, getAngleForServo(_lastAngles, servo));
    }
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