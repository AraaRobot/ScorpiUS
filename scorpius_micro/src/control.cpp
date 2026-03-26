#include "control.h"
#include <Adafruit_PWMServoDriver.h>

// PCA9685 default address: 0x40
static Adafruit_PWMServoDriver _driverModule;

static sAngles _lastAngles;

static constexpr eServo _servos[static_cast<uint8_t>(eServo::NUM_SERVOS)] = {eServo::VERT_A,
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
    _driverModule = Adafruit_PWMServoDriver(0x40);
    _driverModule.begin();
    delay(100);

    _driverModule.setPWMFreq(50);
    delay(200);

    goHome();
}

static int8_t getAngleForServo(sAngles& angles_, eServo servoId_)
{
    switch (servoId_)
    {
        case eServo::VERT_A:
            return angles_.vert_a;
        case eServo::VERT_B:
            return angles_.vert_b;
        case eServo::VERT_C:
            return angles_.vert_c;
        case eServo::VERT_D:
            return angles_.vert_d;
        case eServo::VERT_E:
            return angles_.vert_e;
        case eServo::VERT_F:
            return angles_.vert_f;
        case eServo::HORIZ_A:
            return angles_.hori_a;
        case eServo::HORIZ_B:
            return angles_.hori_b;
        case eServo::HORIZ_C:
            return angles_.hori_c;
        case eServo::HORIZ_D:
            return angles_.hori_d;
        case eServo::HORIZ_E:
            return angles_.hori_e;
        case eServo::HORIZ_F:
            return angles_.hori_f;
        default:
            return angles_.vert_a;
    }
}
void processAngles(const sAngles& angles_)
{
    _lastAngles = angles_;
}

void updatePosition()
{
    for (eServo servo : _servos)
    {
        servoGoTo(servo, getAngleForServo(_lastAngles, servo));
    }
}

static int angleToPulse(int ang_)  // gets the angle in degree and returns the pulse width
{
    int pulse = map(ang_, -90, 90, SERVOMIN, SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max
    return pulse;
}

void servoGoTo(eServo servoId_, int angle_)
{
    int desiredAngle = 0;

    switch (servoId_)
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
            desiredAngle = constrain(angle_, MIN_ANGLE_VERTICAL, MAX_ANGLE_VERTICAL);
            _driverModule.setPWM(static_cast<uint8_t>(servoId_), 0, angleToPulse(desiredAngle));
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
            desiredAngle = constrain(angle_, MIN_ANGLE_HORIZONTAL, MAX_ANGLE_HORIZONTAL);
            _driverModule.setPWM(static_cast<uint8_t>(servoId_), 0, angleToPulse(desiredAngle));
            break;
        default:
            COMM_DEBUG("Invalid servo ID");
            static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::INVALID_SERVO_ID)};
            comm_send(eSerialMsgType::ERROR, errPayload, 1);
            break;
    }

    delay(500);
}

void goHome()
{
    for (eServo s : _servos)
    {
        servoGoTo(s, HOME_ANGLE);
        delay(50);
    }

    static const uint8_t infoPayload[1] = {static_cast<uint8_t>(eInfoCode::SERVOS_HOMED)};
    comm_send(eSerialMsgType::INFO, infoPayload, 1);
}