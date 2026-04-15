#include "control.h"
#include <Adafruit_PWMServoDriver.h>

namespace
{
    Adafruit_PWMServoDriver _driverModule1;
    Adafruit_PWMServoDriver _driverModule2;

    constexpr uint8_t DRIVER_ADRESS_1 = 0x40;
    constexpr uint8_t DRIVER_ADRESS_2 = 0x41;

    sAngles _lastAngles;

    constexpr eServo1 _servosBoard1[static_cast<uint8_t>(eServo1::NUM_SERVOS)]
        = {eServo1::VERT_A, eServo1::VERT_B, eServo1::VERT_F, eServo1::HORIZ_A, eServo1::HORIZ_B, eServo1::HORIZ_F};
    constexpr eServo2 _servosBoard2[static_cast<uint8_t>(eServo2::NUM_SERVOS)]
        = {eServo2::VERT_D, eServo2::VERT_E, eServo2::VERT_C, eServo2::HORIZ_D, eServo2::HORIZ_E, eServo2::HORIZ_C, eServo2::TAIL};

    enum class eControllerState : uint8_t
    {
        IDLE = 0U,
        BOARD1,
        BOARD2
    } controllerState;

    unsigned long _lastLoopTime = 0U;
    eControllerState _lastState = eControllerState::BOARD2;

    constexpr uint8_t DELAY_BETWEEN_UPDATE_MS = 50U;
}  // namespace

void controlInit()
{
    _driverModule1 = Adafruit_PWMServoDriver(DRIVER_ADRESS_1);
    _driverModule2 = Adafruit_PWMServoDriver(DRIVER_ADRESS_2);
    _driverModule1.begin();
    _driverModule2.begin();
    delay(100);

    _driverModule1.setPWMFreq(50);
    _driverModule2.setPWMFreq(50);

    delay(200);

    goHome();
}

void controlReset()
{
    _driverModule1.reset();
    _driverModule2.reset();
}

static int8_t getAngleForServo1(sAngles& angles_, eServo1 servoId_)
{
    switch (servoId_)
    {
        case eServo1::VERT_A:
            return angles_.vert_a;
        case eServo1::VERT_B:
            return angles_.vert_b;
        case eServo1::VERT_F:
            return angles_.vert_f;
        case eServo1::HORIZ_A:
            return angles_.hori_a;
        case eServo1::HORIZ_B:
            return angles_.hori_b;
        case eServo1::HORIZ_F:
            return angles_.hori_f;
        default:
            return angles_.vert_a;
    }
}

static int8_t getAngleForServo2(sAngles& angles_, eServo2 servoId_)
{
    switch (servoId_)
    {
        case eServo2::VERT_D:
            return angles_.vert_d;
        case eServo2::VERT_E:
            return angles_.vert_e;
        case eServo2::VERT_C:
            return angles_.vert_c;
        case eServo2::HORIZ_D:
            return angles_.hori_d;
        case eServo2::HORIZ_E:
            return angles_.hori_e;
        case eServo2::HORIZ_C:
            return angles_.hori_c;
        // case eServo2::TAIL:
        //     return angles_.tail;
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
    switch (controllerState)
    {
        case eControllerState::IDLE:
            if (millis() - _lastLoopTime > DELAY_BETWEEN_UPDATE_MS)
            {
                if (_lastState == eControllerState::BOARD2)
                {
                    controllerState = eControllerState::BOARD1;
                }
                else if (_lastState == eControllerState::BOARD1)
                {
                    controllerState = eControllerState::BOARD2;
                }
            }
            break;

        case eControllerState::BOARD1:
            for (eServo1 servo : _servosBoard1)
            {
                servoGoTo1(servo, getAngleForServo1(_lastAngles, servo));
            }
            _lastLoopTime = millis();
            _lastState = controllerState;
            controllerState = eControllerState::IDLE;
            break;

        case eControllerState::BOARD2:
            for (eServo2 servo : _servosBoard2)
            {
                servoGoTo2(servo, getAngleForServo2(_lastAngles, servo));
            }
            _lastLoopTime = millis();
            _lastState = controllerState;
            controllerState = eControllerState::IDLE;
            break;

        default:
            COMM_DEBUG("Invalid controller state");
            controllerState = eControllerState::BOARD1;
            break;
    }
}

void updatePositionForce(uint8_t steps_)
{
    // Ensure we always start from IDLE so the first call selects a group.
    controllerState = eControllerState::IDLE;

    for (uint8_t i = 0U; i < steps_; i++)
    {
        // Bypass the normal time gating by making the elapsed time check pass.
        _lastLoopTime = 0U;

        // First call: IDLE -> choose next group.
        updatePosition();
        // Second call: perform chosen group update.
        updatePosition();
    }
}

static int angleToPulse(int ang_)  // gets the angle in degree and returns the pulse width
{
    int pulse = map(ang_, -90, 90, SERVOMIN, SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max
    return pulse;
}

void servoGoTo1(eServo1 servoId_, int angle_)
{
    int desiredAngle = 0;

    switch (servoId_)
    {
        case eServo1::VERT_A:
            [[fallthrough]];
        case eServo1::VERT_B:
            [[fallthrough]];
        case eServo1::VERT_F:
            desiredAngle = constrain(angle_, MIN_ANGLE_VERTICAL, MAX_ANGLE_VERTICAL);
            _driverModule1.setPWM(static_cast<uint8_t>(servoId_), 0, angleToPulse(desiredAngle));
            break;
        case eServo1::HORIZ_A:
            [[fallthrough]];
        case eServo1::HORIZ_B:
            [[fallthrough]];
        case eServo1::HORIZ_F:
            desiredAngle = constrain(angle_, MIN_ANGLE_HORIZONTAL, MAX_ANGLE_HORIZONTAL);
            _driverModule1.setPWM(static_cast<uint8_t>(servoId_), 0, angleToPulse(desiredAngle));
            break;
        default:
            COMM_DEBUG("Invalid servo ID");
            static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::INVALID_SERVO_ID)};
            commSend(eSerialMsgType::ERROR, errPayload, 1);
            break;
    }
}

void servoGoTo2(eServo2 servoId_, int angle_)
{
    int desiredAngle = 0;

    switch (servoId_)
    {
        case eServo2::VERT_D:
            [[fallthrough]];
        case eServo2::VERT_E:
            [[fallthrough]];
        case eServo2::VERT_C:
            desiredAngle = constrain(angle_, MIN_ANGLE_VERTICAL, MAX_ANGLE_VERTICAL);
            _driverModule2.setPWM(static_cast<uint8_t>(servoId_), 0, angleToPulse(desiredAngle));
            break;
        case eServo2::HORIZ_D:
            [[fallthrough]];
        case eServo2::HORIZ_E:
            [[fallthrough]];
        case eServo2::HORIZ_C:
            [[fallthrough]];
        case eServo2::TAIL:
            desiredAngle = constrain(angle_, MIN_ANGLE_HORIZONTAL, MAX_ANGLE_HORIZONTAL);
            _driverModule2.setPWM(static_cast<uint8_t>(servoId_), 0, angleToPulse(desiredAngle));
            break;
        default:
            COMM_DEBUG("Invalid servo ID");
            static const uint8_t errPayload[1] = {static_cast<uint8_t>(eErrorCode::INVALID_SERVO_ID)};
            commSend(eSerialMsgType::ERROR, errPayload, 1);
            break;
    }
}

void goHome()
{
    for (eServo1 s : _servosBoard1)
    {
        servoGoTo1(s, HOME_ANGLE);
    }

    delay(60);

    for (eServo2 s : _servosBoard2)
    {
        servoGoTo2(s, HOME_ANGLE);
    }

    static const uint8_t infoPayload[1] = {static_cast<uint8_t>(eInfoCode::SERVOS_HOMED)};
    commSend(eSerialMsgType::INFO, infoPayload, 1);
}