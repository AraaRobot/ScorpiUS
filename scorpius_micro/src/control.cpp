#include "control.h"
#include <Adafruit_PWMServoDriver.h>

namespace
{
    Adafruit_PWMServoDriver _driverModule;

    // PCA9685 default address: 0x40
    constexpr uint8_t DRIVER_ADRESS = 0x40;

    sAngles _lastAngles;

    constexpr eServo _servosVert[static_cast<uint8_t>(eServo::NUM_SERVOS) / 2U]
        = {eServo::VERT_A, eServo::VERT_B, eServo::VERT_C, eServo::VERT_D, eServo::VERT_E, eServo::VERT_F};
    constexpr eServo _servosHoriz[static_cast<uint8_t>(eServo::NUM_SERVOS) / 2U]
        = {eServo::HORIZ_A, eServo::HORIZ_B, eServo::HORIZ_C, eServo::HORIZ_D, eServo::HORIZ_E, eServo::HORIZ_F};

    enum class eControllerState : uint8_t
    {
        IDLE = 0U,
        VERT,
        HORIZ
    } controllerState;

    constexpr uint8_t DELAY_BETWEEN_UPDATE_MS = 60U;
}  // namespace

void controlInit()
{
    _driverModule = Adafruit_PWMServoDriver(DRIVER_ADRESS);
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
    static unsigned long lastLoopTime = 0U;
    static eControllerState lastState = eControllerState::HORIZ;

    switch (controllerState)
    {
        case eControllerState::IDLE:
            if (millis() - lastLoopTime > DELAY_BETWEEN_UPDATE_MS)
            {
                if (lastState == eControllerState::HORIZ)
                {
                    controllerState = eControllerState::VERT;
                }
                else if (lastState == eControllerState::VERT)
                {
                    controllerState = eControllerState::HORIZ;
                }

                lastLoopTime = millis();
            }
            break;
            
        case eControllerState::HORIZ:
            for (eServo servo : _servosHoriz)
            {
                servoGoTo(servo, getAngleForServo(_lastAngles, servo));
            }
            lastState = controllerState;
            controllerState = eControllerState::IDLE;
            break;

        case eControllerState::VERT:
            for (eServo servo : _servosVert)
            {
                servoGoTo(servo, getAngleForServo(_lastAngles, servo));
            }
            lastState = controllerState;
            controllerState = eControllerState::IDLE;
            break;

        default:
            COMM_DEBUG("Invalid controller state");
            controllerState = eControllerState::HORIZ;
            break;
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
            commSend(eSerialMsgType::ERROR, errPayload, 1);
            break;
    }
}

void goHome()
{
    for (eServo s : _servosVert)
    {
        servoGoTo(s, HOME_ANGLE);
    }

    delay(60);

    for (eServo s : _servosHoriz)
    {
        servoGoTo(s, HOME_ANGLE);
    }

    static const uint8_t infoPayload[1] = {static_cast<uint8_t>(eInfoCode::SERVOS_HOMED)};
    commSend(eSerialMsgType::INFO, infoPayload, 1);
}