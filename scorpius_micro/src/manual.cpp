#if ENABLE_MANUAL

#include "manual.h"

namespace
{
    constexpr uint8_t NUM_SERVOS_BOARD_1 = static_cast<uint8_t>(eServo1::NUM_SERVOS);
    constexpr uint8_t NUM_SERVOS_BOARD_2 = static_cast<uint8_t>(eServo2::NUM_SERVOS);
    constexpr uint8_t NUM_SERVOS_TOTAL = NUM_SERVOS_BOARD_1 + NUM_SERVOS_BOARD_2;

    constexpr eServo1 SERVO_ORDER_BOARD_1[NUM_SERVOS_BOARD_1]
        = {eServo1::VERT_A, eServo1::VERT_B, eServo1::VERT_F, eServo1::HORIZ_A, eServo1::HORIZ_B, eServo1::HORIZ_F};
    constexpr eServo2 SERVO_ORDER_BOARD_2[NUM_SERVOS_BOARD_2] = {eServo2::VERT_D,
                                                                 eServo2::VERT_E,
                                                                 eServo2::VERT_C,
                                                                 eServo2::HORIZ_D,
                                                                 eServo2::HORIZ_E,
                                                                 eServo2::HORIZ_C,
                                                                 eServo2::TAIL};

    uint8_t getServoIndexWithinBoard(uint8_t servoIndex_)
    {
        return (servoIndex_ < NUM_SERVOS_BOARD_1) ? servoIndex_ : (servoIndex_ - NUM_SERVOS_BOARD_1);
    }

    bool isVerticalServo(const eServo1 servo_)
    {
        switch (servo_)
        {
            case eServo1::VERT_A:
            case eServo1::VERT_B:
            case eServo1::VERT_F:
                return true;
            default:
                return false;
        }
    }

    bool isVerticalServo(const eServo2 servo_)
    {
        switch (servo_)
        {
            case eServo2::VERT_D:
            case eServo2::VERT_E:
            case eServo2::VERT_C:
                return true;
            default:
                return false;
        }
    }

    bool isVerticalServo(uint8_t servoIndex_)
    {
        if (servoIndex_ < NUM_SERVOS_BOARD_1)
        {
            return isVerticalServo(SERVO_ORDER_BOARD_1[servoIndex_]);
        }

        const uint8_t board2Index = getServoIndexWithinBoard(servoIndex_);
        return isVerticalServo(SERVO_ORDER_BOARD_2[board2Index]);
    }

    int getServoMinAngle(uint8_t servoIndex_)
    {
        return isVerticalServo(servoIndex_) ? MIN_ANGLE_VERTICAL : MIN_ANGLE_HORIZONTAL;
    }

    int getServoMaxAngle(uint8_t servoIndex_)
    {
        return isVerticalServo(servoIndex_) ? MAX_ANGLE_VERTICAL : MAX_ANGLE_HORIZONTAL;
    }

    void setAngleForServoId(sAngles& angles_, const eServo1 servo_, const int8_t angle_)
    {
        switch (servo_)
        {
            case eServo1::VERT_A:
                angles_.vert_a = angle_;
                break;
            case eServo1::VERT_B:
                angles_.vert_b = angle_;
                break;
            case eServo1::VERT_F:
                angles_.vert_c = angle_;
                break;
            case eServo1::HORIZ_A:
                angles_.hori_a = angle_;
                break;
            case eServo1::HORIZ_B:
                angles_.hori_b = angle_;
                break;
            case eServo1::HORIZ_F:
                angles_.hori_c = angle_;
                break;
            default:
                break;
        }
    }

    void setAngleForServoId(sAngles& angles_, const eServo2 servo_, const int8_t angle_)
    {
        switch (servo_)
        {
            case eServo2::VERT_D:
                angles_.vert_d = angle_;
                break;
            case eServo2::VERT_E:
                angles_.vert_e = angle_;
                break;
            case eServo2::VERT_C:
                angles_.vert_f = angle_;
                break;
            case eServo2::HORIZ_D:
                angles_.hori_d = angle_;
                break;
            case eServo2::HORIZ_E:
                angles_.hori_e = angle_;
                break;
            case eServo2::HORIZ_C:
                angles_.hori_f = angle_;
                break;
            // case eServo2::TAIL:
            //     angles_.tail = angle_;
            default:
                break;
        }
    }

    void setAngleForServo(sAngles& angles_, uint8_t servoIndex_, const int8_t angle_)
    {
        if (servoIndex_ < NUM_SERVOS_BOARD_1)
        {
            setAngleForServoId(angles_, SERVO_ORDER_BOARD_1[servoIndex_], angle_);
            return;
        }

        const uint8_t board2Index = getServoIndexWithinBoard(servoIndex_);
        setAngleForServoId(angles_, SERVO_ORDER_BOARD_2[board2Index], angle_);
    }
}  // namespace

void manual()
{
    COMM_DEBUG("Choose state:\ns: single servo\t\tm: multiple servos\t\tv: validation demonstration");

    while (true)
    {
        char c = Serial.read();
        if (c == 's')
        {
            debugStateMachine = eDebugStates::JOG_SERVO;
            COMM_DEBUG("Jogging single servo");
            break;
        }
        else if (c == 'm')
        {
            debugStateMachine = eDebugStates::JOG_MULTIPLE;
            COMM_DEBUG("Jogging multiple servos");
            break;
        }
        else if (c == 'v')
        {
            debugStateMachine = eDebugStates::VALID_DEMO;
            COMM_DEBUG("Executing demo valid");
            break;
        }
        else if (c == 'h')
        {
            COMM_DEBUG("Options\ns: single servo\t\tm: multiple servos\t\tv: validation demonstration");
        }
    }

    executeManualFunc();

    delay(50);
}

void executeManualFunc()
{
    switch (debugStateMachine)
    {
        case eDebugStates::JOG_SERVO:
            jogServo();
            break;

        case eDebugStates::JOG_MULTIPLE:
            jogMultiple();
            break;

        case eDebugStates::VALID_DEMO:
            testLegJoints();
            break;
    }
}

void jogServo()
{
    char c = Serial.read();
    COMM_DEBUG("Entering single servo mode");

    static sAngles angles;

    while (c != 'q')
    {
        c = Serial.read();
        static int angle = 0;
        static uint8_t servo = 0U;
        const int minAngle = getServoMinAngle(servo);
        const int maxAngle = getServoMaxAngle(servo);

        if (c == 'w')
        {
            if (angle < maxAngle)
                angle += 5;
            if (angle > maxAngle)
                angle = maxAngle;

            setAngleForServo(angles, servo, angle);
            processAngles(angles);
            updatePositionForce(2U);
            COMM_DEBUG("Current angle: ");
            COMM_DEBUG(angle);
        }
        else if (c == 's')
        {
            if (angle > minAngle)
                angle -= 5;
            if (angle < minAngle)
                angle = minAngle;

            setAngleForServo(angles, servo, angle);
            processAngles(angles);
            updatePositionForce(2U);
            COMM_DEBUG("Current angle: ");
            COMM_DEBUG(angle);
        }
        else if (c == 'a')
        {
            if (servo > 0)
            {
                servo--;
                angle = 0;
            }
            COMM_DEBUG("Current servo: ");
            COMM_DEBUG(servo);
        }
        else if (c == 'd')
        {
            if (servo < static_cast<int>(NUM_SERVOS_TOTAL) - 1)
            {
                servo++;
                angle = 0;
            }
            COMM_DEBUG("Current servo: ");
            COMM_DEBUG(servo);
        }
        else if (c == 'z')
        {
            goHome();
            COMM_DEBUG("Homed");
            angles = {};
            angle = 0;
        }

        delay(50);
    }

    COMM_DEBUG("Quitting single servo mode");
}

void jogMultiple()
{
    char c = Serial.read();
    COMM_DEBUG("Entering multiple servos mode");

    static sAngles angles;

    while (c != 'q')
    {
        c = Serial.read();
        static int angle = 0;
        static uint8_t numberOfServos = 1U;

        if (c == 'z')
        {
            goHome();
            COMM_DEBUG("Homed");
            angles = {};
            angle = 0;
        }
        else if (c == 'w')
        {
            if (angle < MAX_ANGLE_VERTICAL)
                angle += 5;
            if (angle > MAX_ANGLE_VERTICAL)
                angle = MAX_ANGLE_VERTICAL;
            COMM_DEBUG("Current angle: ");
            COMM_DEBUG(angle);
        }
        else if (c == 's')
        {
            if (angle > MIN_ANGLE_VERTICAL)
                angle -= 5;
            if (angle < MIN_ANGLE_VERTICAL)
                angle = MIN_ANGLE_VERTICAL;
            COMM_DEBUG("Current angle: ");
            COMM_DEBUG(angle);
        }
        else if (c == 'd')
        {
            if (numberOfServos < NUM_SERVOS_TOTAL)
            {
                numberOfServos++;
            }
            COMM_DEBUG("Number of servos: ");
            COMM_DEBUG(numberOfServos);
        }
        else if (c == 'a')
        {
            if (numberOfServos > 0)
            {
                numberOfServos--;
            }
            COMM_DEBUG("Number of servos: ");
            COMM_DEBUG(numberOfServos);
        }

        for (uint8_t i = 0U; i < numberOfServos; i++)
        {
            setAngleForServo(angles, i, angle);
        }

        processAngles(angles);
        updatePositionForce(2U);
        delay(100);
    }

    COMM_DEBUG("Quitting multiple servos mode");
}

void testLegJoints()
{
    sAngles angles;
    while (true)
    {
        char c = Serial.read();

        if (c == 'q')
        {
            break;
        }

        int angle0 = 0;
        int angle1 = 0;

        setAngleForServoId(angles, eServo1::VERT_A, angle0);
        setAngleForServoId(angles, eServo1::HORIZ_A, angle1);
        processAngles(angles);
        updatePositionForce(2U);
        delay(500);

        while (true)
        {
            if (angle0 <= MIN_ANGLE_VERTICAL)
                break;
            if (angle1 > MIN_ANGLE_HORIZONTAL)
                angle1 -= 2;
            angle0 -= 5;

            setAngleForServoId(angles, eServo1::VERT_A, angle0);
            setAngleForServoId(angles, eServo1::HORIZ_A, angle1);
            processAngles(angles);
            updatePositionForce(2U);
            delay(50);
        }

        while (angle1 < MAX_ANGLE_HORIZONTAL)
        {
            angle1 += 5;
            setAngleForServoId(angles, eServo1::HORIZ_A, angle1);
            processAngles(angles);
            updatePositionForce(2U);
            delay(50);
        }

        while (true)
        {
            if (angle0 < 0)
            {
                angle0 += 5;
                if (angle0 > 0)
                    angle0 = 0;
            }
            if (angle1 > 0)
            {
                angle1 -= 2;
                if (angle1 < 0)
                    angle1 = 0;
            }
            setAngleForServoId(angles, eServo1::VERT_A, angle0);
            setAngleForServoId(angles, eServo1::HORIZ_A, angle1);
            processAngles(angles);
            updatePositionForce(2U);
            delay(50);
            if (angle0 == 0 && angle1 == 0)
                break;
        }
    }
    COMM_DEBUG("Quitting single validation mode");
}

#endif  // ENABLE_MANUAL