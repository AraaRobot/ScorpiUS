#if ENABLE_MANUAL

#include "manual.h"

namespace
{
    bool isVerticalServo(eServo servo_)
    {
        return (static_cast<uint8_t>(servo_) % 2U) == 0U;
    }

    int getServoMinAngle(eServo servo_)
    {
        return isVerticalServo(servo_) ? MIN_ANGLE_VERTICAL : MIN_ANGLE_HORIZONTAL;
    }

    int getServoMaxAngle(eServo servo_)
    {
        return isVerticalServo(servo_) ? MAX_ANGLE_VERTICAL : MAX_ANGLE_HORIZONTAL;
    }

    void setAngleForServo(sAngles& angles_, eServo servoId_, int angle_)
    {
        const int8_t a = static_cast<int8_t>(angle_);
        switch (servoId_)
        {
            case eServo::VERT_A:
                angles_.vert_a = a;
                break;
            case eServo::VERT_B:
                angles_.vert_b = a;
                break;
            case eServo::VERT_C:
                angles_.vert_c = a;
                break;
            case eServo::VERT_D:
                angles_.vert_d = a;
                break;
            case eServo::VERT_E:
                angles_.vert_e = a;
                break;
            case eServo::VERT_F:
                angles_.vert_f = a;
                break;
            case eServo::HORIZ_A:
                angles_.hori_a = a;
                break;
            case eServo::HORIZ_B:
                angles_.hori_b = a;
                break;
            case eServo::HORIZ_C:
                angles_.hori_c = a;
                break;
            case eServo::HORIZ_D:
                angles_.hori_d = a;
                break;
            case eServo::HORIZ_E:
                angles_.hori_e = a;
                break;
            case eServo::HORIZ_F:
                angles_.hori_f = a;
                break;
            default:
                break;
        }
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
        static int servo = 0;
        const eServo servoId = static_cast<eServo>(servo);
        const int minAngle = getServoMinAngle(servoId);
        const int maxAngle = getServoMaxAngle(servoId);

        if (c == 'w')
        {
            if (angle < maxAngle)
                angle += 5;
            if (angle > maxAngle)
                angle = maxAngle;

            setAngleForServo(angles, servoId, angle);
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

            setAngleForServo(angles, servoId, angle);
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
            if (servo < static_cast<int>(eServo::NUM_SERVOS) - 1)
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
            if (numberOfServos < static_cast<uint8_t>(eServo::NUM_SERVOS))
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
            setAngleForServo(angles, static_cast<eServo>(i), angle);
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

        setAngleForServo(angles, eServo::VERT_A, angle0);
        setAngleForServo(angles, eServo::HORIZ_A, angle1);
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

            setAngleForServo(angles, eServo::VERT_A, angle0);
            setAngleForServo(angles, eServo::HORIZ_A, angle1);
            processAngles(angles);
            updatePositionForce(2U);
            delay(50);
        }

        while (angle1 < MAX_ANGLE_HORIZONTAL)
        {
            angle1 += 5;
            setAngleForServo(angles, eServo::HORIZ_A, angle1);
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
            setAngleForServo(angles, eServo::VERT_A, angle0);
            setAngleForServo(angles, eServo::HORIZ_A, angle1);
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