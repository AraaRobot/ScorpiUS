#if ENABLE_DEBUG

#include "debug.h"

void debug()
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

    executeDebugFunc();

    delay(50);
}

void executeDebugFunc()
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
    while (c != 'q')
    {
        c = Serial.read();
        static int angle = 0;
        static int servo = 0;

        if (c == 'w')
        {
            if (servo % 2 == 0)
            {
                if (angle < 30)
                    angle += 5;
            }
            else if (servo % 2 == 1)
            {
                if (angle < 45)
                    angle += 5;
            }
            servoGoTo(static_cast<eServo>(servo), angle);
            COMM_DEBUG("Current angle: ");
            COMM_DEBUG(angle);
        }
        else if (c == 's')
        {
            if (servo % 2 == 0)
            {
                if (angle > -90)
                    angle -= 5;
            }
            else if (servo % 2 == 1)
            {
                if (angle > -45)
                    angle -= 5;
            }
            servoGoTo(static_cast<eServo>(servo), angle);
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
            if (servo < 11)
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

    while (c != 'q')
    {
        c = Serial.read();
        static int angle = 0;
        static uint8_t numberOfServos = 12U;

        if (c == 'z')
        {
            goHome();
            COMM_DEBUG("Homed");
            angle = 0;
        }
        else if (c == 'w')
        {
            if (angle < 90)
                angle += 5;
            COMM_DEBUG("Current angle: ");
            COMM_DEBUG(angle);
        }
        else if (c == 's')
        {
            if (angle > -90)
                angle -= 5;
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
            servoGoTo(static_cast<eServo>(i), angle);
        }
        delay(50);
    }

    COMM_DEBUG("Quitting multiple servos mode");
}

void testLegJoints()
{
    while (true)
    {
        char c = Serial.read();

        if (c == 'q')
        {
            break;
        }

        int angle0 = 0;
        int angle1 = 0;
        servoGoTo(eServo::VERT_A, angle0);
        servoGoTo(eServo::HORIZ_A, angle1);
        delay(500);

        while (true)
        {
            if (angle0 <= -90)
                break;
            if (angle1 > -45)
                angle1 -= 2;
            angle0 -= 5;

            servoGoTo(eServo::VERT_A, angle0);
            servoGoTo(eServo::HORIZ_A, angle1);
            delay(50);
        }

        while (angle1 < 45)
        {
            angle1 += 5;
            servoGoTo(eServo::HORIZ_A, angle1);
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
            servoGoTo(eServo::VERT_A, angle0);
            servoGoTo(eServo::HORIZ_A, angle1);
            delay(50);
            if (angle0 == 0 && angle1 == 0)
                break;
        }
    }
    COMM_DEBUG("Quitting single validation mode");
}

#endif  // ENABLE_DEBUG