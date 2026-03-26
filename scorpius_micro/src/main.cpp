#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "comm.h"
#include "control.h"

void testLegJoints();
void executeDebug();

void setup()
{
    Serial.begin(115200);
    delay(100);  // Give serial time to initialize
    Serial.println("Initializing ScorpiUS project...");

    Wire.begin();
    Wire.setClock(100000);
    delay(100);

    controlInit();
    comm_init(Serial);
    Serial.println("Initialization complete. Entering main loop.");
}

void loop()
{
    // comm_process();
    // sAngles angles;
    // if (comm_consume(angles))
    // {
    //     processAngles(angles);
    // }

    // static unsigned long lastUpdate = 0;
    // unsigned long now = millis();
    // if (now - lastUpdate >= 20)  // ~50 Hz
    // {
    //     updatePosition();
    //     lastUpdate = now;
    // }

    executeDebug();
}

void testLegJoints()
{
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

void executeDebug()
{
    static int angle = 0;
    static int servo = 0;
    Serial.print("Current servo: ");
    Serial.print(servo);
    Serial.print(" Current angle: ");
    Serial.println(angle);
    char c = Serial.read();

    if (c == 'e')
    {
        while (c != 'q')
        {
            testLegJoints();
            c = Serial.read();
        }
    }
    else if (c == 'w')
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
    }
    else if (c == 'a')
    {
        if (servo > 0)
            servo--;
    }
    else if (c == 'd')
    {
        if (servo < 11)
            servo++;
    }
    else if (c == 'z')
    {
        for (uint8_t sm = 0; sm < 12U; sm++)
        {
            servoGoTo(static_cast<eServo>(sm), 0);
            angle = 0;
        }
    }

    delay(50);
}