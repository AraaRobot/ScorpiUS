#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 default address: 0x40
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN  102    // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  522    // this is the 'maximum' pulse length count (out of 4096)

int angleToPulse(int ang);

void setup()
{

    Serial.begin(115200);
    board1.begin();
    board1.setPWMFreq(50);
    delay(50);
    
    Serial.println("Waiting for start...");
    char c = Serial.read();
    while (c != 's')
    {
        delay(200);
        c = Serial.read();
    }
    Serial.println("Starting code");
}

void loop() 
{ 
    char c = Serial.read();
    // for (int angle = 0; angle <= 180; angle += 2)
    // {
    //     for (int servo = 6; servo < 12; servo++)
    //     {
    //         // c = Serial.read();
    //         // if (c == 'c')
    //         // {
    //         //     while (c != 's')
    //         //     {
    //         //         delay(100);
    //         //         c = Serial.read();
    //         //     }
    //         // }
    //         board1.setPWM(servo, 0, angleToPulse(angle));
    //     }
    //     delay(100);
    // }

    // for (int angle = 180; angle >= 0; angle -= 2)
    // {
    //     for (int servo = 6; servo < 12; servo++)
    //     {
    //         // c = Serial.read();
    //         // if (c == 'c')
    //         // {
    //         //     while (c != 's')
    //         //     {
    //         //         delay(100);
    //         //         c = Serial.read();
    //         //     }
    //         // }
    //         board1.setPWM(servo, 0, angleToPulse(angle));
    //     }
    //     delay(100);
    // }
    static int angle = 0;

    // if (c == 'c')
    // {
    //     while (c != 's')
    //     {
    //         delay(100);
    //         c = Serial.read();
    //     }
    // }
    if (c == 'f')
    {
        if (angle < 180)
        {
            Serial.println("Forward:");
            angle += 10;
            board1.setPWM(6, 0, angleToPulse(angle));
        }
    }
    else if (c == 'b')
    {
        if (angle > 0)
        {
            Serial.println("Backward:");
            angle -= 10;
            board1.setPWM(6, 0, angleToPulse(angle));
        }
    }
}

int angleToPulse(int ang) //gets the angle in degree and returns the pulse width
{  
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
    Serial.print("Angle: ");
    Serial.print(ang);
    Serial.print(" pulse: ");
    Serial.println(pulse);
    return pulse;
}