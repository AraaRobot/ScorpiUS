#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 default address: 0x40
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN  102    // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  522    // this is the 'maximum' pulse length count (out of 4096)

int angleToPulse(int ang);

void setup()
{

    Serial.begin(115200);
    delay(100);  // Give serial time to initialize
    
    Wire.begin();
    Wire.setClock(100000);
    delay(100);
    
    Serial.println("Initializing PCA9685...");
    board1.begin();
    delay(100);
    
    board1.setPWMFreq(50);
    delay(200);
    
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
    for (int angle = 0; angle <= 180; angle += 2)
    {
        for (int servo = 0; servo < 2; servo++)
        {
            // Serial.print("Servo: "); Serial.print(servo\t);
            board1.setPWM(servo, 0, angleToPulse(angle));
        }
        delay(100);
    }

    for (int angle = 180; angle >= 0; angle -= 2)
    {
        for (int servo = 0; servo < 2; servo++)
        {
            // Serial.print("Servo: "); Serial.print(servo\t);
            board1.setPWM(servo, 0, angleToPulse(angle));
        }
        delay(100);
    }

    // char c = Serial.read();
    // static int angle = 0;

    // if (c == 'f')
    // {
    //     if (angle < 180)
    //     {
    //         Serial.println("Forward:");
    //         angle += 10;
    //         board1.setPWM(0, 0, angleToPulse(angle));
    //     }
    // }
    // else if (c == 'b')
    // {
    //     if (angle > 0)
    //     {
    //         Serial.println("Backward:");
    //         angle -= 10;
    //         board1.setPWM(0, 0, angleToPulse(angle));
    //     }
    // }
}

int angleToPulse(int ang) //gets the angle in degree and returns the pulse width
{  
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
    Serial.print("Angle: ");
    Serial.print(ang);
    Serial.print("\tpulse: ");
    Serial.println(pulse);
    return pulse;
}