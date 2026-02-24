#include "control.h"
#include <Adafruit_PWMServoDriver.h>

// PCA9685 default address: 0x40
static Adafruit_PWMServoDriver driverModule;

void controlInit()
{
    driverModule = Adafruit_PWMServoDriver(0x40);
    driverModule.begin();
    delay(100);
    
    driverModule.setPWMFreq(50);
    delay(200);
}

static int angleToPulse(int ang) //gets the angle in degree and returns the pulse width
{  
    if (ang < -90 || ang > 90) return map(90, 0, 180, SERVOMIN, SERVOMAX);
    int pulse = map(ang, -90, 90, SERVOMIN, SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
    //Serial.print("Angle: ");
    //Serial.print(ang);
    //Serial.print("\tpulse: ");
    //Serial.println(pulse);
    return pulse;
}

void servoGoTo(eServo servoId, int angle)
{
    if (static_cast<uint8_t>(servoId) < 0 || static_cast<uint8_t>(servoId) > 15) return;

    driverModule.setPWM(static_cast<uint8_t>(servoId), 0, angleToPulse(angle));
}