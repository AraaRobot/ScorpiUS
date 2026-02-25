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

static int angleToPulse(int ang)  // gets the angle in degree and returns the pulse width
{
    int pulse = map(ang, -90, 90, SERVOMIN, SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max
    // Serial.print("Angle: ");
    // Serial.print(ang);
    // Serial.print("\tpulse: ");
    // Serial.println(pulse);
    return pulse;
}

void servoGoTo(eServo servoId, int angle)
{
    int desiredAngle = 0;

    switch (servoId)
    {
        case eServo::A_0:
        case eServo::B_0:
        case eServo::C_0:
        case eServo::D_0:
        case eServo::E_0:
        case eServo::F_0:
            desiredAngle = constrain(angle, MIN_ANGLE_VERTICAL, MAX_ANGLE_VERTICAL);
            driverModule.setPWM(static_cast<uint8_t>(servoId), 0, angleToPulse(desiredAngle));
            break;
        case eServo::A_1:
        case eServo::B_1:
        case eServo::C_1:
        case eServo::D_1:
        case eServo::E_1:
        case eServo::F_1:
            desiredAngle = constrain(angle, MIN_ANGLE_HORIZONTAL, MAX_ANGLE_HORIZONTAL);
            driverModule.setPWM(static_cast<uint8_t>(servoId), 0, angleToPulse(desiredAngle));
            break;
        default:
            Serial.println("Invalid servo ID");
            break;
    }
}