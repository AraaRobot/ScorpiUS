#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <avr/wdt.h>
#include "comm.h"
#include "control.h"
#include "state_machine.h"

#if ENABLE_MANUAL
#include "manual.h"
#endif

void resetArduino()
{
    controlReset();
    delay(20);

    wdt_enable(WDTO_15MS);
    while (1)
    {
        // Watchdog will trigger after 15 ms and reset the arduino
    }
}

void setup()
{
    wdt_disable();
    Serial.begin(115200);
    delay(100);  // Give serial time to initialize

    Wire.begin();
    Wire.setClock(100000);
    delay(100);

    controlInit();
    commInit(Serial);
    COMM_DEBUG("Initialization complete. Entering main loop.");

    static const uint8_t infoPayload[1] = {static_cast<uint8_t>(eInfoCode::INIT_COMPLETE)};
    commSend(eSerialMsgType::INFO, infoPayload, 1);
}

void loop()
{
#if ENABLE_MANUAL
    manual();
#else
    commProcess();
    sAngles angles;
    eSerialMsgType type = commConsume(angles);

    if (type == eSerialMsgType::COMMAND && controllerStateMachine == eStates::RUNNING)
    {
        processAngles(angles);
        updatePosition();
    }
    else if (type == eSerialMsgType::STATE && controllerStateMachine == eStates::HOME)
    {
        goHome();
    }
    else if (type == eSerialMsgType::STATE && controllerStateMachine == eStates::REBOOT)
    {
        resetArduino();
    }

    static unsigned long lastHeartbeat = 0;
    unsigned long now = millis();
    // Heartbeat at 2 Hz (every 500 ms)
    if (now - lastHeartbeat >= 500)
    {
        commHeartbeat();
        lastHeartbeat = now;
    }
#endif  // ENABLE_MANUAL
}