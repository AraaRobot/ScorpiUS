#if ENABLE_DEBUG

#ifndef DEBUG_H
#define DEBUG_H

#include "comm.h"
#include "control.h"
#include "state_machine.h"


void debug();
void executeDebugFunc();
void jogServo();
void jogMultiple();
void testLegJoints();

#endif // DEBUG_H
#endif // ENABLE_DEBUG