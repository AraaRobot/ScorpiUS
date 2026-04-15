#if ENABLE_MANUAL

#ifndef MANUAL_H
#define MANUAL_H

#include "comm.h"
#include "control.h"
#include "state_machine.h"

void manual();
void executeManualFunc();
void jogServo();
void jogMultiple();
void testLegJoints();

#endif  // MANUAL_H
#endif  // ENABLE_MANUAL