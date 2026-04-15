#if ENABLE_MANUAL

#ifndef MANUAL_H
#define MANUAL_H

#include "comm.h"
#include "control.h"
#include "state_machine.h"

/**
 * @brief Entry point for manual mode processing.
 *
 * Polls incoming manual commands and dispatches the selected manual action.
 */
void manual();

/**
 * @brief Executes the function associated with the active manual debug state.
 */
void executeManualFunc();

/**
 * @brief Manually jogs a single servo based on user-provided input.
 */
void jogServo();

/**
 * @brief Manually jogs multiple servos in one operation.
 */
void jogMultiple();

/**
 * @brief Runs a manual joint test routine for leg servos.
 */
void testLegJoints();

#endif  // MANUAL_H
#endif  // ENABLE_MANUAL