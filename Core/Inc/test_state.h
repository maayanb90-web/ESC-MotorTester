#ifndef TEST_STATE_H
#define TEST_STATE_H

#include <stdint.h>

#include "rpm_stats.h"

/*
 * Composite test cycle state machine. One instance, driven from the
 * 1 kHz tick.
 *
 *   Idle       : motors off. Double-press → PlateauA.
 *   PlateauA   : motors at APP_PLATEAU_A_PCT throttle, sampling RPM.
 *                After APP_PLATEAU_DURATION_MS → PlateauB.
 *   PlateauB   : motors at APP_PLATEAU_B_PCT throttle. → PlateauC.
 *   PlateauC   : motors at APP_PLATEAU_C_PCT throttle. → SpinDown.
 *   SpinDown   : motors commanded stop; record each motor's tick at
 *                which its RPM crosses below half its plateau-C mean.
 *                → Result.
 *   Result     : evaluate composite, drive LED + beacon, then loop
 *                the per-failed-motor cadence (if any) until cleared.
 *                Single press → Idle.
 *
 * Single press at any active phase aborts to Idle silently (PRD §5).
 */

typedef enum {
    TEST_IDLE = 0,
    TEST_PLATEAU_A,
    TEST_PLATEAU_B,
    TEST_PLATEAU_C,
    TEST_SPIN_DOWN,
    TEST_RESULT,
    TEST_CALIBRATION_DONE,
} TestPhase;

void                        TestState_Init(void);
void                        TestState_Tick(void);            /* 1 kHz */
TestPhase                   TestState_Phase(void);
const CompositeResult      *TestState_LastResult(void);

#endif /* TEST_STATE_H */
