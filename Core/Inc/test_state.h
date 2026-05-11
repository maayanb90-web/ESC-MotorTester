#ifndef TEST_STATE_H
#define TEST_STATE_H

#include <stdint.h>

#include "rpm_stats.h"

/*
 * Test cycle state machine. One instance, driven from the 1 kHz tick.
 *
 *   Idle    : motors off. Double-press -> Running.
 *   Running : motors at 15% throttle, sampling RPM. 10 s elapsed or
 *             single-press -> stop motors and evaluate.
 *   Result  : LED indicates pass/fail. Single-press -> Idle.
 */

typedef enum {
    TEST_IDLE = 0,
    TEST_RUNNING,
    TEST_RESULT,
} TestPhase;

void       TestState_Init(void);
void       TestState_Tick(void);                /* 1 kHz */
TestPhase  TestState_Phase(void);
const RpmEvalResult *TestState_LastResult(void);

#endif /* TEST_STATE_H */
