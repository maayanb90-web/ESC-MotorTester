#ifndef RPM_STATS_H
#define RPM_STATS_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"

/*
 * Per-motor running mean of RPM samples plus a +/- tolerance check
 * against the across-motor mean. The accumulator is caller-owned so
 * multiple windows can coexist (e.g. one per staircase plateau).
 *
 * Usage:
 *   RpmAccumulator s[APP_NUM_MOTORS];
 *   RpmStats_Reset(s);
 *   ...each tick: RpmStats_Sample(s, ch, rpm);
 *   RpmEvalResult r;
 *   bool ok = RpmStats_Evaluate(s, tolerance_pct_x10, &r);
 *
 * `tolerance_pct_x10` is tenths of a percent — e.g. 75 means +/- 7.5%.
 * A motor whose accumulator is empty (no telemetry) is marked
 * present=false and fails the check (PRD §5).
 */

typedef struct {
    uint64_t sum;
    uint32_t count;
} RpmAccumulator;

typedef struct {
    bool     present;           /* at least one telem sample arrived */
    bool     pass;              /* within tolerance of group mean    */
    uint32_t mean_rpm;          /* per-motor mean over the window    */
    int32_t  deviation_pct_x10; /* deviation from group mean × 10    */
} RpmMotorResult;

typedef struct {
    uint32_t        group_mean_rpm;
    bool            overall_pass;
    RpmMotorResult  per_motor[APP_NUM_MOTORS];
} RpmEvalResult;

void RpmStats_Reset   (RpmAccumulator s[APP_NUM_MOTORS]);
void RpmStats_Sample  (RpmAccumulator s[APP_NUM_MOTORS], uint8_t channel, uint32_t rpm);
bool RpmStats_Evaluate(const RpmAccumulator s[APP_NUM_MOTORS],
                       uint16_t tolerance_pct_x10,
                       RpmEvalResult *out);

/*
 * Half-life check: each motor has one tick value representing the time
 * from "motor-stop sent" until its RPM dropped below half its plateau
 * mean. A value of 0 means "never crossed the threshold" → no telem →
 * fails. Re-uses the RpmEvalResult shape so the test-state can treat
 * its output uniformly with the plateau evaluators.
 */
bool HalfLife_Evaluate(const uint32_t half_life_ticks[APP_NUM_MOTORS],
                       uint16_t tolerance_pct_x10,
                       RpmEvalResult *out);

/*
 * Composite result: the three staircase plateau evaluations + the spin-
 * down half-life evaluation, plus the per-motor AND of all four. The
 * test-state machine reads `per_motor_pass[]` to decide which channels
 * to beep during the failure-indicate cadence.
 */
typedef struct {
    RpmEvalResult plateau[3];                       /* A, B, C */
    RpmEvalResult spin_down;                        /* half-life */
    bool          per_motor_pass[APP_NUM_MOTORS];   /* AND across all 4 sub-results */
    bool          overall_pass;                     /* AND across motors */
} CompositeResult;

void Composite_Aggregate(CompositeResult *r);

#endif /* RPM_STATS_H */
