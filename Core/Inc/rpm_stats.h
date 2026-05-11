#ifndef RPM_STATS_H
#define RPM_STATS_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"

/*
 * Per-motor running mean of RPM samples over the test window, plus the
 * PRD §5 pass/fail rule: each motor must be within ±5% of the 4-motor mean
 * across the sampling window; any motor with zero samples (no telemetry)
 * fails.
 *
 * Usage:
 *   RpmStats_Reset();
 *   ... each tick during sampling: RpmStats_Sample(ch, rpm);
 *   bool ok = RpmStats_Evaluate(&result);
 */

typedef struct {
    bool     present;          /* at least one telem sample arrived */
    bool     pass;             /* within tolerance of group mean    */
    uint32_t mean_rpm;         /* per-motor mean over the window    */
    int32_t  deviation_pct_x10;/* deviation from group mean × 10    */
} RpmMotorResult;

typedef struct {
    uint32_t        group_mean_rpm;
    bool            overall_pass;
    RpmMotorResult  per_motor[APP_NUM_MOTORS];
} RpmEvalResult;

void RpmStats_Reset(void);
void RpmStats_Sample(uint8_t channel, uint32_t rpm);
bool RpmStats_Evaluate(RpmEvalResult *out);

#endif /* RPM_STATS_H */
