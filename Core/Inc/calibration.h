#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

#include "rpm_stats.h"

/*
 * Auto-calibration: accumulates per-phase deviation samples across N
 * back-to-back composite cycles on a known-good batch, then summarises
 * the natural variance as a recommended tolerance per phase. Replaces
 * the manual "process a known-good batch and compute 3 sigma in a
 * spreadsheet" workflow from PRD section 8.
 *
 * Phases tracked: plateau A, plateau B, plateau C, spin-down half-life.
 * One sigma per phase across all (cycle x motor) deviations.
 *
 * Usage from the test state machine:
 *
 *   Calibration_Reset();
 *   for (cycle = 0; cycle < N; cycle++) {
 *       run a composite cycle, compute CompositeResult r;
 *       Calibration_FoldCycle(&r);
 *   }
 *   CalibrationSummary s;
 *   Calibration_Summarize(&s);
 *   Trace_PrintCalibration(&s);
 */

#define CALIBRATION_PHASE_A          0
#define CALIBRATION_PHASE_B          1
#define CALIBRATION_PHASE_C          2
#define CALIBRATION_PHASE_HALF_LIFE  3
#define CALIBRATION_NUM_PHASES       4

typedef struct {
    uint16_t sigma_pct_x10[CALIBRATION_NUM_PHASES];
    uint16_t recommended_pct_x10[CALIBRATION_NUM_PHASES];
    uint16_t default_pct_x10[CALIBRATION_NUM_PHASES];
    uint32_t n_cycles;
    uint32_t n_samples_per_phase; /* n_cycles * APP_NUM_MOTORS */
} CalibrationSummary;

void     Calibration_Reset    (void);
void     Calibration_FoldCycle(const CompositeResult *r);
void     Calibration_Summarize(CalibrationSummary *out);

/* Newton-Raphson integer square root. Exposed so the host test harness
 * can pin its behaviour without linking the full calibration module. */
uint32_t Calibration_ISqrt(uint64_t value);

#endif /* CALIBRATION_H */
