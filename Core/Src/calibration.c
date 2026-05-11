#include "calibration.h"

#include "app_config.h"

#include <string.h>

/*
 * Per-phase running sum-of-squares of per-motor deviations
 * (`RpmEvalResult::per_motor[i].deviation_pct_x10`). Deviations are
 * centred at 0 by construction (the group mean is the mean across
 * motors per cycle), so sigma is sqrt(sum_sq / n) with no mean
 * subtraction needed.
 *
 * Stored value is Σ(dev_x10²). One dev_x10 is at most a few hundred
 * (deviation in tenths of a percent), so squares fit in uint32 and
 * the accumulated total over 80 samples fits in uint64 trivially.
 */
static struct {
    uint64_t sum_sq;
    uint32_t n_samples;
} s_phase[CALIBRATION_NUM_PHASES];

static uint32_t s_cycle_count;

/* Newton-Raphson integer square root. value < 2^64; result < 2^32.
 * Returns floor(sqrt(value)). 0 -> 0, 1 -> 1. */
uint32_t Calibration_ISqrt(uint64_t value)
{
    if (value < 2U) {
        return (uint32_t)value;
    }
    uint64_t x   = value;
    uint64_t y   = (x + 1U) >> 1;
    while (y < x) {
        x = y;
        y = (x + value / x) >> 1;
    }
    return (uint32_t)x;
}

static void fold_dev_into_phase(uint8_t phase, int32_t dev_x10)
{
    uint64_t sq = (uint64_t)((int64_t)dev_x10 * (int64_t)dev_x10);
    s_phase[phase].sum_sq    += sq;
    s_phase[phase].n_samples += 1U;
}

void Calibration_Reset(void)
{
    memset(s_phase, 0, sizeof(s_phase));
    s_cycle_count = 0;
}

void Calibration_FoldCycle(const CompositeResult *r)
{
    if (r == NULL) return;
    for (uint8_t i = 0; i < APP_NUM_MOTORS; ++i) {
        for (uint8_t p = 0; p < CALIBRATION_NUM_PHASES; ++p) {
            /* Phases 0..2 are the three plateaus; phase 3 is the
             * spin-down half-life check. Same RpmEvalResult shape, just
             * different sub-structs. */
            const RpmEvalResult *src = (p < 3U) ? &r->plateau[p]
                                                : &r->spin_down;
            fold_dev_into_phase(p, src->per_motor[i].deviation_pct_x10);
        }
    }
    s_cycle_count++;
}

void Calibration_Summarize(CalibrationSummary *out)
{
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));

    /* The four default tolerances live in app_config.h and act as a
     * floor: a quieter-than-expected batch shouldn't tighten below the
     * conservative starting point. */
    out->default_pct_x10[CALIBRATION_PHASE_A]         = APP_PLATEAU_A_TOL_PCT_X10;
    out->default_pct_x10[CALIBRATION_PHASE_B]         = APP_PLATEAU_B_TOL_PCT_X10;
    out->default_pct_x10[CALIBRATION_PHASE_C]         = APP_PLATEAU_C_TOL_PCT_X10;
    out->default_pct_x10[CALIBRATION_PHASE_HALF_LIFE] = APP_HALF_LIFE_TOL_PCT_X10;

    out->n_cycles            = s_cycle_count;
    out->n_samples_per_phase = s_phase[0].n_samples;

    for (uint8_t p = 0; p < CALIBRATION_NUM_PHASES; ++p) {
        if (s_phase[p].n_samples == 0U) {
            out->sigma_pct_x10[p]       = 0U;
            out->recommended_pct_x10[p] = out->default_pct_x10[p];
            continue;
        }
        uint64_t mean_sq = s_phase[p].sum_sq / s_phase[p].n_samples;
        uint32_t sigma   = Calibration_ISqrt(mean_sq);
        if (sigma > 0xFFFFU) sigma = 0xFFFFU;
        out->sigma_pct_x10[p] = (uint16_t)sigma;

        /* 3σ rule: ~99.7 % of samples in a normal distribution fall
         * within ±3σ, so any motor outside that envelope is treated
         * as a defect. The floor keeps the threshold from tightening
         * below the design baseline on an unusually quiet batch. */
        uint32_t three_sigma = sigma * 3U;
        if (three_sigma > 0xFFFFU) three_sigma = 0xFFFFU;

        out->recommended_pct_x10[p] = (three_sigma > out->default_pct_x10[p])
            ? (uint16_t)three_sigma
            : out->default_pct_x10[p];
    }
}
