#include "rpm_stats.h"

#include <string.h>

void RpmStats_Reset(RpmAccumulator s[APP_NUM_MOTORS])
{
    if (s == NULL) return;
    memset(s, 0, APP_NUM_MOTORS * sizeof(RpmAccumulator));
}

void RpmStats_Sample(RpmAccumulator s[APP_NUM_MOTORS], uint8_t channel, uint32_t rpm)
{
    if (s == NULL || channel >= APP_NUM_MOTORS) {
        return;
    }
    s[channel].sum   += rpm;
    s[channel].count += 1;
}

bool RpmStats_Evaluate(const RpmAccumulator s[APP_NUM_MOTORS],
                       uint16_t tolerance_pct_x10,
                       RpmEvalResult *out)
{
    if (s == NULL || out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));

    uint32_t present_count = 0;
    uint64_t group_sum     = 0;

    for (uint8_t i = 0; i < APP_NUM_MOTORS; ++i) {
        if (s[i].count == 0) {
            out->per_motor[i].present  = false;
            out->per_motor[i].mean_rpm = 0;
            continue;
        }
        out->per_motor[i].present  = true;
        out->per_motor[i].mean_rpm = (uint32_t)(s[i].sum / s[i].count);
        group_sum   += out->per_motor[i].mean_rpm;
        present_count++;
    }

    if (present_count == 0) {
        out->group_mean_rpm = 0;
        out->overall_pass   = false;
        return false;
    }

    out->group_mean_rpm = (uint32_t)(group_sum / present_count);

    if (out->group_mean_rpm == 0U) {
        /* Every present motor reported exactly 0 — divide-by-zero guard. */
        out->overall_pass = false;
        return false;
    }

    bool all_pass = true;
    const int32_t tol_x10 = (int32_t)tolerance_pct_x10;

    for (uint8_t i = 0; i < APP_NUM_MOTORS; ++i) {
        RpmMotorResult *m = &out->per_motor[i];
        if (!m->present) {
            m->pass              = false;
            m->deviation_pct_x10 = 0;
            all_pass             = false;
            continue;
        }
        int32_t delta   = (int32_t)m->mean_rpm - (int32_t)out->group_mean_rpm;
        int32_t dev_x10 = (int32_t)(((int64_t)delta * 1000) / (int64_t)out->group_mean_rpm);
        m->deviation_pct_x10 = dev_x10;
        m->pass              = (dev_x10 >= -tol_x10) && (dev_x10 <= tol_x10);
        all_pass             = all_pass && m->pass;
    }

    out->overall_pass = all_pass;
    return all_pass;
}

bool HalfLife_Evaluate(const uint32_t half_life_ticks[APP_NUM_MOTORS],
                       uint16_t tolerance_pct_x10,
                       RpmEvalResult *out)
{
    /* Re-use RpmStats_Evaluate by packing each motor's half-life as a
     * single sample. Motors that never crossed the threshold (value 0)
     * become empty accumulators -> not-present -> fail. */
    RpmAccumulator s[APP_NUM_MOTORS];
    for (uint8_t i = 0; i < APP_NUM_MOTORS; ++i) {
        if (half_life_ticks[i] == 0) {
            s[i].sum   = 0;
            s[i].count = 0;
        } else {
            s[i].sum   = half_life_ticks[i];
            s[i].count = 1;
        }
    }
    return RpmStats_Evaluate(s, tolerance_pct_x10, out);
}

void Composite_Aggregate(CompositeResult *r)
{
    if (r == NULL) return;

    bool any_motor_failed = false;
    for (uint8_t i = 0; i < APP_NUM_MOTORS; ++i) {
        r->per_motor_pass[i] = r->plateau[0].per_motor[i].pass &&
                               r->plateau[1].per_motor[i].pass &&
                               r->plateau[2].per_motor[i].pass &&
                               r->spin_down.per_motor[i].pass;
        if (!r->per_motor_pass[i]) {
            any_motor_failed = true;
        }
    }
    r->overall_pass = !any_motor_failed;
}
