#include "rpm_stats.h"

#include <string.h>

typedef struct {
    uint64_t sum;
    uint32_t count;
} accumulator_t;

static accumulator_t s_acc[APP_NUM_MOTORS];

void RpmStats_Reset(void)
{
    memset(s_acc, 0, sizeof(s_acc));
}

void RpmStats_Sample(uint8_t channel, uint32_t rpm)
{
    if (channel >= APP_NUM_MOTORS) {
        return;
    }
    s_acc[channel].sum   += rpm;
    s_acc[channel].count += 1;
}

bool RpmStats_Evaluate(RpmEvalResult *out)
{
    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));

    /* Per-motor means. A motor with zero samples is flagged not-present;
     * PRD §5 says that counts as a failure. */
    uint32_t present_count = 0;
    uint64_t group_sum     = 0;

    for (uint8_t i = 0; i < APP_NUM_MOTORS; ++i) {
        if (s_acc[i].count == 0) {
            out->per_motor[i].present  = false;
            out->per_motor[i].mean_rpm = 0;
            continue;
        }
        out->per_motor[i].present  = true;
        out->per_motor[i].mean_rpm = (uint32_t)(s_acc[i].sum / s_acc[i].count);
        group_sum   += out->per_motor[i].mean_rpm;
        present_count++;
    }

    if (present_count == 0) {
        /* Nothing reported — the entire rig failed. */
        out->group_mean_rpm = 0;
        out->overall_pass   = false;
        return false;
    }

    out->group_mean_rpm = (uint32_t)(group_sum / present_count);

    if (out->group_mean_rpm == 0U) {
        /* Every present motor reported exactly 0 RPM. The deviation check
         * below would divide by zero; treat the batch as failed. The TX/RX
         * path is supposed to filter the motor_stopped sentinel into an
         * invalid telemetry sample (see Core/Src/dshot.c::dshot_decode_rx),
         * so reaching here means something else is wrong (broken ESC, noise). */
        out->overall_pass = false;
        return false;
    }

    /* Per-motor deviation check. We use integer math in tenths-of-percent
     * to avoid floating point. Deviation = (mean - group) * 1000 / group. */
    bool all_pass = true;
    const int32_t tol_x10 = (int32_t)APP_TOLERANCE_PERCENT * 10;

    for (uint8_t i = 0; i < APP_NUM_MOTORS; ++i) {
        RpmMotorResult *m = &out->per_motor[i];
        if (!m->present) {
            m->pass     = false;
            m->deviation_pct_x10 = 0;
            all_pass    = false;
            continue;
        }
        int32_t delta = (int32_t)m->mean_rpm - (int32_t)out->group_mean_rpm;
        /* Tenths of a percent: delta * 1000 / group, then expressed × 10 of % */
        int32_t dev_x10 = (int32_t)(((int64_t)delta * 1000) / (int64_t)out->group_mean_rpm);
        m->deviation_pct_x10 = dev_x10;
        m->pass     = (dev_x10 >= -tol_x10) && (dev_x10 <= tol_x10);
        all_pass    = all_pass && m->pass;
    }

    out->overall_pass = all_pass;
    return all_pass;
}
