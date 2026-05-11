#include "test_state.h"

#include "app_config.h"
#include "button.h"
#include "dshot.h"
#include "led.h"
#include "rpm_stats.h"

/*
 * Composite v2 motor QA: 3-plateau staircase (15 / 25 / 40 %) + 4 s
 * spin-down + result phase with per-failed-motor beep cadence.
 *
 * DShot value for a throttle percent. Maps the full 0..2047 DShot range
 * so that e.g. 15 % -> 307 (matches PRD §5 / the v1 implementation).
 */
#define APP_THROTTLE_DSHOT(pct) \
    ((uint16_t)((DSHOT_THROTTLE_MAX + 1U) * (uint32_t)(pct) / 100U))

/* Per-plateau accumulators + spin-down state. File-scope so they live
 * across phase transitions but never alias another module's state. */
static RpmAccumulator   s_acc[3][APP_NUM_MOTORS];
static uint32_t         s_plateau_c_mean[APP_NUM_MOTORS];   /* half-life threshold per motor */
static uint32_t         s_half_life_ticks[APP_NUM_MOTORS];

static volatile TestPhase  s_phase       = TEST_IDLE;
static volatile uint32_t   s_phase_ticks = 0;
static CompositeResult     s_last_result = {0};

/* ------------------------------ transitions -------------------------------- */

static void enter_idle(void)
{
    DShot_StopAll();
    Led_SetMode(LED_OFF);
    s_phase       = TEST_IDLE;
    s_phase_ticks = 0;
}

static void enter_plateau_a(void)
{
    RpmStats_Reset(s_acc[0]);
    RpmStats_Reset(s_acc[1]);
    RpmStats_Reset(s_acc[2]);
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        s_plateau_c_mean[ch]  = 0;
        s_half_life_ticks[ch] = 0;
    }
    Led_SetMode(LED_BLINK_FAST);
    s_phase       = TEST_PLATEAU_A;
    s_phase_ticks = 0;
}

static void enter_plateau_b(void)
{
    s_phase       = TEST_PLATEAU_B;
    s_phase_ticks = 0;
}

static void enter_plateau_c(void)
{
    s_phase       = TEST_PLATEAU_C;
    s_phase_ticks = 0;
}

static void enter_spin_down(void)
{
    /* Snapshot plateau-C means as the per-motor half-life threshold.
     * A motor with no plateau-C telemetry has mean = 0; its half-life
     * stays 0 (never crossed), HalfLife_Evaluate marks it as
     * not-present -> fail. */
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        s_plateau_c_mean[ch] = (s_acc[2][ch].count > 0)
            ? (uint32_t)(s_acc[2][ch].sum / s_acc[2][ch].count)
            : 0U;
    }
    s_phase       = TEST_SPIN_DOWN;
    s_phase_ticks = 0;
}

static void enter_result(void)
{
    DShot_StopAll();

    RpmStats_Evaluate(s_acc[0], APP_PLATEAU_A_TOL_PCT_X10, &s_last_result.plateau[0]);
    RpmStats_Evaluate(s_acc[1], APP_PLATEAU_B_TOL_PCT_X10, &s_last_result.plateau[1]);
    RpmStats_Evaluate(s_acc[2], APP_PLATEAU_C_TOL_PCT_X10, &s_last_result.plateau[2]);
    HalfLife_Evaluate(s_half_life_ticks, APP_HALF_LIFE_TOL_PCT_X10,
                      &s_last_result.spin_down);
    Composite_Aggregate(&s_last_result);

    Led_SetMode(s_last_result.overall_pass ? LED_SOLID_ON : LED_BLINK_SLOW);
    s_phase       = TEST_RESULT;
    s_phase_ticks = 0;
}

/* --------------------------- tick helpers ---------------------------------- */

static void drive_plateau_tick(uint8_t plateau_idx, uint16_t throttle_dshot)
{
    DShot_SendAll(throttle_dshot, true);
    if (s_phase_ticks >= APP_PLATEAU_SKIP_MS) {
        for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
            DShotTelem t = DShot_ConsumeTelem(ch);
            if (t.valid) {
                RpmStats_Sample(s_acc[plateau_idx], ch, t.rpm);
            }
        }
    }
}

static void drive_spin_down_tick(void)
{
    DShot_SendAll(DSHOT_CMD_MOTOR_STOP, true);
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        if (s_half_life_ticks[ch] != 0U) continue;     /* already recorded */
        if (s_plateau_c_mean[ch]  == 0U) continue;     /* no threshold to cross */

        DShotTelem t = DShot_ConsumeTelem(ch);
        if (t.valid && (t.rpm * 2U) < s_plateau_c_mean[ch]) {
            s_half_life_ticks[ch] = s_phase_ticks;
        }
    }
}

static void drive_result_tick(void)
{
    uint16_t values[APP_NUM_MOTORS];

    if (s_phase_ticks < APP_BEACON_DURATION_MS) {
        /* Initial 100 ms unified tone: high chime on pass, low buzz on
         * failed channels only on fail. Sets the operator's
         * expectation immediately. */
        if (s_last_result.overall_pass) {
            for (uint8_t i = 0; i < APP_NUM_MOTORS; ++i) {
                values[i] = APP_BEACON_PASS_CMD;
            }
        } else {
            for (uint8_t i = 0; i < APP_NUM_MOTORS; ++i) {
                values[i] = s_last_result.per_motor_pass[i]
                    ? DSHOT_CMD_MOTOR_STOP
                    : APP_BEACON_FAIL_CMD;
            }
        }
        DShot_SendPerChannel(values, true);
        return;
    }

    if (s_last_result.overall_pass) {
        /* All passed — stay silent, wait for button. LD3 stays solid. */
        return;
    }

    /* Fail-indicate cadence: 400 ms BEACON1 on failed motors, 200 ms
     * silence on all motors, repeat. The silence gap re-triggers the
     * ESC's beacon on each cycle, so the chirps are crisp and
     * countable. */
    const uint32_t cycle_len = APP_FAIL_INDICATE_ON_MS + APP_FAIL_INDICATE_OFF_MS;
    const uint32_t pos       = (s_phase_ticks - APP_BEACON_DURATION_MS) % cycle_len;
    const bool     in_beep   = pos < APP_FAIL_INDICATE_ON_MS;

    for (uint8_t i = 0; i < APP_NUM_MOTORS; ++i) {
        if (in_beep && !s_last_result.per_motor_pass[i]) {
            values[i] = APP_BEACON_FAIL_CMD;
        } else {
            values[i] = DSHOT_CMD_MOTOR_STOP;
        }
    }
    DShot_SendPerChannel(values, true);
}

/* ------------------------------ public API --------------------------------- */

void TestState_Init(void)
{
    enter_idle();
}

void TestState_Tick(void)
{
    const ButtonEvent evt = Button_Read();

    /* Single-press during any active phase aborts to Idle. */
    if (evt == BUTTON_EVENT_SINGLE && s_phase != TEST_IDLE) {
        enter_idle();
        return;
    }

    switch (s_phase) {
    case TEST_IDLE:
        if (evt == BUTTON_EVENT_DOUBLE) {
            enter_plateau_a();
        }
        break;

    case TEST_PLATEAU_A:
        drive_plateau_tick(0, APP_THROTTLE_DSHOT(APP_PLATEAU_A_PCT));
        s_phase_ticks++;
        if (s_phase_ticks >= APP_PLATEAU_DURATION_MS) enter_plateau_b();
        break;

    case TEST_PLATEAU_B:
        drive_plateau_tick(1, APP_THROTTLE_DSHOT(APP_PLATEAU_B_PCT));
        s_phase_ticks++;
        if (s_phase_ticks >= APP_PLATEAU_DURATION_MS) enter_plateau_c();
        break;

    case TEST_PLATEAU_C:
        drive_plateau_tick(2, APP_THROTTLE_DSHOT(APP_PLATEAU_C_PCT));
        s_phase_ticks++;
        if (s_phase_ticks >= APP_PLATEAU_DURATION_MS) enter_spin_down();
        break;

    case TEST_SPIN_DOWN:
        drive_spin_down_tick();
        s_phase_ticks++;
        if (s_phase_ticks >= APP_SPIN_DOWN_DURATION_MS) enter_result();
        break;

    case TEST_RESULT:
        drive_result_tick();
        s_phase_ticks++;
        break;
    }
}

TestPhase TestState_Phase(void)
{
    return s_phase;
}

const CompositeResult *TestState_LastResult(void)
{
    return &s_last_result;
}
