#include "test_state.h"

#include "app_config.h"
#include "button.h"
#include "calibration.h"
#include "display.h"
#include "dshot.h"
#include "led.h"
#include "main.h"          /* HAL_GetTick for trace timestamps */
#include "nvconfig.h"
#include "rpm_stats.h"
#include "trace.h"

#include <stdio.h>         /* snprintf for display line buffers */

/*
 * Composite v2 motor QA: 3-plateau staircase (15 / 25 / 40 %) + 4 s
 * spin-down + result phase with per-failed-motor beep cadence.
 *
 * The throttle-percent -> DShot-value macro APP_THROTTLE_DSHOT(pct)
 * lives in app_config.h alongside the plateau percent tunables, so
 * any future caller picks up the same conversion.
 */

/* Per-plateau accumulators + spin-down state. File-scope so they live
 * across phase transitions but never alias another module's state. */
static RpmAccumulator   s_acc[3][APP_NUM_MOTORS];
static uint32_t         s_plateau_c_mean[APP_NUM_MOTORS];   /* half-life threshold per motor */
static uint32_t         s_half_life_ticks[APP_NUM_MOTORS];
static uint16_t         s_post_valid_frames[APP_NUM_MOTORS]; /* link-test frame counter per channel */

static const uint16_t   s_fail_cmd_per_motor[APP_NUM_MOTORS] =
    APP_BEACON_FAIL_CMD_PER_MOTOR;

static volatile TestPhase  s_phase       = TEST_IDLE;
static volatile uint32_t   s_phase_ticks = 0;
static CompositeResult     s_last_result = {0};
static uint32_t            s_cycle_id    = 0;
/* True while a test cycle is in flight (between enter_plateau_a and
 * either enter_result or enter_idle-on-abort). Distinguishes mid-test
 * abort from the operator clearing a finished cycle. */
static bool                s_active_run         = false;
/* Auto-calibration: when active, enter_result folds the cycle into the
 * Calibration accumulators and loops back to plateau A until N cycles
 * have completed; only then does the summary print and we transition to
 * TEST_CALIBRATION_DONE. Single-press during a calibration aborts the
 * whole run (no summary). */
static bool                s_calibration_active = false;
static uint32_t            s_calibration_cycle  = 0;
/* True iff the last calibration's NvConfig_Save succeeded. Drives the
 * LED state in TEST_CALIBRATION_DONE and gates the 2 s BEACON1 alarm. */
static bool                s_calibration_save_ok = true;

/* Runtime per-phase tolerances. Initialised at boot from
 * NvConfig_Load (if flash holds valid data) or the compile-time
 * defaults. Updated immediately after a successful NvConfig_Save so
 * the next test cycle uses the new values without a reboot. */
static uint16_t            s_runtime_tol[CALIBRATION_NUM_PHASES] = {
    APP_PLATEAU_A_TOL_PCT_X10,
    APP_PLATEAU_B_TOL_PCT_X10,
    APP_PLATEAU_C_TOL_PCT_X10,
    APP_HALF_LIFE_TOL_PCT_X10,
};

/* ------------------------------ transitions -------------------------------- */

static void enter_idle(void)
{
    DShot_StopAll();
    Led_SetMode(LED_OFF);

    /* If we're aborting from an active test (not from TEST_RESULT after
     * the cycle finished), log the abort so the host can see the gap. */
    if (s_active_run) {
        Trace_PrintResult(NULL, ++s_cycle_id, HAL_GetTick(), true);
        s_active_run = false;
    }
    /* A mid-calibration abort drops the summary entirely. The cycles
     * already emitted are still in the host log; the operator can
     * compute sigma offline if needed. */
    s_calibration_active = false;
    s_calibration_cycle  = 0;

    Display_Status("READY", "2x press = TEST", "3x press = CAL");

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
    s_active_run  = true;
    if (s_calibration_active) {
        char l2[24];
        (void)snprintf(l2, sizeof(l2), "cycle %lu/%u",
                       (unsigned long)(s_calibration_cycle + 1U),
                       (unsigned)APP_CALIBRATION_CYCLES);
        Display_Status("CALIBRATING", l2, "");
    } else {
        Display_Status("TEST 15%", "PLATEAU A  1/3", "");
    }
    s_phase       = TEST_PLATEAU_A;
    s_phase_ticks = 0;
}

static void enter_plateau_b(void)
{
    if (!s_calibration_active) {
        Display_Status("TEST 25%", "PLATEAU B  2/3", "");
    }
    s_phase       = TEST_PLATEAU_B;
    s_phase_ticks = 0;
}

static void enter_plateau_c(void)
{
    if (!s_calibration_active) {
        Display_Status("TEST 40%", "PLATEAU C  3/3", "");
    }
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
    if (!s_calibration_active) {
        Display_Status("COOL-DOWN", "SPIN-DOWN  4/4", "");
    }
    s_phase       = TEST_SPIN_DOWN;
    s_phase_ticks = 0;
}

static void enter_result(void)
{
    DShot_StopAll();

    RpmStats_Evaluate(s_acc[0], s_runtime_tol[0], &s_last_result.plateau[0]);
    RpmStats_Evaluate(s_acc[1], s_runtime_tol[1], &s_last_result.plateau[1]);
    RpmStats_Evaluate(s_acc[2], s_runtime_tol[2], &s_last_result.plateau[2]);
    HalfLife_Evaluate(s_half_life_ticks, s_runtime_tol[3],
                      &s_last_result.spin_down);
    Composite_Aggregate(&s_last_result);

    Trace_PrintResult(&s_last_result, ++s_cycle_id, HAL_GetTick(), false);
    s_active_run = false;

    if (s_calibration_active) {
        /* Fold this cycle's deviations into the calibration
         * accumulators. If more cycles remain, loop straight back into
         * plateau A without entering the TEST_RESULT beacon phase.
         * Otherwise summarise, save to flash, and finalise. */
        Calibration_FoldCycle(&s_last_result);
        s_calibration_cycle++;

        if (s_calibration_cycle < APP_CALIBRATION_CYCLES) {
            enter_plateau_a();
            return;
        }

        CalibrationSummary summary;
        Calibration_Summarize(&summary);
        Trace_PrintCalibration(&summary);

        /* Persist to flash. On success, update s_runtime_tol[] so the
         * next test cycle uses the new values without a reboot. On
         * failure, runtime keeps the previous values; LED + BEACON1
         * alarm in TEST_CALIBRATION_DONE flags the issue. */
        NvConfig cfg = {
            .magic         = NVCONFIG_MAGIC,
            .version       = NVCONFIG_SCHEMA_VERSION,
            .plat_a_x10    = summary.recommended_pct_x10[CALIBRATION_PHASE_A],
            .plat_b_x10    = summary.recommended_pct_x10[CALIBRATION_PHASE_B],
            .plat_c_x10    = summary.recommended_pct_x10[CALIBRATION_PHASE_C],
            .half_life_x10 = summary.recommended_pct_x10[CALIBRATION_PHASE_HALF_LIFE],
            .reserved      = 0,
            .crc32         = 0,
            .pad           = 0,
        };
        NvConfig_FillCrc(&cfg);

        s_calibration_save_ok = NvConfig_Save(&cfg);
        Trace_PrintConfigSaved(s_calibration_save_ok, 0U);

        if (s_calibration_save_ok) {
            s_runtime_tol[0] = cfg.plat_a_x10;
            s_runtime_tol[1] = cfg.plat_b_x10;
            s_runtime_tol[2] = cfg.plat_c_x10;
            s_runtime_tol[3] = cfg.half_life_x10;
            Led_SetMode(LED_SOLID_ON);
            Display_Status("CAL DONE", "SAVED to flash", "press to clear");
        } else {
            Led_SetMode(LED_BLINK_FAST);
            Display_Status("CAL DONE", "SAVE FAILED", "see # RECOMMEND");
        }

        s_calibration_active = false;
        s_calibration_cycle  = 0;
        s_phase       = TEST_CALIBRATION_DONE;
        s_phase_ticks = 0;
        return;
    }

    Led_SetMode(s_last_result.overall_pass ? LED_SOLID_ON : LED_BLINK_SLOW);

    if (s_last_result.overall_pass) {
        Display_Status("PASS", "all 4 motors good", "press to clear");
    } else {
        char l2[24];
        (void)snprintf(l2, sizeof(l2), "M0:%s M1:%s M2:%s M3:%s",
                       s_last_result.per_motor_pass[0] ? "OK" : "**",
                       s_last_result.per_motor_pass[1] ? "OK" : "**",
                       s_last_result.per_motor_pass[2] ? "OK" : "**",
                       s_last_result.per_motor_pass[3] ? "OK" : "**");
        Display_Status("FAIL", l2, "press to clear");
    }

    s_phase       = TEST_RESULT;
    s_phase_ticks = 0;
}

static void enter_calibration(void)
{
    Calibration_Reset();
    s_calibration_active = true;
    s_calibration_cycle  = 0;
    enter_plateau_a();   /* sets LED, s_active_run, etc. */
}

static void enter_post(void)
{
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        s_post_valid_frames[ch] = 0;
    }
    Led_SetMode(LED_BLINK_FAST);
    Display_Status("SELF-TEST", "checking ESC link...", "");
    s_phase       = TEST_POST;
    s_phase_ticks = 0;
}

/* Send one MOTOR_STOP frame and harvest telemetry. Each tick adds at
 * most one valid frame per channel (the bidir RX path can only deliver
 * one telem update per TX cycle). Over APP_POST_DURATION_MS ticks that
 * gives us ~APP_POST_DURATION_MS chances per channel — well above the
 * APP_POST_MIN_VALID_FRAMES threshold on a healthy link. */
static void drive_post_tick(void)
{
    DShot_SendAll(DSHOT_CMD_MOTOR_STOP, true);
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        DShotTelem t = DShot_ConsumeTelem(ch);
        if (t.valid) {
            s_post_valid_frames[ch]++;
        }
    }
}

static void finalize_post(void)
{
    /* Build a minimal CompositeResult view: only per_motor_pass[] and
     * overall_pass are read by drive_result_tick during the fail-
     * indicate cadence; the plateau / spin_down sub-results are
     * harmlessly zero. */
    memset(&s_last_result, 0, sizeof(s_last_result));
    bool overall = true;
    for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
        const bool ok = (s_post_valid_frames[ch] >= APP_POST_MIN_VALID_FRAMES);
        s_last_result.per_motor_pass[ch] = ok;
        if (!ok) overall = false;
    }
    s_last_result.overall_pass = overall;

    Trace_PrintPost(s_post_valid_frames, overall);

    if (overall) {
        /* Link is healthy — drop straight to Idle, no audio cue. */
        enter_idle();
        return;
    }
    /* Reuse the existing fail-indicate path: per-motor pitched buzz
     * cadence + slow LED blink until the operator clears with a
     * single press. */
    Led_SetMode(LED_BLINK_SLOW);
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
                    : s_fail_cmd_per_motor[i];
            }
        }
        DShot_SendPerChannel(values, true);
        return;
    }

    if (s_last_result.overall_pass) {
        /* All passed — stay silent, wait for button. LD3 stays solid. */
        return;
    }

    /* Fail-indicate cadence: 400 ms of per-motor BEACONs on the failed
     * channels, 200 ms silence on all channels, repeat. Each failed
     * motor gets its own pitch (s_fail_cmd_per_motor) so two failures
     * sound clearly different; the silence gap re-triggers each ESC's
     * beacon on every cycle so the chirps are crisp and countable. */
    const uint32_t cycle_len = APP_FAIL_INDICATE_ON_MS + APP_FAIL_INDICATE_OFF_MS;
    const uint32_t pos       = (s_phase_ticks - APP_BEACON_DURATION_MS) % cycle_len;
    const bool     in_beep   = pos < APP_FAIL_INDICATE_ON_MS;

    for (uint8_t i = 0; i < APP_NUM_MOTORS; ++i) {
        if (in_beep && !s_last_result.per_motor_pass[i]) {
            values[i] = s_fail_cmd_per_motor[i];
        } else {
            values[i] = DSHOT_CMD_MOTOR_STOP;
        }
    }
    DShot_SendPerChannel(values, true);
}

/* ------------------------------ public API --------------------------------- */

void TestState_Init(void)
{
    /* Load previously-saved tolerances from flash, if any. Falls back
     * to the compile-time defaults in app_config.h on bad magic / bad
     * CRC / unprogrammed page. The boot trace records which source
     * is active so the operator can see it at a glance. */
    NvConfig cfg;
    if (NvConfig_Load(&cfg)) {
        s_runtime_tol[0] = cfg.plat_a_x10;
        s_runtime_tol[1] = cfg.plat_b_x10;
        s_runtime_tol[2] = cfg.plat_c_x10;
        s_runtime_tol[3] = cfg.half_life_x10;
        Trace_PrintConfigLoaded(s_runtime_tol, "flash");
    } else {
        Trace_PrintConfigLoaded(s_runtime_tol, "defaults");
    }

    /* The boot path runs POST first; finalize_post drops to Idle on
     * success or transitions into the fail-indicate cadence on any
     * channel failure. */
    enter_post();
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
        } else if (evt == BUTTON_EVENT_TRIPLE) {
            enter_calibration();
        }
        break;

    case TEST_POST:
        drive_post_tick();
        s_phase_ticks++;
        if (s_phase_ticks >= APP_POST_DURATION_MS) {
            finalize_post();
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

    case TEST_CALIBRATION_DONE:
        /* On save-failure, sound a one-shot BEACON1 alarm on all four
         * motors for the first APP_SAVE_FAIL_BUZZ_MS ticks; LD3 stays
         * fast-blinking until the operator clears with single-press.
         * On save-success the rig is silent, LD3 solid — same UX as
         * any "all done, waiting for clear" state. */
        if (!s_calibration_save_ok && s_phase_ticks < APP_SAVE_FAIL_BUZZ_MS) {
            const uint16_t alarm[APP_NUM_MOTORS] = {
                DSHOT_CMD_BEACON1, DSHOT_CMD_BEACON1,
                DSHOT_CMD_BEACON1, DSHOT_CMD_BEACON1,
            };
            DShot_SendPerChannel(alarm, true);
        }
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
