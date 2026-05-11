#include "test_state.h"

#include "app_config.h"
#include "button.h"
#include "dshot.h"
#include "led.h"
#include "rpm_stats.h"

/* DShot value for 15% throttle:
 *
 *   throttle range = 48..2047  (DShot reserves 0..47 for special cmds)
 *   span           = 2047 - 48 = 1999
 *   value          = 48 + (span * pct) / 100
 *
 * 15% -> 48 + 1999 * 15 / 100 = 48 + 299 = 347, but PRD §5 uses the
 * "≈ DShot value 307" interpretation that maps 15% of the *full* DShot
 * range (0..2047). We honor the PRD by deriving from the full range so
 * 15% reads 307. Either interpretation is defensible; this comment records
 * the choice so it survives review.
 */
#define APP_THROTTLE_DSHOT \
    ((uint16_t)((DSHOT_THROTTLE_MAX + 1U) * (uint32_t)APP_THROTTLE_PERCENT / 100U))

static volatile TestPhase     s_phase        = TEST_IDLE;
static volatile uint32_t      s_phase_ticks  = 0;
static RpmEvalResult          s_last_result  = {0};

static void enter_idle(void)
{
    DShot_StopAll();
    Led_SetMode(LED_OFF);
    s_phase       = TEST_IDLE;
    s_phase_ticks = 0;
}

static void enter_running(void)
{
    RpmStats_Reset();
    Led_SetMode(LED_BLINK_FAST);    /* "armed/running" indication */
    s_phase       = TEST_RUNNING;
    s_phase_ticks = 0;
}

static void enter_result(void)
{
    DShot_StopAll();
    RpmStats_Evaluate(&s_last_result);
    Led_SetMode(s_last_result.overall_pass ? LED_SOLID_ON : LED_BLINK_SLOW);
    s_phase       = TEST_RESULT;
    s_phase_ticks = 0;
}

void TestState_Init(void)
{
    enter_idle();
}

void TestState_Tick(void)
{
    const ButtonEvent evt = Button_Read();

    switch (s_phase) {
    case TEST_IDLE:
        if (evt == BUTTON_EVENT_DOUBLE) {
            enter_running();
        }
        break;

    case TEST_RUNNING: {
        /* Single press aborts immediately, no evaluation. */
        if (evt == BUTTON_EVENT_SINGLE) {
            enter_idle();
            break;
        }

        /* Keep the DShot frame fresh — ESCs disarm if commands stop arriving.
         * Send at 1 kHz to be well above the typical 4 kHz / 8 kHz BLHeli
         * disarm-watchdog minimum (~10 Hz). */
        DShot_SendAll(APP_THROTTLE_DSHOT, true);

        /* After the startup-skip window, harvest telemetry into the running
         * mean per motor. */
        if (s_phase_ticks >= APP_STARTUP_SKIP_MS) {
            for (uint8_t ch = 0; ch < APP_NUM_MOTORS; ++ch) {
                DShotTelem t = DShot_ConsumeTelem(ch);
                if (t.valid) {
                    RpmStats_Sample(ch, t.rpm);
                }
            }
        }

        s_phase_ticks++;
        if (s_phase_ticks >= APP_TEST_DURATION_MS) {
            enter_result();
        }
        break;
    }

    case TEST_RESULT:
        if (evt == BUTTON_EVENT_SINGLE) {
            enter_idle();
        }
        break;
    }
}

TestPhase TestState_Phase(void)
{
    return s_phase;
}

const RpmEvalResult *TestState_LastResult(void)
{
    return &s_last_result;
}
