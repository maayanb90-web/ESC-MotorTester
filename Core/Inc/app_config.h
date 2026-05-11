#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * PRD-level tunables. Edit here, not at the call site.
 * Cross-referenced with docs/motor_test_rig_prd.md.
 */

#define APP_NUM_MOTORS              4

/* PRD §4: 14-pole motors. eRPM -> mechanical RPM = eRPM * 2 / poles. */
#define APP_MOTOR_POLE_COUNT        14

/* Throttle percent (0..100, of the full 0..2047 DShot range) -> raw
 * DShot value. 15 % maps to ~307, matching the PRD §5 example. Lives
 * here rather than in test_state.c so any future caller (calibration
 * tool, alternative test profile) gets the same conversion. */
#define APP_THROTTLE_DSHOT(pct) \
    ((uint16_t)(((uint32_t)(pct) * 2048U) / 100U))

/* PRD §5 / §8: double-press window between two presses. */
#define APP_DOUBLE_PRESS_WINDOW_MS  400U

/* PRD §5: 5 Hz fast-blink while running, 2 Hz slow-blink on failure. */
#define APP_LED_BLINK_FAST_HZ       5U
#define APP_LED_BLINK_SLOW_HZ       2U

/* Composite test (v2): staircase of three throttle plateaus + spin-down.
 * Replaces the single 15%/10s test from PRD §5. Throttle range
 * 10-40% per bench safety (no-prop Mad 650KV at 6S free-spins to
 * ~16k RPM at higher throttle). Total run time: 3*3000 + 4000 = 13 s. */
#define APP_PLATEAU_A_PCT           15U
#define APP_PLATEAU_B_PCT           25U
#define APP_PLATEAU_C_PCT           40U
#define APP_PLATEAU_DURATION_MS     3000U
#define APP_PLATEAU_SKIP_MS         500U    /* skip startup transient on each plateau */
#define APP_SPIN_DOWN_DURATION_MS   4000U   /* coast-down telemetry window */

/* Per-phase tolerances in tenths-of-percent (75 = 7.5%). Defaults are
 * empirical mid-points: tight enough to catch the original PRD failure,
 * loose enough that normal in-family variance doesn't false-fail.
 * Calibrate against a known-good batch and set each to max(default, 3σ). */
#define APP_PLATEAU_A_TOL_PCT_X10   70U     /* ±7.0% */
#define APP_PLATEAU_B_TOL_PCT_X10   80U     /* ±8.0% */
#define APP_PLATEAU_C_TOL_PCT_X10   100U    /* ±10.0% — slip noise grows with throttle */
#define APP_HALF_LIFE_TOL_PCT_X10   200U    /* ±20.0% — bearing variance is wide */

/* DShot300 TX: 300 kbit/s. With TIM1 at 80 MHz, ARR+1 = 267 gives one
 * bit per ~3.34 us. T1H ~= 75% * Tbit, T0H ~= 37.5% * Tbit. */
#define APP_DSHOT_ARR               266U
#define APP_DSHOT_T1H               200U   /* CCR for a logical 1 */
#define APP_DSHOT_T0H               100U   /* CCR for a logical 0 */

/* Audible pass/fail feedback via DShot beacons (motor-as-speaker).
 * Played at the start of the TEST_RESULT phase. High pitch = pass,
 * low pitch = fail. BLHeli requires >=6 consecutive frames before
 * acting; 100 frames at 1 kHz is comfortably above the minimum.
 *
 * Values are the raw DShot command codes so this header doesn't have
 * to pull in dshot.h (which would be circular: dshot.h includes us).
 * If you change them, keep them aligned with DSHOT_CMD_BEACON{1..5}
 * in Core/Inc/dshot.h. */
#define APP_BEACON_DURATION_MS      100U
#define APP_BEACON_PASS_CMD         5U   /* DSHOT_CMD_BEACON5 — ~870 Hz chime */

/* Per-motor fail-beacon mapping. Motor i is beeped with
 * DSHOT_CMD_BEACON(i+1) so the four channels carry four distinct
 * pitches (~250 / 280 / 330 / 430 Hz). Lets the operator identify
 * which motor(s) are buzzing by ear when several fail at once.
 * BEACON5 stays reserved for the all-pass chime above. */
#define APP_BEACON_FAIL_CMD_PER_MOTOR  { 1U, 2U, 3U, 4U }

/* Auto-calibration: triple-press from Idle runs this many composite
 * cycles back-to-back on a known-good batch, then prints recommended
 * per-phase tolerances over the CSV log. 20 cycles takes ~4-5 min and
 * gives 80 samples per phase (4 motors x 20 cycles) for the sigma
 * estimate. */
#define APP_CALIBRATION_CYCLES      20U

/* Power-on self-test: send bidir MOTOR_STOP frames for N ticks at
 * boot and require each channel to return at least M CRC-valid
 * telemetry frames. Catches dead ESC / wrong pin map / broken
 * wiring before the operator commits to a full 13-s test cycle.
 * Motors stay still throughout — every frame is MOTOR_STOP. */
#define APP_POST_DURATION_MS        100U
#define APP_POST_MIN_VALID_FRAMES   50U    /* >= 50 of ~100 frames */

/* After the initial 100 ms pass/fail tone, if the test failed the rig
 * loops a per-motor indicator cadence: 400 ms BEACON1 on each failed
 * channel + 200 ms silence (MOTOR_STOP) on all channels. The silence
 * gap re-triggers the ESC's beacon (avoiding anti-repeat lockouts) and
 * lets the operator count individual chirps if multiple motors failed. */
#define APP_FAIL_INDICATE_ON_MS     400U
#define APP_FAIL_INDICATE_OFF_MS    200U

/* DShot300 RX (input capture, after end-of-frame). PSC=7 -> 10 MHz tick
 * (0.1 us); ARR=1500 -> 150 us RX timeout window; nominal bit cell at the
 * same 300 kbit/s rate -> 33 ticks. Decoder samples mid-cell so it
 * tolerates +/- 0.5 bit-cell of ESC clock drift. */
#define APP_DSHOT_RX_PSC            7U
#define APP_DSHOT_RX_ARR            1500U
#define APP_DSHOT_RX_BIT_TICKS      33U

#endif /* APP_CONFIG_H */
