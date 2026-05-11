#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * PRD-level tunables. Edit here, not at the call site.
 * Cross-referenced with docs/motor_test_rig_prd.md.
 */

#define APP_NUM_MOTORS              4

/* PRD §5: 15% throttle => DShot value 307 (in the 48..2047 throttle range). */
#define APP_THROTTLE_PERCENT        15

/* PRD §5: 10 s total run, skip the first 2 s of telemetry. */
#define APP_TEST_DURATION_MS        10000U
#define APP_STARTUP_SKIP_MS         2000U

/* PRD §5: ±5% deviation from the 4-motor mean. */
#define APP_TOLERANCE_PERCENT       5

/* PRD §4: 14-pole motors. eRPM -> mechanical RPM = eRPM * 2 / poles. */
#define APP_MOTOR_POLE_COUNT        14

/* PRD §5 / §8: double-press window between two presses. */
#define APP_DOUBLE_PRESS_WINDOW_MS  400U

/* PRD §5: 5 Hz fast-blink while running, 2 Hz slow-blink on failure. */
#define APP_LED_BLINK_FAST_HZ       5U
#define APP_LED_BLINK_SLOW_HZ       2U

/* DShot300 TX: 300 kbit/s. With TIM1 at 80 MHz, ARR+1 = 267 gives one
 * bit per ~3.34 us. T1H ~= 75% * Tbit, T0H ~= 37.5% * Tbit. */
#define APP_DSHOT_ARR               266U
#define APP_DSHOT_T1H               200U   /* CCR for a logical 1 */
#define APP_DSHOT_T0H               100U   /* CCR for a logical 0 */

/* DShot300 RX (input capture, after end-of-frame). PSC=7 -> 10 MHz tick
 * (0.1 us); ARR=1500 -> 150 us RX timeout window; nominal bit cell at the
 * same 300 kbit/s rate -> 33 ticks. Decoder samples mid-cell so it
 * tolerates +/- 0.5 bit-cell of ESC clock drift. */
#define APP_DSHOT_RX_PSC            7U
#define APP_DSHOT_RX_ARR            1500U
#define APP_DSHOT_RX_BIT_TICKS      33U

#endif /* APP_CONFIG_H */
