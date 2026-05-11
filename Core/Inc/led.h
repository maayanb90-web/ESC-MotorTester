#ifndef LED_H
#define LED_H

/*
 * LD3 (PB3, active-high) driver.
 *
 * The PRD §5 reuses LD3 for both run-state and pass/fail result via
 * a blink pattern:
 *
 *   LED_OFF         idle
 *   LED_SOLID_ON    all 4 motors passed
 *   LED_BLINK_SLOW  at least one motor failed (~2 Hz)
 *   LED_BLINK_FAST  test running (~5 Hz)
 *
 * Led_Tick() must be called from SysTick at 1 kHz.
 */

typedef enum {
    LED_OFF = 0,
    LED_SOLID_ON,
    LED_BLINK_SLOW,
    LED_BLINK_FAST,
} LedMode;

void Led_Init(void);
void Led_SetMode(LedMode mode);
void Led_Tick(void);

#endif /* LED_H */
