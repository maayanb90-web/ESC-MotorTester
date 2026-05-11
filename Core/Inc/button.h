#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Debounced single-button driver for PA0 (USER_BTN, active-low with pull-up).
 *
 * The hardware emits noisy edges; the application wants three semantic
 * events: SINGLE press (release), DOUBLE press (two releases within
 * APP_DOUBLE_PRESS_WINDOW_MS), and LONG_HOLD (>= 1 s — reserved, not used
 * by PRD §6 today but useful for future).
 *
 * Button_Tick() must be called from the SysTick handler at 1 kHz. Events
 * are queued single-shot and consumed by the application via Button_Read().
 */

typedef enum {
    BUTTON_EVENT_NONE   = 0,
    BUTTON_EVENT_SINGLE,
    BUTTON_EVENT_DOUBLE,
    BUTTON_EVENT_TRIPLE,
} ButtonEvent;

void        Button_Init(void);
void        Button_Tick(void);       /* call at 1 kHz from SysTick */
ButtonEvent Button_Read(void);       /* one-shot consume */

#endif /* BUTTON_H */
