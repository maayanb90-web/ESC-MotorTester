#ifndef APP_H
#define APP_H

/*
 * Top-level entry points called from CubeMX-generated main().
 *
 *   App_Init() : invoked from the "USER CODE BEGIN 2" block in main.c
 *                after HAL/clocks/peripherals are up. Initializes all
 *                application modules.
 *
 *   App_Tick() : invoked from the SysTick_Handler (1 kHz) — drives the
 *                button debouncer, LED blinker and test state machine.
 *
 *   App_Loop() : invoked from the "while (1)" body in main.c. Currently
 *                a wait-for-interrupt; placeholder for future foreground
 *                work (e.g. UART telemetry output).
 */

void App_Init(void);
void App_Tick(void);
void App_Loop(void);

#endif /* APP_H */
