#ifndef DISPLAY_H
#define DISPLAY_H

#include "app_config.h"

/*
 * Optional bench-display abstraction. Default build has
 * `APP_DISPLAY_ENABLED = 0` (in app_config.h) and every call here
 * expands to a no-op macro that **discards its arguments** — so any
 * snprintf-like work the caller hands in (e.g. formatting a "cycle
 * 17/20" string) is elided at compile time, not just the call.
 *
 * When the operator wires a real panel (SSD1306 OLED on I2C1 PB6/PB7,
 * ST7789 colour TFT on SPI1, or an HD44780 char LCD on an I2C
 * backpack), set APP_DISPLAY_ENABLED = 1 and fill in the driver
 * bodies in Core/Src/display.c. The application code (test_state.c)
 * already calls Display_Status / Display_Statusf at every state
 * transition.
 *
 * The abstraction is intentionally generic three-line text — works
 * for any of the candidate panels without coupling the application
 * to a specific resolution or graphics primitive.
 */

#if APP_DISPLAY_ENABLED == 1

void Display_Init(void);

/* Plain three-line status. Any NULL argument is treated as empty. */
void Display_Status(const char *l1, const char *l2, const char *l3);

/* printf-style helper: l1 is plain text, l2 is the format string for
 * the second line (line 3 is left empty). The driver implementation
 * runs the snprintf internally so the cost is only paid when the
 * display is actually wired. */
void Display_Statusf(const char *l1, const char *l2_fmt, ...);

void Display_Clear(void);

#else

/* Macro stubs: arguments are not evaluated when APP_DISPLAY_ENABLED == 0.
 * Any expensive formatting the caller passes (snprintf, ternaries
 * picking strings, etc.) is fully elided at compile time. */
#define Display_Init()                            ((void)0)
#define Display_Status(l1, l2, l3)                ((void)0)
#define Display_Statusf(l1, l2_fmt, ...)          ((void)0)
#define Display_Clear()                           ((void)0)

#endif /* APP_DISPLAY_ENABLED */

#endif /* DISPLAY_H */
