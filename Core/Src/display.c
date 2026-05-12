#include "display.h"

/*
 * When APP_DISPLAY_ENABLED == 0 (the default), display.h replaces
 * every Display_* call with a `((void)0)` macro that discards its
 * arguments. This translation unit then contains no symbols at all
 * — the linker emits nothing for it. Zero binary cost, zero runtime
 * cost, no snprintf invoked at call sites.
 *
 * When APP_DISPLAY_ENABLED == 1, the user is expected to wire one
 * of the supported panels and implement the four functions below.
 * The `#error` exists so an accidental enable without a driver
 * fails at compile time rather than silently producing a non-
 * functioning rig.
 */

#if APP_DISPLAY_ENABLED == 1

/*
 * Recommended panel choices and pinouts on the Nucleo-L432KC:
 *
 *   SSD1306 OLED (128×64, monochrome, I2C, ~$3-5)
 *     I2C1 on PB6 (SCL, AF4) and PB7 (SDA, AF4).
 *     1024 B framebuffer in SRAM; 3 lines × ~16 chars in 6×8 font
 *     or 3 lines × ~10 chars in 12×16 font. Default I2C address 0x3C.
 *
 *   ST7789 colour TFT (240×240, SPI, ~$8-12)
 *     SPI1 SCK on PA5, MOSI on PA7; CS/DC/RST on any of
 *     PA1, PA3, PA4, PA12, PB0, PB1, PB4, PB5.
 *     Full framebuffer (115 kB) won't fit in 64 kB SRAM — render
 *     one row at a time directly to SPI.
 *
 *   HD44780 char LCD with I2C backpack (16×2 or 20×4 chars, ~$5-8)
 *     I2C1 on PB6/PB7 (same as SSD1306). Backpack at 0x27.
 *     ~150 lines of driver.
 *
 * Display_Statusf is the printf-style sibling of Display_Status —
 * keep the snprintf inside the driver so the cost only lands when
 * the display is actually wired.
 */

#error "APP_DISPLAY_ENABLED is set but no driver is implemented in display.c"

void Display_Init(void)
{
    /* TODO: bring up I2C1 or SPI1, init the panel. */
}

void Display_Status(const char *l1, const char *l2, const char *l3)
{
    /* TODO: render the three lines. */
    (void)l1; (void)l2; (void)l3;
}

void Display_Statusf(const char *l1, const char *l2_fmt, ...)
{
    /* TODO: vsnprintf(l2_fmt, args) into a small buffer, then call
     * Display_Status(l1, formatted, ""). */
    (void)l1; (void)l2_fmt;
}

void Display_Clear(void)
{
    /* TODO: blank the panel. */
}

#endif /* APP_DISPLAY_ENABLED */
