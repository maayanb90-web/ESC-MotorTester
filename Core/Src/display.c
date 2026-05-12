#include "display.h"
#include "app_config.h"

#if APP_DISPLAY_ENABLED == 0

/*
 * Stub implementation: all calls compile to no-ops.
 * The application calls Display_Status at every state transition;
 * with the addon disabled, those calls vanish.
 */

void Display_Init(void) { }

void Display_Status(const char *l1, const char *l2, const char *l3)
{
    (void)l1;
    (void)l2;
    (void)l3;
}

void Display_Clear(void) { }

#else

/*
 * Real driver lives here when APP_DISPLAY_ENABLED == 1.
 *
 * Recommended panel choices and pinouts on the Nucleo-L432KC:
 *
 *   SSD1306 OLED (128×64, monochrome, I2C, ~$3-5)
 *     I2C1 on PB6 (SCL, AF4) and PB7 (SDA, AF4).
 *     1024 B framebuffer in SRAM; 3 lines × ~16 chars in 6×8 font
 *     or 3 lines × ~10 chars in 12×16 font.
 *
 *   ST7789 colour TFT (240×240, SPI, ~$8-12)
 *     SPI1 SCK on PA5, MOSI on PA7; CS/DC/RST on any of
 *     PA1, PA3, PA4, PA12, PB0, PB1, PB4, PB5.
 *     Full framebuffer (115 kB) won't fit in 64 kB SRAM — render
 *     one row at a time directly to SPI.
 *
 *   HD44780 char LCD with I2C backpack (16×2 or 20×4 chars, ~$5-8)
 *     I2C1 on PB6/PB7 (same as SSD1306).
 *     ~150 lines of driver, addon at 0x27 by convention.
 *
 * Implement the three functions below using your chosen driver.
 * Display_Status receives generic three-line text; the driver maps
 * it onto whatever the panel supports (a 2-line LCD might drop l3,
 * a 240×240 TFT might centre the lines and add a coloured banner).
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

void Display_Clear(void)
{
    /* TODO: blank the panel. */
}

#endif
