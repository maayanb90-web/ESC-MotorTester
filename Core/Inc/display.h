#ifndef DISPLAY_H
#define DISPLAY_H

/*
 * Optional bench-display abstraction. Default build has
 * `APP_DISPLAY_ENABLED = 0` (in app_config.h) and every call here
 * compiles to a no-op — zero runtime cost, zero binary footprint.
 *
 * When the operator wires a real panel (SSD1306 OLED on I2C1 PB6/PB7,
 * an ST7789 colour TFT on SPI1, or an HD44780 char LCD on an I2C
 * backpack), set APP_DISPLAY_ENABLED = 1 and fill in the driver
 * bodies in Core/Src/display.c. The application code (test_state.c)
 * already calls Display_Status at every state transition, so the
 * display will start rendering as soon as the driver lands — no
 * other source changes needed.
 *
 * The abstraction is intentionally generic three-line text — works
 * for any of the candidate panels without coupling the application
 * to a specific resolution or graphics primitive.
 */

void Display_Init(void);

/* Show a three-line status. Any NULL is treated as empty. The
 * driver implementation handles wrapping / truncation to the
 * panel's pixel or character budget. */
void Display_Status(const char *l1, const char *l2, const char *l3);

/* Blank the panel (rig power-off intent). */
void Display_Clear(void);

#endif /* DISPLAY_H */
