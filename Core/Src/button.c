#include "button.h"

#include "app_config.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_gpio.h"

#define BTN_PORT          GPIOA
#define BTN_PIN           LL_GPIO_PIN_0

#define DEBOUNCE_MS       20U   /* ignore transitions shorter than this */

/*
 * State machine — tick @ 1 kHz.
 *
 *   IDLE              -> press detected, start debounce
 *   DEBOUNCE_PRESS    -> stable low for DEBOUNCE_MS, record press
 *   PRESSED           -> wait for release
 *   DEBOUNCE_RELEASE  -> stable high for DEBOUNCE_MS, emit a press tally
 *   WAIT_2ND          -> if a second press arrives within
 *                        APP_DOUBLE_PRESS_WINDOW_MS -> double; else single
 */
typedef enum {
    BTN_IDLE = 0,
    BTN_DEBOUNCE_PRESS,
    BTN_PRESSED,
    BTN_DEBOUNCE_RELEASE,
    BTN_WAIT_2ND,
} btn_state_t;

static volatile btn_state_t s_state           = BTN_IDLE;
static volatile uint16_t    s_state_timer_ms  = 0;
static volatile uint16_t    s_window_timer_ms = 0;
static volatile uint8_t     s_press_count     = 0;
static volatile ButtonEvent s_pending         = BUTTON_EVENT_NONE;

/*
 * Hardware: a 2-pin momentary SPST tactile push-button is wired between
 * PA0 and GND. STM32's internal pull-up (~30-50 kΩ) holds PA0 HIGH when
 * the button is open; pressing shorts the two legs together, pulling
 * PA0 LOW. No external pull-up or current-limiting resistor required.
 * The two legs of a 2-pin tactile switch are interchangeable — either
 * one can go to PA0 and either one can go to GND.
 */

static bool button_is_down(void)
{
    /* Active-low: pressed when the input reads 0 (legs shorted to GND). */
    return LL_GPIO_IsInputPinSet(BTN_PORT, BTN_PIN) == 0;
}

void Button_Init(void)
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef io = {
        .Pin   = BTN_PIN,
        .Mode  = LL_GPIO_MODE_INPUT,
        .Pull  = LL_GPIO_PULL_UP,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
    };
    LL_GPIO_Init(BTN_PORT, &io);
}

void Button_Tick(void)
{
    const bool down = button_is_down();

    switch (s_state) {
    case BTN_IDLE:
        if (down) {
            s_state          = BTN_DEBOUNCE_PRESS;
            s_state_timer_ms = 0;
        }
        break;

    case BTN_DEBOUNCE_PRESS:
        if (!down) {
            s_state = BTN_IDLE;          /* bounce */
        } else if (++s_state_timer_ms >= DEBOUNCE_MS) {
            s_state = BTN_PRESSED;
        }
        break;

    case BTN_PRESSED:
        if (!down) {
            s_state          = BTN_DEBOUNCE_RELEASE;
            s_state_timer_ms = 0;
        }
        break;

    case BTN_DEBOUNCE_RELEASE:
        if (down) {
            s_state = BTN_PRESSED;        /* bounce on release */
        } else if (++s_state_timer_ms >= DEBOUNCE_MS) {
            s_press_count++;
            s_window_timer_ms = 0;
            s_state           = BTN_WAIT_2ND;
        }
        break;

    case BTN_WAIT_2ND:
        if (down) {
            s_state          = BTN_DEBOUNCE_PRESS;
            s_state_timer_ms = 0;
        } else if (++s_window_timer_ms >= APP_DOUBLE_PRESS_WINDOW_MS) {
            s_pending     = (s_press_count >= 2) ? BUTTON_EVENT_DOUBLE
                                                 : BUTTON_EVENT_SINGLE;
            s_press_count = 0;
            s_state       = BTN_IDLE;
        }
        break;
    }
}

ButtonEvent Button_Read(void)
{
    ButtonEvent e = s_pending;
    s_pending     = BUTTON_EVENT_NONE;
    return e;
}
