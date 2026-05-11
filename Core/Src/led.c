#include "led.h"

#include "app_config.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_gpio.h"

#define LED_PORT     GPIOB
#define LED_PIN      LL_GPIO_PIN_3

static volatile LedMode  s_mode  = LED_OFF;
static volatile uint16_t s_phase = 0;

static void led_write(int on)
{
    if (on) {
        LL_GPIO_SetOutputPin(LED_PORT, LED_PIN);
    } else {
        LL_GPIO_ResetOutputPin(LED_PORT, LED_PIN);
    }
}

void Led_Init(void)
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    LL_GPIO_InitTypeDef io = {
        .Pin        = LED_PIN,
        .Mode       = LL_GPIO_MODE_OUTPUT,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull       = LL_GPIO_PULL_NO,
        .Speed      = LL_GPIO_SPEED_FREQ_LOW,
    };
    LL_GPIO_Init(LED_PORT, &io);

    led_write(0);
}

void Led_SetMode(LedMode mode)
{
    if (s_mode != mode) {
        s_mode  = mode;
        s_phase = 0;
        led_write(mode == LED_SOLID_ON);
    }
}

void Led_Tick(void)
{
    /* Period in ms for a full on/off cycle. */
    const uint16_t slow_half = (uint16_t)(500U / APP_LED_BLINK_SLOW_HZ);
    const uint16_t fast_half = (uint16_t)(500U / APP_LED_BLINK_FAST_HZ);

    s_phase++;

    switch (s_mode) {
    case LED_OFF:
        led_write(0);
        break;
    case LED_SOLID_ON:
        led_write(1);
        break;
    case LED_BLINK_SLOW:
        led_write((s_phase / slow_half) & 1U);
        break;
    case LED_BLINK_FAST:
        led_write((s_phase / fast_half) & 1U);
        break;
    }
}
