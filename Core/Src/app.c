#include "app.h"

#include "button.h"
#include "dshot.h"
#include "led.h"
#include "test_state.h"

#include "stm32l4xx.h"

void App_Init(void)
{
    Led_Init();
    Button_Init();
    DShot_Init();
    TestState_Init();
}

void App_Tick(void)
{
    /* Order matters: button first so the same tick can act on a fresh
     * event; LED last so it reflects the current phase. */
    Button_Tick();
    TestState_Tick();
    Led_Tick();
}

void App_Loop(void)
{
    /* Sleep until the next interrupt (SysTick at minimum). All real work
     * is in App_Tick(). */
    __WFI();
}
