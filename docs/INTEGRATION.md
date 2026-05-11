# Integration steps after `Generate Code`

This codebase ships with the **application** firmware (drivers, state machine,
button/LED, RPM stats) but **not** the vendor HAL/CMSIS tree, the linker
script, the startup file, or the auto-generated peripheral init code —
those are produced by STM32CubeMX from `motor_test_rig.ioc` on demand.

After running **STM32CubeIDE → Project → Generate Code** for the first time
(or any time you re-generate from the `.ioc`), perform the following steps.
They are all surgical edits inside CubeMX's `USER CODE BEGIN/END` markers,
so they survive future regenerations.

## 1. Wire `App_Tick()` into SysTick

Open `Core/Src/stm32l4xx_it.c`, find `SysTick_Handler`, and insert the call:

```c
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */
    App_Tick();
    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */
    /* USER CODE END SysTick_IRQn 1 */
}
```

Add the include in the file's `USER CODE BEGIN Includes` block:

```c
#include "app.h"
```

## 2. (Optional) Disable `MX_GPIO_Init` / `MX_TIM1_Init` / `MX_DMA_Init`

Our LL drivers configure GPIO, TIM1 and DMA themselves. The CubeMX-generated
HAL init functions (`MX_GPIO_Init`, `MX_TIM1_Init`, `MX_DMA_Init`,
`MX_TIM2_Init`) will overwrite our config when called from `main()`.

The simplest fix: in `Core/Src/main.c`, comment out the `MX_*_Init()` calls
inside the `Initialize all configured peripherals` block, **except** for
`HAL_Init()` and `SystemClock_Config()`.

Future cleanup: deselect "Initialise as HAL" in the .ioc for those
peripherals so CubeMX doesn't generate the init calls at all.

## 3. Confirm `Drivers/STM32L4xx_HAL_Driver/Src` contains the LL sources

Our drivers `#include` the following LL headers, which must compile:

- `stm32l4xx_ll_bus.h`
- `stm32l4xx_ll_dma.h`
- `stm32l4xx_ll_gpio.h`
- `stm32l4xx_ll_tim.h`

In `stm32l4xx_hal_conf.h` (auto-generated), make sure `USE_FULL_LL_DRIVER`
is defined. CubeMX defines it automatically once any peripheral is set
to "LL" mode; if you used HAL mode in the .ioc, add the define manually:

```c
#define USE_FULL_LL_DRIVER
```

## 4. Bidirectional DShot RX bring-up

`Core/Src/dshot.c::dshot_rx_decode()` is a stub that always reports
"no telemetry", which the application interprets as a failure per PRD §5.
This is intentional — until the GCR decoder is verified against a logic
analyzer, silently passing would be worse than visibly failing.

To finish RX, three things need to be implemented in `dshot.c`:

1. In `DMA1_Channel5_IRQHandler` end-of-frame: reconfigure CH1..CH4 as
   input capture (rising-then-falling), arm `DMA1_Channel2..5` as
   circular timestamp recorders into `s_rx_buf[ch]`, and set TIM1 ARR
   to a ~100 µs window so the update event acts as an RX timeout.

2. Implement the 5b/4b GCR table and clock-recovery logic in
   `dshot_rx_decode()`. References:
   - Betaflight `src/main/drivers/dshot.c::process_dshot_telemetry_data`
   - <https://brushlesswhoop.com/dshot-and-bidirectional-dshot/>

3. Convert the decoded eRPM-period to mechanical RPM:
   ```c
   uint32_t erpm = 60000000U / period_us;
   uint32_t rpm  = erpm * 2U / APP_MOTOR_POLE_COUNT;   // pole_count = 14
   ```

Until step 2 is done, the rig is still useful for verifying the TX path
end-to-end (frame format, sync, ESC arm/disarm) on a scope.
