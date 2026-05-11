# Integration & bring-up notes

## First-time setup

1. Install STM32CubeIDE 1.13+ and the STM32CubeL4 firmware package
   (CubeIDE will fetch it on first build if missing).
2. `File → Open Projects from File System…`, select this directory.
3. Right-click `motor_test_rig.ioc` → **Generate Code**.

`Core/Inc/main.h`, `Core/Inc/stm32l4xx_it.h` and `Core/Src/stm32l4xx_it.c`
are **already shipped with the `App_Tick()` wiring inside the
`USER CODE` markers**. CubeMX will merge its generated content around our
markers and preserve the call into our app on this and every future
regeneration.

The DShot driver owns `DMA1_Channel5_IRQHandler` and
`TIM1_UP_TIM16_IRQHandler` directly in `Core/Src/dshot.c`. The `.ioc`
sets "Generate IRQ handler = false" for both vectors so CubeMX does not
emit duplicate handler stubs.

## On the order of init calls in main.c

CubeMX-generated `MX_GPIO_Init` / `MX_TIM1_Init` / `MX_DMA_Init` run
**before** `App_Init()` (which lives in `USER CODE BEGIN 2`). Our LL
drivers reconfigure GPIO, TIM1 and DMA1 from scratch — so any HAL config
from the `MX_*_Init` calls is harmlessly overwritten. **No manual edit
of `main.c` is required.**

If you regenerate from the `.ioc` and the `MX_*_Init` calls cause
unwanted side effects (e.g. driving outputs briefly during startup),
delete them inside `/* USER CODE BEGIN 2 */` blocks; CubeMX will not
re-insert them.

## Bidirectional DShot RX bring-up

The RX/GCR decode lives entirely in `Core/Src/dshot.c`. The decoder
function (`dshot_rx_decode`) returns RPM via `DShotTelem`; if it returns
`false` for a channel, `RpmStats` treats that motor as failed
("no telemetry" per PRD §5).

When bringing up RX on the bench, work in this order:

1. **TX-only smoke test**: scope PA8–PA11 during a 10 s run. All four
   lines should carry the same DShot300 frame (value 307) with no
   inter-channel skew. ESC should arm and motors should spin.

2. **Frame timing**: trigger the scope on the end of a TX frame.
   Confirm the line returns high (pull-up + open-drain ESC), then the
   ESC pulls low ~30 µs later and emits ~21 edges over ~110 µs.

3. **GCR decode** (in `dshot_rx_decode`): see references in that file's
   header comment. Verify on captured-edge fixtures via the host test
   under `tools/` before flashing.

4. **Single-motor test**: connect one motor only. Expect the connected
   channel to report a sane RPM (~5000 RPM for the production motor at
   15 % throttle); the other three should fail "no telemetry" → 2 Hz
   blink.

5. **Four-motor test** with a known-good batch: all four within
   ±2-3 % of the group mean → solid LD3.

6. **Negative test**: pull one signal wire during a run → blink result.
