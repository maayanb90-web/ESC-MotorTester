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

The DShot driver owns `DMA1_Channel4_IRQHandler` and
`TIM1_UP_TIM16_IRQHandler` directly in `Core/Src/dshot.c`. The `.ioc`
sets "Generate IRQ handler = false" for both vectors so CubeMX does not
emit duplicate handler stubs.

On STM32L432KC, TIM1_CH4's DMA request routes through **DMA1 channel 4**
(selected via `DMA1->CSELR` field `C4S = 0b0111`). The L432 has no
DMAMUX peripheral — that's an L4+/G4/U5 feature — so the driver's
`dshot_csel_route()` programs CSELR once at init. The end-of-frame
interrupt therefore fires on `DMA1_Channel4_IRQn`. CH3 lives on
DMA1 channel 6, and CH1/CH2 on channels 2/3 (RM0394 Table 41).

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

The RX path is **implemented end-to-end** and exercised by host tests
(see `tools/decode_test.c`). End-of-frame DMA TC transitions TIM1 from
PWM-out to input-capture in `Core/Src/dshot.c::dshot_rx_switch_to_input`;
after the RX timeout fires, `dshot_decode_rx` walks each channel's edge
buffer and delegates the pure GCR/CRC math to
`Core/Src/dshot_gcr.c::DShotGcr_Decode`. A frame that fails CRC (or
reports the ESC's motor-stopped sentinel) drops `s_telem[ch].valid` to
`false`, which the RPM-stats layer treats as a motor failure per
PRD §5.

When bringing the RX path up against real hardware, work in this order:

1. **TX-only smoke test**: scope PA8–PA11 during a 10 s run. All four
   lines should carry the same DShot300 frame (value 307) with no
   inter-channel skew. ESC should arm and motors should spin.

2. **Frame timing**: trigger the scope on the end of a TX frame.
   Confirm the line returns high (pull-up + open-drain ESC), then the
   ESC pulls low ~30 µs later and emits ~21 edges over ~110 µs.

3. **GCR decode sanity**: `make -C tools test` must report
   `34 passed, 0 failed`. The round-trip fixture builds a bidir frame
   for a synthesised eRPM period, runs it through `DShotGcr_Decode`,
   and asserts the recovered period and CRC. If you capture a real
   frame with a logic analyzer, you can drop its edge timestamps into
   `tools/decode_test.c` to verify the decoder against your actual ESC.

4. **Single-motor test**: connect one motor only. Expect the connected
   channel to report a sane RPM (~5000 RPM for the production motor at
   15 % throttle); the other three should fail "no telemetry" → 2 Hz
   blink + low buzz.

5. **Four-motor test** with a known-good batch: all four within
   ±2-3 % of the group mean → solid LD3 + high chime.

6. **Negative test**: pull one signal wire during a run → blink + buzz.

## Pass/fail beacon

When the test cycle completes, the firmware drives the motor windings as
piezo speakers for ~100 ms via DShot beacon commands. Configured in
`Core/Inc/app_config.h`:

- `APP_BEACON_PASS_CMD` — currently `5` (`DSHOT_CMD_BEACON5`, ~870 Hz).
- `APP_BEACON_FAIL_CMD` — currently `1` (`DSHOT_CMD_BEACON1`, ~250 Hz).
- `APP_BEACON_DURATION_MS` — currently `100`. Must be ≥ 6 frames at
  the SysTick rate (BLHeli's documented minimum-consecutive-frames
  requirement); the value here gives ~16× margin.

Played from `Core/Src/test_state.c::TestState_Tick` inside the
`TEST_RESULT` case. The motors **hum but do not spin** during playback
— don't confuse the audible humming with a runaway motor. Swap the
PASS/FAIL command numbers in `app_config.h` if the bench iteration
turns up a preference for different pitches.
