# ESC-MotorTester — Motor QA Test Rig (Quad X)

Bench-top firmware that drives 4 production motors through a 4-in-1 ESC
(Bluejay, bidirectional DShot300) and flags any motor whose RPM deviates
from the 4-motor mean. Target MCU: **STM32 Nucleo-L432KC**.

See [`docs/motor_test_rig_prd.md`](docs/motor_test_rig_prd.md) for the full PRD.

## Pin map

| Function       | Pin   | Peripheral        |
|----------------|-------|-------------------|
| DShot CH1      | PA8   | TIM1_CH1 (AF1)    |
| DShot CH2      | PA9   | TIM1_CH2 (AF1)    |
| DShot CH3      | PA10  | TIM1_CH3 (AF1)    |
| DShot CH4      | PA11  | TIM1_CH4 (AF1)    |
| Button         | PA0   | GPIO input, internal pull-up, polled @ 1 kHz |
| Status LED LD3 | PB3   | GPIO output (active high) |
| SWD            | PA13/PA14 | reserved      |

All 4 DShot channels share TIM1, which guarantees sub-cycle synchronization
across the 4 ESC signal lines.

## Wiring

**4-in-1 ESC:** signal lines for motors 1-4 → PA8, PA9, PA10, PA11
respectively. GND of the ESC must be tied to the Nucleo's GND. Power
the ESC from the 6S LiPo / bench PSU; do **not** back-feed the Nucleo
from the ESC's BEC.

**Button:** a 2-pin momentary SPST tactile push-button — the kind where
the two legs are shorted together when the button is pressed — wired
between **PA0 and GND**. Either leg can go to either side; the part is
non-polar. No external pull-up or current-limiting resistor is needed:
the STM32 drives PA0 with its internal pull-up (~30-50 kΩ) and the
firmware reads "pressed" as PA0 LOW. The 1 kHz debounce in
`Core/Src/button.c` swallows ≤20 ms of contact bounce.

**Status LED:** LD3 is the on-board green LED on the Nucleo-L432KC
(PB3). No wiring needed.

## Build & flash

This project is laid out as an **STM32CubeIDE** workspace.

1. Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) 1.13+ and the **STM32CubeL4** package (auto-fetched on first build).
2. `File → Open Projects from File System…` and select this directory.
3. Right-click `motor_test_rig.ioc` → **Generate Code**. This populates `Drivers/` (HAL/LL/CMSIS), the linker script, the startup file, and `system_stm32l4xx.c`. The hand-written sources under `Core/Src` and `Core/Inc` are preserved (CubeMX keeps everything inside `USER CODE BEGIN`/`USER CODE END` markers).
4. Build (Hammer icon) and flash via the on-board ST-LINK (`Run` button). No manual post-generation edits are required — the SysTick wiring and IRQ ownership are already baked into `Core/Src/stm32l4xx_it.c` and the `.ioc`. See [`docs/INTEGRATION.md`](docs/INTEGRATION.md) for the design rationale and the DShot RX bring-up checklist.

Vendor drivers are intentionally **not committed**; they are regenerated
from `motor_test_rig.ioc`.

## Operation

1. Mount 4 motors on the rig (no propellers). Wire them to the 4-in-1 ESC.
2. Power the rig from a 6S LiPo or bench PSU. All motors stay off.
3. **Double-press** the button → motors spin at 15% throttle for 10 s.
4. Test ends automatically. LD3:
   - **Solid ON** = all 4 motors within ±5% of the group mean → batch passes.
   - **Blinking @ 2 Hz** = at least one motor failed (or no telemetry).
5. **Single-press** at any time aborts a running test, or clears the result.

## Module layout

```
Core/
├── Inc/
│   ├── dshot.h          bidirectional DShot300 driver API
│   ├── dshot_gcr.h      pure 5b/4b GCR decoder (host-testable)
│   ├── button.h         debounced single/double-press detection
│   ├── led.h            LD3 state (off / solid / slow blink / fast blink)
│   ├── rpm_stats.h      ±5% deviation pass/fail
│   ├── test_state.h     Idle / Running / Result state machine
│   └── app_config.h     PRD-level tunables in one place
├── Src/
│   ├── dshot.c          TIM1 + DMA TX, IC + DMA RX, calls into dshot_gcr
│   ├── dshot_gcr.c      pure decode logic — no STM32 deps
│   ├── button.c
│   ├── led.c
│   ├── rpm_stats.c
│   ├── test_state.c
│   └── app.c            wires modules together; called from main.c
tools/
├── decode_test.c        host-side round-trip + CRC tests for dshot_gcr
└── Makefile             `make -C tools test`
```

## Host-side tests

The GCR/CRC decode logic compiles on the host with no STM32 dependencies.
Run the round-trip + corruption tests with:

```sh
make -C tools test
```

Fixtures can be patched in from logic-analyzer captures by editing
`tools/decode_test.c`. The expected output is `18 passed, 0 failed`.

## Tunables

All PRD-level magic numbers live in [`Core/Inc/app_config.h`](Core/Inc/app_config.h):

| Symbol                       | Value | PRD §  |
|------------------------------|-------|--------|
| `APP_THROTTLE_PERCENT`       | 15    | §5     |
| `APP_TEST_DURATION_MS`       | 10000 | §5     |
| `APP_STARTUP_SKIP_MS`        | 2000  | §5     |
| `APP_TOLERANCE_PERCENT`      | 5     | §5     |
| `APP_MOTOR_POLE_COUNT`       | 14    | §5     |
| `APP_DOUBLE_PRESS_WINDOW_MS` | 400   | §5/§8  |
