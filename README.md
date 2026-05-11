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
| Log TX         | PA2   | USART2_TX (AF7) → ST-LINK VCP @ 115200 8N1 |
| Log RX         | PA15  | USART2_RX (AF3) — reserved for future cmd shell |
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
3. **Double-press** the button → composite test runs for ~13 s:
   - Plateau A — 3 s at 15 % throttle (low-throttle commutation check)
   - Plateau B — 3 s at 25 % throttle (mid-range commutation, magnet strength)
   - Plateau C — 3 s at 40 % throttle (upper-range commutation, winding faults)
   - Spin-down — 4 s coast-down, measuring each motor's RPM half-life
     (bearing drag / rotor imbalance)
4. Test ends automatically. LD3 indicates the overall result:
   - **Solid ON** = every motor passed all four checks within tolerance.
   - **Blinking @ 2 Hz** = at least one motor failed at least one check.

   At the same instant, the motors emit audible cues:
   - **All-pass** → ~100 ms high chime (~870 Hz) on every motor, then silence.
   - **Failure** → after a short low buzz, the **failed motors keep beeping**
     in a 400 ms-on / 200 ms-off cadence until cleared. Passed motors
     stay silent.

   The motors hum (and visibly vibrate) during a tone but **do not spin**.
   The tester walks up to the rig, hears and sees exactly which motors
   are buzzing, pulls those motors from the batch, and clears the result.
5. **Single-press** at any time aborts a running test, or clears the
   fail-indicate cadence and returns to Idle.
6. **Triple-press** from Idle starts an **auto-calibration** run: the
   rig executes `APP_CALIBRATION_CYCLES` (default 20) composite cycles
   back-to-back on a known-good batch, then emits a recommended-
   tolerance summary line over the CSV log. Each cycle still produces
   its normal CSV row, so the raw data is captured too. After the
   summary, LD3 goes solid; single-press to clear. Operator pastes
   the four recommended `APP_*_TOL_PCT_X10` values into `app_config.h`
   and rebuilds — that's the §8 "process a known-good batch and set
   max(default, 3 sigma)" workflow, automated.

## Live logging (CSV over USB)

Every test cycle emits one CSV row over the **same micro-USB cable you
already use to flash the Nucleo**. The Nucleo's on-board ST-LINK
exposes the MCU's USART2 (PA2 / PA15) to the host PC as a virtual COM
port, so plugging the rig in produces `/dev/ttyACM*` on Linux/Mac or
`COMx` on Windows at **115200 baud, 8N1**.

```
$ screen /dev/ttyACM0 115200             # or:  minicom -D /dev/ttyACM0 -b 115200
cycle_id,t_ms,overall_pass,aborted,m0_pass,m0_mean_a,m0_mean_b,m0_mean_c,m0_half_life_ms,...
1,17392,1,0,1,5012,8047,12089,76,1,4998,8021,12104,78,1,5022,8055,12076,72,1,5005,8038,12092,74
2,31118,0,0,1,5008,8033,12102,75,1,5015,8045,12087,77,0,4612,7384,11215,71,1,5020,8049,12088,73
```

`m{0..3}_pass` is the per-motor AND across all four validators. `mean_a/b/c`
are the mean RPMs over the steady-state portion of each plateau, and
`half_life_ms` is the coast-down ticks from motor-stop to half-RPM.
Aborted cycles produce a row with `aborted=1` and zeroed per-motor
columns.

Pipe to a file for archival (`screen | tee log.csv` or
`stty -F /dev/ttyACM0 115200 raw && cat /dev/ttyACM0 >> log.csv`) and
open in Excel — the header line makes the columns self-describing.

## Module layout

```
Core/
├── Inc/
│   ├── dshot.h          bidirectional DShot300 driver API
│   ├── dshot_gcr.h      pure 5b/4b GCR decoder (host-testable)
│   ├── button.h         debounced single/double-press detection
│   ├── led.h            LD3 state (off / solid / slow blink / fast blink)
│   ├── rpm_stats.h      per-window deviation pass/fail + half-life + composite
│   ├── test_state.h     Idle / 3 plateaus / SpinDown / Result FSM
│   ├── trace.h          USART2 CSV logger (one row per cycle)
│   ├── calibration.h    auto-calibration sigma + recommended tolerances
│   └── app_config.h     PRD-level tunables in one place
├── Src/
│   ├── dshot.c          TIM1 + DMA TX, IC + DMA RX, calls into dshot_gcr
│   ├── dshot_gcr.c      pure decode logic — no STM32 deps
│   ├── button.c
│   ├── led.c
│   ├── rpm_stats.c
│   ├── test_state.c
│   ├── trace.c          USART2 LL init + CSV row formatter
│   ├── calibration.c    Welford sigma + isqrt + recommended tolerances
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
`tools/decode_test.c`. The expected output is `80 passed, 0 failed`
(GCR round-trip, CRC corruption, motor-stopped sentinel, eRPM math
edge cases, bidir-frame CRC round-trip, RPM-stats all-zero,
RPM-stats known-good, staircase pass / one-motor-off, half-life
within tolerance / one-sticky, composite aggregation, integer
square root, calibration zero/known/3-sigma scenarios).

## Tunables

All magic numbers live in [`Core/Inc/app_config.h`](Core/Inc/app_config.h):

| Symbol                       | Value | Notes                                 |
|------------------------------|-------|---------------------------------------|
| `APP_MOTOR_POLE_COUNT`       | 14    | PRD §5                                |
| `APP_DOUBLE_PRESS_WINDOW_MS` | 400   | PRD §5 / §8                           |
| `APP_PLATEAU_A_PCT`          | 15    | v2 composite — low-throttle plateau (% of full DShot range) |
| `APP_PLATEAU_B_PCT`          | 25    | v2 composite — mid plateau |
| `APP_PLATEAU_C_PCT`          | 40    | v2 composite — upper plateau (kept ≤ 40 % for no-prop safety) |
| `APP_PLATEAU_DURATION_MS`    | 3000  | Per plateau, including the startup-skip window |
| `APP_PLATEAU_SKIP_MS`        | 500   | Transient skipped at the start of each plateau |
| `APP_SPIN_DOWN_DURATION_MS`  | 4000  | Coast-down telemetry window for the half-life check |
| `APP_PLATEAU_A_TOL_PCT_X10`  | 70    | ±7.0 % — see "On the tolerance choice" in plan doc |
| `APP_PLATEAU_B_TOL_PCT_X10`  | 80    | ±8.0 % |
| `APP_PLATEAU_C_TOL_PCT_X10`  | 100   | ±10.0 % — slip noise grows with throttle |
| `APP_HALF_LIFE_TOL_PCT_X10`  | 200   | ±20.0 % — bearing variance is wide |
| `APP_CALIBRATION_CYCLES`     | 20    | Cycles run on triple-press calibration (~4-5 min) |
| `APP_BEACON_DURATION_MS`     | 100   | Initial overall pass/fail tone |
| `APP_BEACON_PASS_CMD`        | 5     | `DSHOT_CMD_BEACON5` — high chime |
| `APP_BEACON_FAIL_CMD`        | 1     | `DSHOT_CMD_BEACON1` — low buzz |
| `APP_FAIL_INDICATE_ON_MS`    | 400   | Per-failed-motor beep on-duration |
| `APP_FAIL_INDICATE_OFF_MS`   | 200   | Silence between beeps (re-triggers ESC beacon) |
| `APP_DSHOT_RX_BIT_TICKS`     | 33    | RX bit cell @ 10 MHz tick (3.33 µs)   |
| `APP_DSHOT_RX_PSC`           | 7     | TIM1 prescaler during RX → 10 MHz     |
| `APP_DSHOT_RX_ARR`           | 1500  | TIM1 ARR during RX → 150 µs timeout   |
