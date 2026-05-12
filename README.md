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
2. Power the rig from a 6S LiPo or bench PSU. **The rig auto-runs a
   ~100 ms link-test (POST) at boot:** LD3 fast-blinks briefly, the
   firmware sends bidir `MOTOR_STOP` frames on all four channels and
   checks each ESC returns CRC-valid telemetry. Motors stay still.
   Pass → LD3 off, rig is ready. Fail → the fail-indicate cadence
   plays on the bad channel(s); fix the wiring, single-press to clear
   to Idle, power-cycle to re-POST.
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
   - **Failure** → the **failed motors keep beeping** in a 400 ms-on /
     200 ms-off cadence until cleared, each motor at its own pitch
     (motor 0 → ~250 Hz, motor 1 → ~280 Hz, motor 2 → ~330 Hz,
     motor 3 → ~430 Hz). Two motors failing produce two clearly
     different tones, so the tester can identify which channels are
     bad by ear alone. Passed motors stay silent.

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

## Button gestures (operator cheat-sheet)

| Gesture       | Active phase                                  | Effect |
|---------------|-----------------------------------------------|--------|
| Double-press  | Idle                                          | Start composite test (~13 s) |
| Triple-press  | Idle                                          | Start auto-calibration (~4-5 min, 20 cycles) |
| Single-press  | POST / Plateau A-C / Spin-down / mid-calibration | Abort to Idle (silent — motors stopped, no result emitted) |
| Single-press  | Result-fail (fail-indicate cadence)           | Clear result, return to Idle |
| Single-press  | Result-pass (LD3 solid) / Calibration-done    | Clear result, return to Idle |
| _Power cycle_ | _Any_                                         | Re-runs the boot POST link-test |

The button is detected by counting press-and-release events within a
`APP_DOUBLE_PRESS_WINDOW_MS` window (default 400 ms); any value ≥ 3 in
that window registers as a triple-press.

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

### Trace line reference

The host terminal sees four distinct line types. Standard CSV
parsers ignore the `#`-prefixed lines so the data rows can be
imported directly while the comment lines stay human-readable.

| First bytes | Meaning | When emitted | Example |
|---|---|---|---|
| `cycle_id,t_ms,...` | CSV header (column names)              | Once at boot              | `cycle_id,t_ms,overall_pass,aborted,m0_pass,...` |
| `<n>,<ms>,1,0,1,...` | Data row — completed cycle             | End of every cycle        | `1,17392,1,0,1,5012,8047,12089,76,1,4998,...`    |
| `<n>,<ms>,0,1,0,...` | Data row — aborted cycle, zeroed cols  | Operator single-presses mid-test | `2,18900,0,1,0,0,0,0,0,0,0,0,0,...`        |
| `# POST valid_frames = ...` | Power-on self-test result        | Once at boot, after the 100 ms link-test | `# POST valid_frames = c0:97 c1:96 c2:97 c3:97  overall_pass=1` |
| `# CALIBRATION n=... sigma_x10 = ...` | Per-phase σ summary    | End of triple-press calibration | `# CALIBRATION n=20 samples/phase=80  sigma_x10 = a:23 b:31 c:48 hl:142  recommend_x10 = a:70 b:93 c:144 hl:426` |
| `# RECOMMEND APP_PLATEAU_*_TOL_PCT_X10=...` | Copy-pastable tolerance block | Immediately after `# CALIBRATION` | `# RECOMMEND   APP_PLATEAU_A_TOL_PCT_X10=70  APP_PLATEAU_B_TOL_PCT_X10=93  APP_PLATEAU_C_TOL_PCT_X10=144  APP_HALF_LIFE_TOL_PCT_X10=426` |

`t_ms` is `HAL_GetTick()` at the moment the row was emitted; it
increments monotonically from boot and wraps at ~49 days.

## Module layout

```
Core/
├── Inc/
│   ├── dshot.h          bidirectional DShot300 driver API
│   ├── dshot_gcr.h      pure 5b/4b GCR decoder (host-testable)
│   ├── button.h         debounced single / double / triple-press detection
│   ├── led.h            LD3 state (off / solid / slow blink / fast blink)
│   ├── rpm_stats.h      per-window deviation pass/fail + half-life + composite
│   ├── test_state.h     Idle / POST / 3 plateaus / SpinDown / Result / Calibration FSM
│   ├── trace.h          USART2 CSV logger (one row per cycle + boot POST + calibration)
│   ├── calibration.h    auto-calibration sigma + recommended tolerances
│   └── app_config.h     all tunables in one place
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
| `APP_PLATEAU_A_TOL_PCT_X10`  | 70    | ±7.0 % — empirical mid-point; calibrate per batch |
| `APP_PLATEAU_B_TOL_PCT_X10`  | 80    | ±8.0 % |
| `APP_PLATEAU_C_TOL_PCT_X10`  | 100   | ±10.0 % — slip noise grows with throttle |
| `APP_HALF_LIFE_TOL_PCT_X10`  | 200   | ±20.0 % — bearing variance is wide |
| `APP_CALIBRATION_CYCLES`     | 20    | Cycles run on triple-press calibration (~4-5 min) |
| `APP_POST_DURATION_MS`       | 100   | Boot-time link-test duration |
| `APP_POST_MIN_VALID_FRAMES`  | 50    | Min CRC-valid frames per channel for POST pass |
| `APP_BEACON_DURATION_MS`     | 100   | Initial overall pass/fail tone |
| `APP_BEACON_PASS_CMD`        | 5     | `DSHOT_CMD_BEACON5` — high chime |
| `APP_BEACON_FAIL_CMD_PER_MOTOR` | `{1,2,3,4}` | `DSHOT_CMD_BEACON1..4` — distinct pitch per motor |
| `APP_FAIL_INDICATE_ON_MS`    | 400   | Per-failed-motor beep on-duration |
| `APP_FAIL_INDICATE_OFF_MS`   | 200   | Silence between beeps (re-triggers ESC beacon) |
| `APP_LED_BLINK_FAST_HZ`      | 5     | LD3 fast-blink rate (running / POST) |
| `APP_LED_BLINK_SLOW_HZ`      | 2     | LD3 slow-blink rate (failure indication) |
| `APP_DSHOT_ARR`              | 266   | TIM1 ARR during TX → 3.34 µs bit cell (DShot300) |
| `APP_DSHOT_T1H`              | 200   | CCR for a DShot "1" bit (~75 % of bit cell) |
| `APP_DSHOT_T0H`              | 100   | CCR for a DShot "0" bit (~37.5 % of bit cell) |
| `APP_DSHOT_RX_BIT_TICKS`     | 33    | RX bit cell @ 10 MHz tick (3.33 µs)   |
| `APP_DSHOT_RX_PSC`           | 7     | TIM1 prescaler during RX → 10 MHz     |
| `APP_DSHOT_RX_ARR`           | 1500  | TIM1 ARR during RX → 150 µs timeout   |

## Production-readiness status

The firmware has been audited end-to-end at commit `d11c3a5` for the
bug classes that bite embedded production code:

- **Concurrency / ISR safety.** Every shared mutable variable
  between SysTick and the DShot IRQs (`s_telem[]`, `s_pending`,
  `s_phase`, calibration accumulators) is either single-context or
  protected. The producer–consumer handshake on `s_telem[ch]` uses
  the **single-writer-flag** pattern: `dshot_decode_rx` commits the
  data fields, executes `__DMB()`, then sets `.valid`. The consumer
  (`DShot_ConsumeTelem`) reads `.valid` first (single-byte atomic
  LDRB) and the struct under `__disable_irq` PRIMASK protection.
  The contract holds regardless of relative NVIC priority.
- **NVIC priority invariant.** HAL_Init sets SysTick at priority 15
  (lowest); `DShot_Init` sets `TIM1_UP_TIM16_IRQn` and
  `DMA1_Channel4_IRQn` at priority 1. The DShot IRQs can preempt
  SysTick, never the other way around. This is the recommended
  configuration; the single-writer-flag pattern is the belt that
  keeps the consumer correct even if a future CubeMX regeneration
  flips the priorities.
- **Integer math.** Per-motor deviations are bounded by physical
  motor RPM (~ 16 kRPM max no-load). Group means are guarded against
  zero (`RpmStats_Evaluate` early-return). Calibration sigma uses
  uint64 sum-of-squares and an integer Newton-Raphson `isqrt`,
  saturated at 0xFFFF. No undefined behaviour reachable from real
  hardware inputs.
- **State-machine edge cases.** Abort at any phase transitions to
  Idle silently; mid-calibration abort drops the summary cleanly
  (raw cycle rows are still in the host log); equal-priority IRQs
  tail-chain rather than preempt (verified against `.ioc`'s
  `NVIC_PRIORITYGROUP_4`).
- **Host-side tests.** 80 / 80 assertions covering GCR decode round-
  trip, CRC corruption, motor-stopped sentinel, RPM math, RPM-stats
  staircase, half-life envelope, composite aggregation, integer
  square root, and the calibration math at zero / known / 3σ
  variance.

**Outstanding low-risk items** (acceptable for production, can be
hardened later if bench iteration surfaces them):

- The two USART busy-waits in `Core/Src/trace.c` (`trace_usart_init`
  on `TEACK` and `trace_write_byte` on `TXE`) have no timeout. If the
  on-board ST-LINK USART is broken the rig hangs at boot — which is
  the same failure surface as "rig is unusable" anyway, so the
  unbounded wait is not a hidden bug, just defensive polish to add
  if a future board variant warrants it.

**Remaining gate before deployment:** bench bring-up against a real
Nucleo-L432KC + 4-in-1 ESC + production motor batch, walking every
PRD §7 acceptance bullet plus a fresh triple-press calibration run.
The firmware has not yet executed on real hardware; all functional
validation to date is host-side and code-review based.
