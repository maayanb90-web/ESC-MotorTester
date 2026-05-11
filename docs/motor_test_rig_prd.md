# PRD — Motor QA Test Rig (Quad X)

**Owner:** Mechanical & Electrical Tooling
**Status:** Draft v0.3
**Type:** Internal production tooling

---

## 1. Background & Problem

During drone production we have observed that some motors fail to spin consistently when commanded with a specific throttle value. The defect appears to be batch-related — within a given batch, one or more motors behave differently from the rest under an identical command.

Today we have no systematic way to detect these motors before they are integrated into a complete drone. Issues are typically caught only at final flight test, which is costly and slow.

## 2. Objective

Build a bench-top test rig that drives 4 motors simultaneously with an identical throttle command and uses **bidirectional DShot300 telemetry** to read back the actual RPM of each motor. Motors whose RPM deviates from the group are flagged as suspect and removed from the batch before assembly.

## 3. Scope

**In scope (v1):**
- Mechanical rig based on the production Quad X frame
- Integration with the production 4-in-1 ESC running Bluejay firmware, bidirectional DShot300 enabled
- Nucleo-L432KC controller + dedicated firmware
- 4 synchronized DShot300 output channels with RPM telemetry read-back
- Single momentary push button user interface
- Overall pass/fail indication on the Nucleo onboard LED

**Out of scope (v1):**
- Data logging / batch traceability
- Per-motor result display (only overall pass/fail in v1)
- Runtime throttle adjustment from a UI
- Multiple rigs networked together
- Safety enclosure (motors run propeller-less by design)

## 4. Hardware Requirements

| Item | Spec |
|---|---|
| Frame | Production Quad X frame, rigidly fixtured to the bench |
| Motors | 4 × motors under test (DUT), production part, **no propellers**, 14-pole |
| ESC | Production 4-in-1 ESC, Bluejay firmware, bidirectional DShot300 enabled |
| MCU | STM32 Nucleo-L432KC |
| User input | 1 × momentary push button, normally open, debounced |
| Status indication | Onboard user LED (LD3, PB3) for overall pass/fail; "armed/running" indication via blink pattern or shared with LD3 (TBD) |
| Power | 6S LiPo **or** equivalent bench PSU |
| Wiring | 4 ESC signal lines connected to DShot-capable timer pins on the Nucleo (bidirectional, single wire per channel) |

## 5. Firmware Requirements

### Signal protocol
- 4 synchronized **bidirectional DShot300** channels.
- Throttle command: **15%** of full DShot range (≈ DShot value **307**).
- All 4 channels must start, stop, and update synchronously (no inter-channel skew visible on a logic analyzer).
- RPM telemetry decoded from each ESC every cycle. eRPM converted to mechanical RPM using **pole count = 14** (RPM = eRPM × 2 / pole count = eRPM / 7).

### State machine

| State | Event | Action | Next state |
|---|---|---|---|
| Idle | Double press | Arm motors, ramp to 15% throttle, start 10 s test | Running |
| Running | 10 s elapsed | Stop motors, evaluate pass/fail, set result indication | Result |
| Running | Single press | Stop motors immediately, no result evaluation | Idle |
| Result | Single press | Clear LED, return to idle | Idle |

### Test cycle
- Total motor run time: **10 seconds**
- Skip the first ~2 s (startup transient); sample RPM telemetry across the remaining ~8 s
- Compute mean RPM per motor across the sample window

### Pass/fail logic
- For each motor, compute deviation from the 4-motor mean RPM
- Motor **passes** if its deviation is within **±5%** of the mean
- Motor **fails** if it exceeds ±5%, or if no telemetry is received from that channel (motor not spinning / wiring fault)
- **Overall result** displayed on the onboard LED (LD3, green):
  - **Solid ON** = all 4 motors pass
  - **Blinking** (~2 Hz) = at least one motor failed
- Operator clears the result with a single press to return to idle

### Safety
- Motors off at power-on; no auto-arm.
- Single press at any time must stop all motors immediately.
- Software debounce on the button input. Double-press window ~400 ms (TBD).

## 6. Operator Flow

1. Mount 4 motors under test onto the rig (no propellers); connect them to the 4-in-1 ESC.
2. Power the rig from the 6S LiPo or PSU. All motors remain off.
3. Double-press the button → all 4 motors spin at 15% throttle for 10 seconds.
4. Rig stops motors automatically. LD3 indicates the result: solid = all pass, blinking = at least one motor failed.
5. If failed, operator re-runs the test or swaps motors one at a time to isolate the bad unit.
6. Single-press to clear the result and return to idle. Repeat for the next set of motors.

## 7. Acceptance Criteria

- All 4 DShot300 channels output identical throttle commands, verifiable with a logic analyzer.
- RPM telemetry successfully decoded from all 4 ESC channels during normal operation.
- Motors run for exactly 10 s once started; auto-stop reliably.
- Pass/fail result displayed within 1 s of test completion via LD3.
- Single-press abort stops all motors immediately, at any time.
- Motors off at power-on and after every test cycle.
- Rig is operable by a production technician without firmware knowledge.

## 8. Open Questions / TBDs

- **Armed/running indication on a single LED:** LD3 is reused for both run-state and result. Proposed scheme — fast blink (~5 Hz) while running, solid/slow-blink after the cycle for pass/fail. To be finalized in firmware.
- **Double-press detection window:** suggested ~400 ms between presses. Final value to be tuned during firmware bring-up.
- **Tolerance baseline:** ±5% from the 4-motor mean is the v1 starting point. After processing a known-good batch we may revise this from real data.
- **Isolating the failing motor:** v1 only reports overall pass/fail. If batch failure rate is high enough that swap-and-retest becomes painful, consider adding 4 external LEDs (v1.5).
