# C-Band Laser Controller — Testing Guide

**Project:** PTB PIC Assembly — Skylark CubeSat Mission  
**Laser:** QPhotonics QDFBLD-1550-2SM (1550 nm DFB, fiber-coupled)  
**Controller:** Maiman Electronics SF8025-NM  
**Document date:** April 2026

---

## Table of Contents

1. [What the hardware is and how it works](#1-what-the-hardware-is-and-how-it-works)
2. [How the software talks to the hardware](#2-how-the-software-talks-to-the-hardware)
3. [Test files — overview and order](#3-test-files--overview-and-order)
4. [Stage 1 — Basic controller tests (no laser, no TEC)](#4-stage-1--basic-controller-tests-no-laser-no-tec)
5. [Stage 2 — TEC, thermistor, and PID tests (no laser)](#5-stage-2--tec-thermistor-and-pid-tests-no-laser)
6. [The interlock — what it is and the digital workaround](#6-the-interlock--what-it-is-and-the-digital-workaround)
7. [Known bugs found and fixed during development](#7-known-bugs-found-and-fixed-during-development)
8. [Physical wiring reference](#8-physical-wiring-reference)
9. [How to run the tests](#9-how-to-run-the-tests)
10. [Reading test output and what failures mean](#10-reading-test-output-and-what-failures-mean)
11. [Glossary](#11-glossary)

---

## 1. What the hardware is and how it works

### 1.1 The laser diode (QDFBLD-1550-2SM)

A laser diode is a tiny semiconductor chip that emits light when electrical current flows through it. This particular one:

- Emits light at **1550 nm** wavelength — this is the C-band, used in fibre-optic telecommunications and in photonic integrated circuits like the one in this project.
- Is a **DFB (Distributed Feedback)** laser, meaning it emits at a single, very precise wavelength rather than a broad spectrum.
- Is mounted on a small circuit board (the *laser PCB*) with a **fibre pigtail** already attached so its light output goes directly into a single-mode fibre cable.
- Has a maximum forward current of **38 mA**. Going above this can permanently destroy it.
- Has a typical threshold current of **~11 mA** — below this current the device emits no coherent laser light, above it lasing begins.

The laser chip itself sits inside a metal package on the laser PCB. It has several pins:

| Laser pin | Function |
|---|---|
| 1, 2 | Thermistor (temperature sensor) |
| 3 | LD cathode (−) |
| 4 | Monitor photodiode cathode |
| 5 | Monitor photodiode anode |
| 6, 8 | LD anode (+) |
| 7 | RF cathode (not used in CW mode) |

The **monitor photodiode (internal PD)** is a tiny photodetector built into the same package as the laser. It picks up a small fraction of the laser's own light output. Reading the current from this photodiode (pins 4 and 5, measured with a precision multimeter) gives you a relative measure of optical output power — useful as a sanity check even without an external power meter.

### 1.2 The laser PCB

The laser PCB is a small custom circuit board that:

- Holds the laser diode package
- Has a **Peltier cooler (TEC element)** mounted beneath the laser chip, in contact with it through thermal paste
- Has an **NTC thermistor** co-located next to the laser, also in thermal contact

The laser PCB connects to the Maiman controller via a ribbon cable to the **output socket**.

### 1.3 The Peltier cooler (TEC element)

A Peltier cooler (also called a thermoelectric cooler, or TEC) is a solid-state heat pump. It has no moving parts. When electrical current flows through it:

- One face gets **cold** (absorbs heat from the laser chip)
- The other face gets **hot** (dumps that heat into the board/air)
- Reversing the current direction reverses which face is hot and which is cold

This lets the controller actively **heat or cool** the laser chip. Why is this important for a laser? Because a laser diode's wavelength and output power both shift with temperature. For a DFB laser used in a precision photonic circuit, the temperature must be held stable to within ±0.1 °C. Without active temperature control, even normal room temperature fluctuations (a person walking past, the air conditioning cycling) would cause the wavelength to drift.

The Peltier on this laser PCB:
- Connects to the controller via **output socket pins 5, 6 (TEC Out+) and 7, 8 (TEC Out−)**
- Can receive up to ±4 A at ±4 V from the controller
- The polarity of current determines heating vs. cooling

### 1.4 The NTC thermistor

An NTC (Negative Temperature Coefficient) thermistor is a resistor whose electrical resistance **decreases as temperature rises**. At room temperature (25 °C) this particular thermistor has a resistance of exactly **10 kΩ**. At a lower temperature it is higher; at a higher temperature it is lower.

The controller measures this resistance by applying a known voltage and measuring the resulting current. It then converts the resistance to temperature using the Steinhart-Hart equation (a standard formula for NTC thermistors). This gives the controller a live temperature reading of the laser chip with 0.01 °C resolution.

The thermistor connects to the controller via **output socket pins 11 and 12**.

> **Why the thermistor matters for safety:** The controller firmware will refuse to enable the laser driver if the thermistor is disconnected or reading an implausible value. This prevents running the laser without temperature control, which could overheat and destroy the laser chip.

### 1.5 The PID controller

PID stands for **Proportional–Integral–Derivative**. It is a standard control algorithm used throughout engineering to make a measured quantity (here: temperature) track a desired target (the setpoint).

Think of it like this: you are driving a car and want to maintain 100 km/h.

- **Proportional (P):** You press the accelerator harder the more you are below 100 km/h. The bigger the gap, the bigger the correction.
- **Integral (I):** If you have been slightly below 100 km/h for a long time, you gradually press harder to remove the persistent error. This eliminates steady-state offset.
- **Derivative (D):** If you are approaching 100 km/h very rapidly, you ease off the accelerator to avoid overshooting. This damps oscillation.

In the SF8025-NM, the PID controller:
- Reads the thermistor temperature every few milliseconds
- Computes how far it is from the setpoint (the error)
- Calculates how much current to send through the Peltier based on P, I, and D gains
- The Maiman controller ships with factory-tuned PID gains that it calls "self-adjusted" — you do not normally need to change them

The PID gains are stored in registers `0A21h` (P), `0A22h` (I), `0A23h` (D) and can be read digitally.

### 1.6 The Maiman Electronics SF8025-NM controller

This is the main electronics board. It provides:

- A **laser diode driver**: a precision constant-current source that can supply 0–250 mA to the laser (the SF8025 variant; the "25" means 250 mA maximum)
- A **TEC controller**: the PID loop and power electronics to drive the Peltier cooler
- **Digital communication** via USB/UART at 115200 baud — this is how the Python code talks to it
- **Protection circuits**: interlock, overcurrent crowbar, NTC interlock, overheat shutdown

The board has two important physical connectors for signals:

| Connector | Pins | Used for |
|---|---|---|
| Output socket (14-pin) | See wiring table below | Laser, TEC, thermistor |
| Analogue control connector (20-pin) | See wiring table below | Enable signals, interlock, monitoring |

It also has an **8-pin digital connector** (USB/UART) which is how the Python driver communicates via COM3.

---

## 2. How the software talks to the hardware

### 2.1 The serial protocol

The SF8025-NM communicates over a serial (UART) connection at 115200 baud. All commands are plain ASCII text. There are two command types:

- **J-type (Get):** Ask the controller for a value. Example: `J0A15\r` asks for the measured TEC temperature.
- **P-type (Set):** Write a value to the controller. Example: `P0A10 0BB8\r` sets the TEC temperature setpoint to 30.00 °C (0BB8 hex = 3000, divided by 100 = 30.00 °C).

The controller responds with a **K-type** response containing the parameter number and the value as a 4-character hex string.

### 2.2 The Python driver (`controller.py`)

The `SF8xxx` class in `controller.py` wraps this serial protocol into Python method calls. You never need to deal with hex strings directly.

Key methods and what they do:

| Method | Direction | What it does |
|---|---|---|
| `get_tec_temperature()` | Read | Returns thermistor temperature in °C as a float |
| `get_tec_value()` | Read | Returns the currently stored TEC setpoint in °C |
| `set_tec_temperature(t)` | Write | Sends a new temperature setpoint to the controller |
| `get_tec_current()` | Read | Returns the current flowing through the Peltier in Amperes |
| `get_tec_current_limit()` | Read | Returns the maximum allowed TEC current |
| `tec_state()` | Read | Returns a 3-tuple: (tec_on, temp_mode, enable_mode) |
| `set_tec_int()` | Write | Tells the controller to use digital (UART) control for TEC |
| `set_tec_on()` | Write | Starts the TEC / PID loop |
| `set_tec_off()` | Write | Stops the TEC |
| `deny_interlock()` | Write | Tells the controller to ignore the analogue interlock pin |
| `allow_interlock()` | Write | Restores hardware interlock checking |
| `get_pid_p/i/d()` | Read | Returns PID gain values |
| `get_driver_current()` | Read | Returns laser drive current in mA |
| `get_driver_current_max()` | Read | Returns the configured maximum drive current |
| `set_driver_current(i)` | Write | Sets the laser drive current setpoint |
| `set_driver_on()` | Write | Enables the laser driver (starts current flowing) |
| `set_driver_off()` | Write | Disables the laser driver |
| `driver_state()` | Read | Returns a 6-tuple of driver status flags |
| `lock_state()` | Read | Returns a 6-tuple of fault/protection flags |

### 2.3 Internal values and scaling

The controller stores all values as integers internally. The Python driver divides by a scaling factor when reading:

| Quantity | Internal unit | Scaling | Example |
|---|---|---|---|
| TEC temperature | 0.01 °C | ÷ 100 | 2500 → 25.00 °C |
| TEC current | 0.1 A | ÷ 10 | 15 → 1.5 A |
| Driver current | 0.1 mA | ÷ 10 | 380 → 38.0 mA |

---

## 3. Test files — overview and order

There are two test files, run in this order:

| Order | File | What it tests | Laser connected? | TEC running? |
|---|---|---|---|---|
| 1 | `test_hardware.py` | Controller communication, register reads, setpoint round-trips, TEC basic enable | No | Briefly at end |
| 2 | `test_tec_pid.py` | Thermistor sanity, TEC enable/disable, PID response, fault flags, setpoint sweep | No | Yes |

Stage 3 (not yet written) will test with the laser connected and the driver enabled.

---

## 4. Stage 1 — Basic controller tests (no laser, no TEC)

**File:** `test_hardware.py`

### What is being tested and why

This file confirms the Python driver can communicate with the controller and that all readable registers return sensible values. No laser is connected and the TEC is not running for most of this file.

### Test-by-test explanation

#### Connection tests

```
test_connected
test_serial_number_readable
```

These confirm the serial port opened, the controller responded to a command, and returned its unique serial number as a non-zero integer. If these fail, the USB cable is unplugged, the wrong COM port is configured, or the controller is not powered.

#### Driver state tests

```
test_driver_state_readable
test_driver_current_readable
test_driver_max_current_readable
```

These read the driver's status register and two current registers. They check that the responses are valid numbers of the right type. They do not change any settings and do not activate the driver.

The **driver state** is a bitmask — a single number where each individual bit (binary digit) represents a different on/off flag:

- Bit 0: Device powered (always 1)
- Bit 1: Driver started (1 = laser driver on)
- Bit 2: Current set mode (0 = external/analogue, 1 = internal/digital)
- Bit 4: Enable mode (0 = external, 1 = internal)
- Bit 6: External NTC interlock (0 = allowed, 1 = denied)
- Bit 7: Interlock (0 = allowed, 1 = denied)

#### TEC basic tests

```
test_tec_temperature_readable
test_tec_current_readable
test_tec_current_limit_readable
```

Reads the thermistor temperature, current TEC current, and the configured maximum TEC current. The temperature test checks the value is between −20 and +80 °C (a wide sanity range; the narrower 15–35 °C lab range check is in the TEC test file). These do not enable the TEC.

#### Lock/safety state test

```
test_lock_state_readable
```

Reads the protection/fault register. This register reports whether any of the following fault conditions exist:

| Flag | Meaning |
|---|---|
| Interlock | Analogue pin 15 is open (not grounded) |
| LD overcurrent | Laser driver exceeded the current protection threshold |
| LD overheat | Laser chip or controller exceeded temperature limit |
| NTC | External NTC thermistor fault |
| TEC error | TEC overcurrent, short-circuit, or board overheat |
| TEC self-heat | Controller board is overheating |

This test only checks the register is readable — it does not check the flag values (some flags will be set at this stage, which is expected).

#### Setpoint round-trip tests

```
test_set_driver_current
test_set_tec_temperature
```

These write a value to the controller and immediately read it back to confirm the value was stored correctly.

- `test_set_driver_current`: Writes 5.0 mA as the driver current setpoint and reads back the stored value. Tolerance is ±0.5 mA. **This does not turn the driver on** — it only sets the target current for when it is eventually enabled.
- `test_set_tec_temperature`: Writes 25.0 °C as the TEC setpoint and reads back the stored value. Tolerance is ±0.2 °C.

#### Driver enable safety test

```
test_driver_enable_with_interlock_open
```

This test verifies that the laser driver **cannot** be turned on when the analogue interlock pin (pin 15) is open and no `deny_interlock()` has been issued. It:

1. Sets a safe current (5 mA)
2. Attempts `set_driver_on()`
3. Reads the driver state
4. Asserts the driver bit is still 0 (off)

This is a safety verification — confirming the hardware protection circuit is working. Even if someone accidentally calls `set_driver_on()`, no current flows without the interlock being satisfied.

#### TEC thermal control test (interlock open)

```
TestTECThermalControlInterlockOpen::test_tec_setpoint_and_response
```

This test runs the TEC while the analogue interlock pin is open. It:

1. Confirms the driver is off
2. Calls `deny_interlock()` — tells the controller via the serial interface to ignore pin 15
3. Enables TEC in internal (digital) control mode
4. Reads the starting temperature
5. Sets the setpoint 2 °C higher
6. Waits 2 seconds
7. Checks that the stored setpoint matches what was commanded (±0.3 °C)
8. Checks that the measured temperature has not drifted more than 2 °C from the setpoint

The 2-second wait and 2 °C tolerance are intentionally loose here — this is a basic sanity check, not a precision stability test. The detailed TEC tests are in Stage 2.

---

## 5. Stage 2 — TEC, thermistor, and PID tests (no laser)

**File:** `test_tec_pid.py`

### Physical setup for this stage

The laser PCB must be connected to the output socket with the TEC and thermistor wired up. The laser diode itself does not need to be installed or connected. See [Section 7](#7-physical-wiring-reference) for the full wiring table.

### Phase 1 — Thermistor (`TestThermistor`)

These three tests run with the TEC completely off — no current to the Peltier. They test only the thermistor sensor circuit.

**`test_temperature_readable`**  
Reads the temperature once. It must be a float between 15 and 35 °C (the expected lab range). If this fails:
- If the value is 0.0: the thermistor is not connected to output socket pins 11 and 12
- If the value is negative or very large: the thermistor wiring has a short or open circuit
- If the value is outside 15–35 °C but plausible: the lab is unusually cold or warm

**`test_temperature_not_frozen`**  
Takes 5 readings in quick succession. Checks that they are not all identical at 0.0 (which would mean an open-circuit thermistor returning a default value), and that the spread across 5 readings is less than 1 °C (passive temperature without any TEC activity should be very stable). If the spread exceeds 1 °C, the thermistor connection is intermittent or noisy.

**`test_tec_is_off_at_start`**  
Reads the TEC state register and confirms it returns a 3-tuple. This is a connectivity check more than a functional test.

### Phase 2 — TEC enable/disable (`TestTECEnableDisable`)

**`test_tec_state_readable`**  
Reads the TEC state register before the TEC is enabled. Confirms the register is accessible.

**`test_tec_enables_with_internal_control`**  
This is the key enable test. It:
1. Calls `deny_interlock()` — see [Section 6](#6-the-interlock--what-it-is-and-the-digital-workaround) for why this is needed
2. Calls `set_tec_int()` — tells the controller that temperature setpoints come from digital commands (not the onboard potentiometer or the analogue connector)
3. Calls `set_tec_on()` — starts the PID loop and Peltier current
4. Reads the TEC state register and confirms the TEC is reported as running

If this fails, the most likely causes are: the Peltier is not connected (output socket pins 5–8), or the thermistor is not connected (pins 11–12) — both are required for the TEC to start.

**`test_laser_driver_stays_off_while_tec_runs`**  
Reads the driver state and confirms the laser driver bit is 0 (off). This is a safety check confirming that enabling the TEC did not accidentally enable the laser driver.

**`test_tec_disables_cleanly`**  
Calls `set_tec_off()` and confirms the TEC state register changes to stopped.

### Phase 3 — TEC setpoint registers (`TestTECSetpoint`)

These tests confirm that what you write to the controller is what it stores. Before each test the TEC is enabled (via the `ensure_tec_on` fixture that runs automatically).

The tests write setpoints of 20 °C, 25 °C, and 30 °C, then immediately read back the stored value. The tolerance is ±0.3 °C, which is much larger than the controller's 0.01 °C resolution — this gives plenty of room for any rounding in the serial conversion.

> **Important:** These tests check the *stored register value*, not the *measured temperature*. The measured temperature will lag behind the setpoint while the PID works to reach it. The register readback should be immediate and exact.

### Phase 4 — PID parameters (`TestPIDParameters`)

Reads the three PID gain registers (P, I, D) and checks:
- Each is an integer (correct data type from the serial response)
- Each is non-negative
- At least one of them is non-zero (if all three are zero the PID controller is unconfigured and temperature regulation will not work)

The SF8025-NM ships with factory-tuned gains. These tests do not change the gains — they only verify the gains can be read and are sensible.

### Phase 5 — PID response (`TestPIDResponse`)

These are the most physically meaningful tests in the file. They verify that the PID closed loop is actually working: the thermistor feeds temperature into the PID, the PID calculates a correction, and the Peltier produces that correction as heat or cooling.

**`test_tec_current_nonzero_when_off_setpoint`**  
Sets the temperature setpoint 5 °C away from the current measured temperature. Waits 20 seconds. Reads the TEC current. If the PID loop is closed and the Peltier is connected, the PID will be actively driving current to try to reach the new setpoint. The test requires the absolute value of TEC current to be above 10 mA (0.01 A). If current is zero, either the Peltier is not connected or the PID is not running.

**`test_temperature_moves_toward_setpoint`**  
Similar to above, but instead of checking current it checks that the *measured temperature* has moved in the direction of the new setpoint. It computes the error before and after the 20-second wait and confirms the new error is smaller. This is a direct verification that the feedback loop is closed: thermistor → PID → Peltier → heat/cool → thermistor.

**`test_tec_current_within_limit`**  
At any point during PID operation, the current through the Peltier must not exceed the configured current limit. The factory default limit is 2 A. Exceeding this could damage the Peltier.

**`test_temperature_stabilises_at_25c`**  
Sets the setpoint to 25 °C, waits 60 seconds, then takes 10 readings one second apart. Checks that:
- The average of those 10 readings is within ±0.5 °C of 25 °C
- The spread (maximum minus minimum) across the 10 readings is below 0.5 °C (no oscillation)

> **Note for lab use:** The test plan (section 3.2) requires ±0.1 °C stability over 5 minutes before any laser measurements begin. This automated test uses a relaxed ±0.5 °C and only 60 seconds to keep total test time reasonable. The formal 5-minute stability check should be done manually or via BenchSoft before switching the laser on.

### Phase 6 — Fault/lock state (`TestLockStateDuringTECOperation`)

These tests run with the TEC active at 25 °C. They read the protection register and check that no fault flags are set.

| Test | Flag checked | What a failure means |
|---|---|---|
| `test_no_tec_error_flag` | TEC error | Peltier wiring problem, short circuit, or current limit too low |
| `test_no_tec_selfheat_flag` | TEC self-heat | The controller board itself is overheating — check its thermal mounting |
| `test_no_ld_overcurrent_flag` | LD overcurrent | Unexpected — no laser is connected so no current should flow through the driver |
| `test_driver_off_confirmed` | Driver on/off | Laser driver is somehow on — must not happen in this test file |
| `test_driver_state_still_readable` | Register access | Confirms the driver state register is still readable while TEC runs |

### Phase 7 — Setpoint sweep (`TestTECSetpointSweep`)

Sweeps the TEC setpoint across 20 °C → 25 °C → 30 °C with a 30-second dwell at each step. At each step it records:
- The commanded setpoint
- The setpoint read back from the controller register
- The measured temperature (from the thermistor)
- The TEC current in Amperes
- The TEC current limit

These values are printed as a formatted table in the terminal output (requires `pytest -s`). You can copy this table into your lab notebook as an abbreviated version of the test plan's Data Table 3 (section 6.4, Power vs. Temperature Sweep). Note that the full sweep required by the test plan uses 5-minute dwells — this automated version uses 30 seconds just to confirm the PID is tracking.

---

## 6. The interlock — what it is and the digital workaround

### What the interlock is

The SF8025-NM has a hardware safety pin called the **interlock** on the analogue control connector (pin 15). This pin has an internal pull-up resistor that holds it HIGH (+5 V) when nothing is connected to it.

The controller's protection logic watches this pin:
- If pin 15 is HIGH (open / not connected): the controller is in **locked** state. Neither the laser driver nor the TEC can be started.
- If pin 15 is LOW (shorted to GND): the controller is in **unlocked** state. Driver and TEC can be enabled normally.

The physical purpose is to allow an external emergency stop button: connect a normally-closed button between pin 15 and GND. If the button is pressed, the interlock opens and everything shuts off.

### The digital workaround

When you control the SF8025-NM via USB/UART (which is what the Python driver does), the datasheet (section 18.1) explicitly allows you to bypass the analogue interlock pin via software:

> *"Via USB you can set the driver to the 'deny interlock' state. At this case the driver will ignore interlock state and can operate with opened pin 15."*

The Python driver has this method:

```python
controller.deny_interlock()
```

Internally this sends the command `P0700 2000\r` to the controller (write bitmask `0x2000` to the DRIVER_STATE register). After this command, the controller ignores the physical state of pin 15 — it behaves as if the interlock is always closed.

This state persists until the controller is power-cycled or `allow_interlock()` is called.

### Important note for later stages

`deny_interlock()` applies to **both** the TEC and the laser driver (datasheet footnote 3 confirms this). In these TEC-only tests, calling `deny_interlock()` allows the TEC to run. The laser driver remains off because we never call `set_driver_on()`, and because the NTC interlock (pin 17) may still be open, which separately blocks the driver.

When you later run the laser with the driver enabled, you will need to make sure `deny_interlock()` is either called (if using digital control) or pin 15 is physically grounded (if using the analogue connector).

### Why the original test was failing

The test `TestTECThermalControlInterlockOpen::test_tec_setpoint_and_response` in `test_hardware.py` was calling `set_tec_on()` without first calling `deny_interlock()`. The controller was therefore ignoring the enable command (interlock still blocking it), the TEC never started, and two seconds later the temperature had not moved — causing the assertion `abs(t_measured - t_set) < 2.0` to fail by a tiny margin.

The fix was a single line added before the enable sequence:

```python
controller.deny_interlock()   # tell controller to ignore open pin 15
controller.set_tec_int()
controller.set_tec_on()
```

---

## 7. Known bugs found and fixed during development

This section records every bug that caused a test failure during development, what the root cause was, and exactly what was changed to fix it. This is useful background if a test fails again in the future.

---

### Bug 1 — TEC would not enable: missing `deny_interlock()` call

**Tests affected:** `test_hardware.py::TestTECThermalControlInterlockOpen::test_tec_setpoint_and_response`

**Symptom:** Test failed with `assert abs(t_measured - t_set) < 2.0`.

**Root cause:** The SF8025-NM powers up with its interlock in the "allow" state — it checks analogue connector pin 15 before enabling either the laser driver or the TEC. Pin 15 was open (not grounded). When using the digital (UART) interface you can bypass this by calling `deny_interlock()`, which sends `DRIVER_STATE 0x2000` to tell the firmware to ignore pin 15 permanently until the next power cycle. The test was calling `set_tec_on()` without calling `deny_interlock()` first, so the TEC never started.

**Fix:** Added `controller.deny_interlock()` immediately before `controller.set_tec_int()` in that test. The same call was added to every fixture and test in `test_tec_pid.py` that enables the TEC.

**File changed:** `test_hardware.py` (one line added), `test_tec_pid.py` (all TEC enable sequences).

---

### Bug 2 — TEC would not enable even with `deny_interlock()`: setpoint not written before enabling

**Tests affected (all three simultaneously):**
- `test_tec_pid.py::TestTECEnableDisable::test_tec_enables_with_internal_control`
- `test_tec_pid.py::TestPIDResponse::test_tec_current_nonzero_when_off_setpoint`
- `test_tec_pid.py::TestPIDResponse::test_temperature_stabilises_at_25c`

**Symptom:** TEC reported as STOPPED after `set_tec_on()`; TEC current was 0; temperature did not stabilise.

**Root cause — part A (order of operations):** The SF8025-NM firmware will not start the TEC unless a valid temperature setpoint has already been written to the digital setpoint register (`TEC_TEMPERATURE_VALUE`, address `0A10h`). The factory default setpoint only applies to the analogue/potentiometer path. When the controller is switched to digital control by `set_tec_int()`, the digital register starts at 0 (= 0.00 °C), which is below the TEC's valid operating range of +15 to +40 °C. The firmware refuses to start the TEC with an out-of-range setpoint.

In `test_hardware.py`, the test `test_set_tec_temperature(25.0)` happens to run before the TEC enable test, so the digital setpoint register already holds 25.00 °C when `set_tec_on()` is called — and it works. In `test_tec_pid.py`, the `TestTECEnableDisable` tests run before `TestTECSetpoint`, so no setpoint had been written yet — and the TEC refused to start.

Every fixture in `test_tec_pid.py` also had the same mistake: it called `set_tec_on()` first, then `set_tec_temperature(25.0)` — the wrong order.

**Root cause — part B (timing bug in `set_tec_on()`):** After sending the Start command (a P-type write), `set_tec_on()` immediately reads back the TEC state register to confirm the TEC has started. However, the SF8025-NM firmware does not acknowledge P-type commands — so `read_until()` waits for the 200 ms serial timeout before returning an empty response. After this, `set_tec_on()` reads the state register right away. If the firmware needs longer than 200 ms to process the Start command and update the register, the read-back returns "STOPPED" even though the TEC is actually about to start.

Compare with `set_driver_on()` in the same driver: that method sends the Start command and *immediately returns 0* without reading back state at all. `set_tec_on()` is stricter but has no delay, making it brittle.

**Fix — part A:** Changed every fixture and test in `test_tec_pid.py` to call `set_tec_temperature(SP_25)` *before* calling `set_tec_on()`. The correct TEC enable sequence is now enforced everywhere:

```python
controller.deny_interlock()          # 1. bypass analogue interlock pin 15
controller.set_tec_int()             # 2. switch control to digital/UART mode
controller.set_tec_temperature(25.0) # 3. write valid setpoint BEFORE enabling
controller.set_tec_on()              # 4. send Start command
```

**Fix — part B:** Added a 500 ms `time.sleep()` inside `set_tec_on()` in `controller.py`, between sending the Start command and reading the state register. 500 ms is well within the loop time of the SF8025-NM firmware and gives the state register time to update. The test for `test_tec_enables_with_internal_control` was also restructured to check state independently via `controller.tec_state()` rather than relying solely on the return value of `set_tec_on()`.

**Files changed:** `controller.py` (`set_tec_on()` — one `time.sleep(0.5)` added), `test_tec_pid.py` (all fixtures and the enable test).

---

## 8. Physical wiring reference

### Output socket (14-pin, on the controller board)

Ribbon cable from controller to laser PCB.

| Output socket pins | Signal | Connect to |
|---|---|---|
| 1, 2 | LD Anode (+) | Laser diode anode — **leave unconnected at this stage** |
| 3, 4 | LD Cathode (−) | Laser diode cathode — **leave unconnected at this stage** |
| 5, 6 | TEC Out + | Peltier cooler positive terminal on laser PCB |
| 7, 8 | TEC Out − | Peltier cooler negative terminal on laser PCB |
| 9, 10 | GND | System ground |
| 11 | Thermistor | NTC 10 kΩ thermistor on laser PCB |
| 12 | Thermistor | NTC 10 kΩ thermistor on laser PCB (other leg) |
| 13, 14 | GND | System ground |

> **Polarity warning (from datasheet):** Never ground any lead of the laser driver output (pins 1–4). Never use grounded probes (e.g. from an oscilloscope) on the output. Incorrect polarity will permanently damage the laser and the driver.

### Analogue control connector (20-pin)

| Analogue pin | Signal | Used for | Required now? |
|---|---|---|---|
| 2 | TEC Enable | HIGH = TEC on (not used; digital control is used instead) | No |
| 3 | Laser Driver Enable | HIGH = driver on | No |
| 4 | TEC Error output | Fault indicator | Monitor only |
| 5 | LD Overcurrent output | Fault indicator | Monitor only |
| 11 | Driver Current Monitor | Analogue voltage proportional to drive current | Optional |
| 13 | TEC Temperature Monitor | Analogue voltage proportional to thermistor temp | Optional |
| 15 | Interlock | Short to GND to unlock (analogue control only) | **No — handled by `deny_interlock()` in software** |
| 16 | GND | Reference ground | — |
| 17 | NTC Interlock | Connect NTC between this pin and GND for extra safety interlock on driver | Optional |

### ESD protection jumper (board item 11)

A small shorting jumper on the controller board that connects the laser driver output terminals together. While installed, even if the driver is accidentally enabled, the output is shorted so no voltage appears on the pins — protecting the laser diode from static discharge during handling.

**Remove this jumper only when a laser diode is physically installed and you are ready to enable the driver.** You have already removed it. Do not reinstall it unless you are disconnecting and reconnecting the laser.

---

## 9. How to run the tests

### Prerequisites

- Controller powered (5 V DC, power supply ≥ 25 W)
- USB cable connected from controller to PC, appearing as COM3
- Laser PCB connected to output socket (TEC and thermistor wired; laser diode not installed yet)
- ESD protection jumper removed
- Python environment with `pytest` and `pyserial` installed

### Running Stage 1

```bash
cd pymeasure/pymeasure/instruments/maiman_laser_controller
pytest test_hardware.py -v
```

Expected output: all tests PASSED. The last test (`test_tec_setpoint_and_response`) will take ~5 seconds due to the `time.sleep(2)` and TEC startup.

### Running Stage 2

```bash
pytest test_tec_pid.py -v -s
```

The `-v` flag gives verbose output (one line per test). The `-s` flag allows the sweep table from Phase 7 to print to the terminal.

Expected total run time: approximately 5–8 minutes due to the stabilisation waits.

### Running a single test or phase

```bash
# Run only the thermistor phase
pytest test_tec_pid.py::TestThermistor -v

# Run only the PID response phase
pytest test_tec_pid.py::TestPIDResponse -v -s

# Run only the sweep
pytest test_tec_pid.py::TestTECSetpointSweep -v -s
```

### Changing wait times

The wait constants at the top of `test_tec_pid.py` can be adjusted:

```python
STABILISATION_WAIT = 60    # seconds — increase to 300 for the formal ±0.1 °C check
RESPONSE_WAIT = 20         # seconds — time given for PID to respond to a step
```

---

## 10. Reading test output and what failures mean

### Typical passing output

```
test_tec_pid.py::TestThermistor::test_temperature_readable         PASSED
test_tec_pid.py::TestThermistor::test_temperature_not_frozen       PASSED
test_tec_pid.py::TestThermistor::test_tec_is_off_at_start          PASSED
test_tec_pid.py::TestTECEnableDisable::test_tec_enables_...        PASSED
...
```

### Common failures and causes

| Failure | Most likely cause |
|---|---|
| `test_connected` FAILED | Wrong COM port, cable unplugged, controller not powered |
| `test_temperature_readable` FAILED with value 0.0 | Thermistor not connected to output socket pins 11/12 |
| `test_temperature_readable` FAILED with value out of range | Thermistor short/open circuit, or very unusual lab temperature |
| `test_tec_enables_with_internal_control` FAILED | See Bug 1 and Bug 2 in Section 7. Most likely: Peltier or thermistor not connected |
| `test_tec_current_nonzero_when_off_setpoint` FAILED | TEC not running (enable sequence failed); or Peltier not physically connected |
| `test_temperature_moves_toward_setpoint` FAILED | Thermistor feedback broken; Peltier polarity reversed; or TEC not running |
| `test_temperature_stabilises_at_25c` FAILED (wrong average) | TEC not running — temperature stayed at ambient, not 25 °C |
| `test_temperature_stabilises_at_25c` FAILED (spread too large) | TEC running but PID oscillating — check thermal paste quality; lower TEC current limit |
| `test_no_tec_error_flag` FAILED | Peltier short circuit, wrong polarity wiring, or current limit set too low |
| `test_no_tec_selfheat_flag` FAILED | Controller board overheating — needs better thermal mounting or airflow |

### Diagnosing "TEC not running" failures (the most common group)

If three or more tests fail simultaneously — especially `test_tec_enables_with_internal_control`, `test_tec_current_nonzero_when_off_setpoint`, and `test_temperature_stabilises_at_25c` — the TEC is not starting at all. Work through this checklist in order:

1. **Is the Peltier connected?** Output socket pins 5, 6 (TEC Out+) and 7, 8 (TEC Out−) must be wired to the Peltier on the laser PCB. The controller's firmware refuses to start the TEC with an open-circuit TEC output.
2. **Is the thermistor connected?** Output socket pins 11 and 12 must be wired to the NTC 10 kΩ thermistor. The `TestThermistor` tests (Phase 1) run first and will catch this before the TEC tests.
3. **Has the setpoint been written before enabling?** The correct sequence is `deny_interlock()` → `set_tec_int()` → `set_tec_temperature(25.0)` → `set_tec_on()`. If any call is missing or in the wrong order, the TEC will not start. See Bug 2 in Section 7.
4. **Is `deny_interlock()` being called?** Without this, the firmware checks analogue pin 15. If pin 15 is open, the TEC is blocked. See Bug 1 in Section 7 and Section 6.

---

## 11. Glossary

| Term | Plain-language definition |
|---|---|
| **APC connector** | A type of fibre-optic connector with an 8° angled polish on the end-face. The angle prevents back-reflections into the laser. The fibre pigtail on the laser diode uses this. |
| **Bitmask** | A number where each individual binary digit (bit) represents a separate on/off flag. The controller uses bitmasks to pack multiple status flags into a single register value. |
| **CW (Continuous Wave)** | The laser emits a steady, constant beam — as opposed to pulsed operation where it flashes on and off. All tests in this project use CW mode. |
| **DFB laser** | Distributed Feedback laser. A type of semiconductor laser that emits at a single, very precise wavelength. The feedback (like a mirror in a traditional laser) is provided by a grating etched into the semiconductor material. |
| **ESD** | Electrostatic Discharge. The spark you sometimes get when touching a doorknob. Even a tiny, invisible ESD event can permanently destroy a laser diode. The ESD jumper on the controller protects the laser during handling. |
| **Fixture (pytest)** | A piece of setup code that runs before a test (and optionally teardown code after). In these test files, the `controller` fixture opens the serial connection and closes it when tests finish. |
| **Interlock** | A hardware safety circuit that prevents the laser driver (and TEC) from operating unless an external condition is met. Here it is an analogue pin that must be grounded or bypassed via software. |
| **LDD** | Laser Diode Driver. The precision constant-current source inside the SF8025-NM that supplies current to the laser diode. |
| **NTC thermistor** | Negative Temperature Coefficient thermistor. A resistor whose resistance decreases as temperature increases. Used as the temperature sensor in this system. |
| **Output socket** | The 14-pin connector on the Maiman controller board where the laser, Peltier, and thermistor are all connected. |
| **P-I curve** | A graph of optical output Power (y-axis) versus Drive Current (x-axis). It shows the lasing threshold and the slope efficiency of the laser. |
| **Peltier cooler** | A solid-state heat pump with no moving parts. Current in one direction cools one face and heats the other; reversing the current reverses the roles. |
| **PID** | Proportional-Integral-Derivative. A standard feedback control algorithm that drives a measured quantity toward a desired setpoint. |
| **Register** | A named storage location inside the controller firmware. Reading a register retrieves a value; writing a register changes a setting. All communication is done by reading and writing registers over the serial connection. |
| **Serial / UART** | Universal Asynchronous Receiver-Transmitter. A simple two-wire communication protocol (transmit and receive). Used here at 115200 baud (bits per second) to send text commands from the PC to the controller. |
| **Setpoint** | The target value you want the controller to achieve. For the TEC, the temperature setpoint is what you want the laser chip to be held at (e.g. 25 °C). |
| **TEC** | Thermoelectric Cooler (or Controller, depending on context). As hardware it refers to the Peltier element. As software/firmware it refers to the PID controller loop that drives that element. |
| **Threshold current** | The drive current at which a laser diode transitions from spontaneous emission (dim, broad-spectrum light, like an LED) to stimulated emission (coherent laser light). For this laser the datasheet specifies ~11 mA. |
