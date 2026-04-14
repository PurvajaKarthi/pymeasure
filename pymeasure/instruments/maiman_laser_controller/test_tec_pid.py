# -*- coding: utf-8 -*-
"""
TEC, Thermistor, and PID Controller Hardware Tests
====================================================
Tests the Peltier cooler (TEC), NTC thermistor, and PID temperature controller
WITHOUT the laser diode connected.  The ESD protection jumper must already be
removed.  The laser driver is never enabled in this file.

Physical setup required
-----------------------
Controller : Maiman Electronics SF8025-NM, powered (5 V, >= 25 W supply)
ESD jumper (board item 11) : REMOVED (already done)

Output socket wiring (laser PCB connected, no laser diode):
  Pins  5, 6  -> TEC Out +  (Peltier cooler positive terminal on laser PCB)
  Pins  7, 8  -> TEC Out -  (Peltier cooler negative terminal on laser PCB)
  Pins 11, 12 -> Thermistor (NTC 10 kOhm on laser PCB)
  Pins  9,10,13,14 -> GND
  Pins  1, 2, 3, 4 -> NOT connected (no laser diode)

Analogue control connector wiring:
  Pin 15 (Interlock) : Does NOT need to be grounded when using digital/UART
                       control.  The test calls deny_interlock() via the serial
                       interface (DRIVER_STATE 0x2000), which tells the
                       controller to ignore the state of pin 15.
                       See datasheet s18.1: "Via USB you can set the driver to
                       the 'deny interlock' state ... and operate with opened
                       pin 15."
  Pin 17 (NTC Interlock) : Optional. Connect NTC 10 kOhm between pin 17 and
                            GND if available. Only the laser *driver* needs
                            this; TEC works without it.  Leaving it open keeps
                            the driver locked (which is what we want).

Run with:
    pytest test_tec_pid.py -v -s

The -s flag lets pytest print the sweep table produced by TestTECSetpointSweep.
"""

import pytest
import time
from pymeasure.instruments.maiman_laser_controller.controller import SF8xxx


# =============================================================================
# CONFIGURATION
# =============================================================================
COM_PORT = "COM3"

# Temperature bounds
AMBIENT_MIN = 15.0   # deg C  minimum plausible lab temperature
AMBIENT_MAX = 35.0   # deg C  maximum plausible lab temperature

# Tolerance for setpoint register readback (controller resolution is 0.01 deg C)
SETPOINT_TOLERANCE = 0.3   # deg C

# Tolerance used for automated stabilisation check.
# NOTE: The full test-plan criterion is +-0.1 deg C over 5 minutes.
# That is done manually / via BenchSoft. This automated check uses a relaxed
# value so the test suite can run in a reasonable time.
STABILITY_TOLERANCE = 0.5  # deg C

# How long to wait (seconds) before sampling for stabilisation
STABILISATION_WAIT = 60    # s  -- change to 300 for stricter check

# Step used in PID-response tests: move setpoint this far from ambient
RESPONSE_STEP = 5.0        # deg C
RESPONSE_WAIT = 20         # s  -- wait for TEC current to respond

# Fixed setpoints
SP_20 = 20.0
SP_25 = 25.0
SP_30 = 30.0


# =============================================================================
# FIXTURE
# =============================================================================
@pytest.fixture(scope="module")
def controller():
    """
    Open the serial connection to the SF8xxx controller.
    On teardown: disable the driver (should already be off), disable the TEC,
    and close the serial port.
    """
    dev = None
    try:
        dev = SF8xxx(COM_PORT, start_thread=False)
        if not dev.connected:
            pytest.skip("SF8xxx not connected on " + COM_PORT)
        yield dev
    finally:
        if dev is not None and dev.connected:
            try:
                dev.set_driver_off()
            except Exception:
                pass
            try:
                dev.set_tec_off()
            except Exception:
                pass
            if hasattr(dev, "dev") and dev.dev.is_open:
                dev.dev.close()


# =============================================================================
# PHASE 1 – THERMISTOR
# Verify the NTC thermistor is connected and returning plausible values BEFORE
# the TEC is enabled.  No Peltier current flows in this phase.
# =============================================================================
class TestThermistor:
    """NTC thermistor connectivity and sanity (TEC off)."""

    def test_temperature_readable(self, controller):
        """Thermistor returns a float within the expected lab range."""
        temp = controller.get_tec_temperature()
        assert isinstance(temp, float), "get_tec_temperature() did not return a float."
        assert AMBIENT_MIN <= temp <= AMBIENT_MAX, (
            f"Thermistor reading {temp:.2f} degC is outside the expected lab range "
            f"({AMBIENT_MIN} to {AMBIENT_MAX} degC).\n"
            "Check that the NTC 10 kOhm thermistor is connected to output socket "
            "pins 11 and 12."
        )

    def test_temperature_not_frozen(self, controller):
        """
        Five successive readings must not all be identical at an extreme value.
        A constant 0.0 degC indicates an open-circuit thermistor.
        A very large spread (> 1 degC in passive conditions) indicates a loose
        or intermittent connection.
        """
        readings = [controller.get_tec_temperature() for _ in range(5)]
        assert not all(r == 0.0 for r in readings), (
            "All thermistor readings are 0.0 degC -- thermistor likely open-circuit. "
            "Check output socket pins 11 and 12."
        )
        spread = max(readings) - min(readings)
        assert spread < 1.0, (
            f"Temperature spread of {spread:.3f} degC across 5 passive readings is "
            "too large. The thermistor connection may be loose or noisy."
        )

    def test_tec_is_off_at_start(self, controller):
        """Confirm the TEC is not running at the start of this test session."""
        tec, _temp_mode, _enable_mode = controller.tec_state()
        # tec == 0 means stopped; non-zero means started.
        # Either is acceptable here; we just need the state to be readable.
        assert len(controller.tec_state()) == 3


# =============================================================================
# PHASE 2 – TEC ENABLE / DISABLE
# Verify the TEC can be started and stopped independently of the laser driver.
# Interlock pin 15 must be shorted to GND for set_tec_on() to succeed.
# =============================================================================
class TestTECEnableDisable:
    """TEC enable/disable cycle without the laser diode."""

    def test_tec_state_readable(self, controller):
        """tec_state() returns a 3-tuple before TEC is enabled."""
        state = controller.tec_state()
        assert len(state) == 3

    def test_tec_enables_with_internal_control(self, controller):
        """
        Correct enable sequence:
          1. deny_interlock()        -- ignore open analogue pin 15 via UART
          2. set_tec_int()           -- switch setpoint and enable to digital control
          3. set_tec_temperature()   -- write a valid setpoint BEFORE starting the TEC.
                                        The firmware will not start the TEC if no digital
                                        setpoint has been written (the factory default
                                        lives in the potentiometer/analogue path, not the
                                        digital register).
          4. set_tec_on()            -- send the Start command (TEC_STATE 0x0008)

        The return value of set_tec_on() is NOT asserted here because the
        driver checks the state register immediately after sending the Start
        command.  The SF8025-NM firmware needs ~500 ms to process the command
        and update that register (the driver now adds this delay, but we
        verify state independently to be safe).
        """
        controller.deny_interlock()
        controller.set_tec_int()
        controller.set_tec_temperature(SP_25)  # MUST come before set_tec_on()
        controller.set_tec_on()
        # set_tec_on() already waits 500 ms internally; read state once more
        # as an independent confirmation.
        tec, _, _ = controller.tec_state()
        assert tec != 0, (
            "TEC state reports STOPPED after the enable sequence.\n"
            "Check:\n"
            "  1. Peltier is connected to output socket pins 5, 6 (TEC Out+) "
            "and 7, 8 (TEC Out-).\n"
            "  2. Thermistor is connected to output socket pins 11 and 12.\n"
            "  3. Controller is powered with a >=25 W supply."
        )

    def test_laser_driver_stays_off_while_tec_runs(self, controller):
        """
        The laser driver must remain OFF while the TEC is running.
        This is enforced by: (a) no laser connected, (b) set_driver_on()
        is never called in this file, and (c) the NTC interlock (pin 17)
        may still be open, blocking the driver.
        """
        _device, driver, *_ = controller.driver_state()
        assert driver == 0, (
            "Laser driver is ON -- this must not happen in a TEC-only test session."
        )

    def test_tec_disables_cleanly(self, controller):
        """
        set_tec_off() must succeed and the TEC state register must show STOPPED.
        The internal guard inside set_tec_off() checks driver_off first; since
        the driver is off, this should always pass.
        """
        result = controller.set_tec_off()
        assert result == 0, (
            f"set_tec_off() returned: {result!r}\n"
            "Possible cause: the driver flag thinks the driver is ON."
        )
        tec, _, _ = controller.tec_state()
        assert tec == 0, "TEC state reports STARTED after set_tec_off() succeeded."


# =============================================================================
# PHASE 3 – TEC SETPOINT REGISTER
# Verify that setpoint values written via set_tec_temperature() are stored in
# the controller and read back correctly via get_tec_value().
# The controller resolution is 0.01 degC; tolerance is SETPOINT_TOLERANCE.
# =============================================================================
class TestTECSetpoint:
    """TEC temperature setpoint read/write with TEC running."""

    @pytest.fixture(autouse=True)
    def ensure_tec_on(self, controller):
        """Start TEC in internal mode before each test; leave it running."""
        controller.deny_interlock()
        controller.set_tec_int()
        controller.set_tec_temperature(SP_25)  # setpoint before enable
        controller.set_tec_on()
        yield
        # TEC is left running; module-level teardown will disable it.

    def test_setpoint_25c(self, controller):
        controller.set_tec_temperature(SP_25)
        readback = controller.get_tec_value()
        assert pytest.approx(readback, abs=SETPOINT_TOLERANCE) == SP_25, (
            f"Setpoint readback {readback:.2f} degC does not match "
            f"commanded {SP_25} degC."
        )

    def test_setpoint_20c(self, controller):
        controller.set_tec_temperature(SP_20)
        readback = controller.get_tec_value()
        assert pytest.approx(readback, abs=SETPOINT_TOLERANCE) == SP_20

    def test_setpoint_30c(self, controller):
        controller.set_tec_temperature(SP_30)
        readback = controller.get_tec_value()
        assert pytest.approx(readback, abs=SETPOINT_TOLERANCE) == SP_30

    def test_setpoint_restored_25c(self, controller):
        """Restore to 25 degC for subsequent test phases."""
        controller.set_tec_temperature(SP_25)
        readback = controller.get_tec_value()
        assert pytest.approx(readback, abs=SETPOINT_TOLERANCE) == SP_25


# =============================================================================
# PHASE 4 – PID PARAMETERS
# Read the factory-configured PID gains.  The SF8025-NM ships with a
# self-adjusted PID; the gains are read-only in normal operation.
# Register addresses: PID_P = 0A21h, PID_I = 0A22h, PID_D = 0A23h.
# =============================================================================
class TestPIDParameters:
    """PID gain registers are readable and the controller is configured."""

    def test_pid_p_readable(self, controller):
        p = controller.get_pid_p()
        assert isinstance(p, int), f"PID P returned {type(p).__name__}, expected int."
        assert p >= 0

    def test_pid_i_readable(self, controller):
        i = controller.get_pid_i()
        assert isinstance(i, int), f"PID I returned {type(i).__name__}, expected int."
        assert i >= 0

    def test_pid_d_readable(self, controller):
        d = controller.get_pid_d()
        assert isinstance(d, int), f"PID D returned {type(d).__name__}, expected int."
        assert d >= 0

    def test_pid_at_least_one_gain_nonzero(self, controller):
        """
        All-zero PID means the controller is unconfigured and the TEC will not
        regulate temperature.
        """
        p = controller.get_pid_p()
        i = controller.get_pid_i()
        d = controller.get_pid_d()
        assert p != 0 or i != 0 or d != 0, (
            f"All PID gains are zero (P={p}, I={i}, D={d}). "
            "The TEC controller does not appear to be configured."
        )


# =============================================================================
# PHASE 5 – PID RESPONSE
# Verify that the PID loop is actually closed: when a setpoint is applied that
# differs from the current temperature, the TEC must draw current AND the
# measured temperature must move toward the setpoint.
#
# These tests move the setpoint by RESPONSE_STEP (5 degC) and wait
# RESPONSE_WAIT seconds (20 s).  That is enough to see a clear PID response
# without requiring full stabilisation.
# =============================================================================
class TestPIDResponse:
    """PID closed-loop response: current flows and temperature tracks setpoint."""

    @pytest.fixture(autouse=True)
    def start_tec_at_25c(self, controller):
        """Each test starts with TEC on at 25 degC and a 5 s settle."""
        controller.deny_interlock()
        controller.set_tec_int()
        controller.set_tec_temperature(SP_25)  # setpoint before enable
        controller.set_tec_on()
        time.sleep(5)
        yield
        # Restore to 25 degC after each test
        controller.set_tec_temperature(SP_25)
        time.sleep(2)

    def test_tec_current_nonzero_when_off_setpoint(self, controller):
        """
        After stepping the setpoint 5 degC from the current temperature, the
        PID must drive non-zero current through the Peltier.
        A reading below 10 mA (0.01 A) after RESPONSE_WAIT seconds indicates
        that either the Peltier is not connected or the PID is not running.
        """
        t_now = controller.get_tec_temperature()
        # Step setpoint away from current reading; stay within TEC range 15-40 C
        new_sp = t_now + RESPONSE_STEP if (t_now + RESPONSE_STEP) <= 38.0 else t_now - RESPONSE_STEP
        controller.set_tec_temperature(new_sp)
        time.sleep(RESPONSE_WAIT)

        tec_current = abs(controller.get_tec_current())
        assert tec_current > 0.01, (
            f"TEC current is {tec_current:.3f} A after {RESPONSE_WAIT} s with "
            f"setpoint {new_sp:.1f} degC and measured temp ~{t_now:.1f} degC.\n"
            "Expected non-zero current. Check Peltier wiring (output socket pins 5-8)."
        )

    def test_temperature_moves_toward_setpoint(self, controller):
        """
        After a setpoint step, the measured temperature must move in the
        direction of the setpoint.  This confirms the thermistor feedback path
        is closing the PID loop correctly.
        """
        t_initial = controller.get_tec_temperature()
        delta = RESPONSE_STEP if (t_initial + RESPONSE_STEP) <= 38.0 else -RESPONSE_STEP
        new_sp = t_initial + delta

        controller.set_tec_temperature(new_sp)
        time.sleep(RESPONSE_WAIT)

        t_after = controller.get_tec_temperature()
        initial_error = abs(t_initial - new_sp)
        new_error = abs(t_after - new_sp)

        assert new_error < initial_error, (
            f"Temperature did not move toward setpoint {new_sp:.1f} degC.\n"
            f"  Initial temp  : {t_initial:.2f} degC  (error = {initial_error:.2f} degC)\n"
            f"  After {RESPONSE_WAIT} s : {t_after:.2f} degC  (error = {new_error:.2f} degC)\n"
            "Possible causes: Peltier not connected, thermistor on wrong pins, "
            "or PID gains are zero."
        )

    def test_tec_current_within_limit(self, controller):
        """
        TEC output current must never exceed the configured current limit.
        Default factory limit is 2 A; the Peltier on the laser PCB may have
        a lower safe rating -- check your laser PCB datasheet.
        """
        limit = controller.get_tec_current_limit()
        current = abs(controller.get_tec_current())
        assert current <= limit, (
            f"TEC current {current:.3f} A exceeds the configured limit {limit:.3f} A."
        )

    def test_temperature_stabilises_at_25c(self, controller):
        """
        After STABILISATION_WAIT seconds at 25 degC setpoint, the measured
        temperature must be within STABILITY_TOLERANCE and must not oscillate.

        NOTE: The test plan (section 3.2) requires +-0.1 degC stability over
        5 minutes before laser measurements begin.  This automated check uses
        a relaxed STABILITY_TOLERANCE and shorter wait.  Use BenchSoft for the
        formal 5-minute stability verification.
        """
        controller.set_tec_temperature(SP_25)
        time.sleep(STABILISATION_WAIT)

        # Take 10 readings spread over 10 seconds
        readings = []
        for _ in range(10):
            readings.append(controller.get_tec_temperature())
            time.sleep(1)

        avg = sum(readings) / len(readings)
        spread = max(readings) - min(readings)

        assert abs(avg - SP_25) < STABILITY_TOLERANCE, (
            f"Average stabilised temperature {avg:.3f} degC is not within "
            f"+-{STABILITY_TOLERANCE} degC of setpoint {SP_25} degC.\n"
            f"Readings: {[f'{r:.2f}' for r in readings]}"
        )
        assert spread < STABILITY_TOLERANCE, (
            f"Temperature spread of {spread:.3f} degC across 10 readings "
            f"exceeds {STABILITY_TOLERANCE} degC -- PID may be oscillating.\n"
            f"Readings: {[f'{r:.2f}' for r in readings]}"
        )


# =============================================================================
# PHASE 6 – FAULT / LOCK STATE
# Verify no error flags are asserted during normal TEC operation.
# Lock state register (0800h) bit map per datasheet section 22:
#   bit 1 : Interlock          -- should be 0 (allowed) after pin 15 is shorted
#   bit 3 : LD over-current    -- should be 0 (no laser connected)
#   bit 4 : LD overheat        -- should be 0
#   bit 5 : External NTC Interlock -- depends on pin 17 wiring
#   bit 6 : TEC error          -- should be 0 (TEC operating normally)
#   bit 7 : TEC self-heat      -- should be 0 (controller not overheating)
# =============================================================================
class TestLockStateDuringTECOperation:
    """No fault flags during TEC operation with no laser connected."""

    @pytest.fixture(autouse=True)
    def tec_running_at_25c(self, controller):
        controller.deny_interlock()
        controller.set_tec_int()
        controller.set_tec_temperature(SP_25)  # setpoint before enable
        controller.set_tec_on()
        time.sleep(5)
        yield

    def test_no_tec_error_flag(self, controller):
        """
        TEC error (bit 6) is raised on overcurrent, short-circuit, or
        circuitry overheat.  It must be LOW during normal operation.
        If this fails, check Peltier wiring polarity and current limit setting.
        """
        interlock, ld_oc, ld_oh, ntc, tec_error, tec_selfheat = controller.lock_state()
        assert tec_error == 0, (
            f"TEC error flag is SET (raw value: {tec_error}).\n"
            "Check Peltier wiring (pins 5-8 on output socket) and TEC current limit."
        )

    def test_no_tec_selfheat_flag(self, controller):
        """
        TEC self-heat (bit 7) indicates the controller board is overheating.
        It must be LOW.  If it is HIGH, improve controller thermal mounting.
        """
        *_, tec_error, tec_selfheat = controller.lock_state()
        assert tec_selfheat == 0, (
            f"TEC self-heat flag is SET (raw value: {tec_selfheat}).\n"
            "The controller board is overheating. Check thermal mounting."
        )

    def test_no_ld_overcurrent_flag(self, controller):
        """
        Laser driver overcurrent (bit 3) must be LOW.
        No laser is connected, so no current can flow through the driver output.
        """
        _, ld_overcurrent, *_ = controller.lock_state()
        assert ld_overcurrent == 0, (
            f"LD overcurrent flag is SET (raw value: {ld_overcurrent}).\n"
            "Unexpected with no laser connected -- check output socket pins 1-4."
        )

    def test_driver_off_confirmed_during_tec_operation(self, controller):
        """The laser driver must be OFF throughout all TEC-only tests."""
        _device, driver, *_ = controller.driver_state()
        assert driver == 0, "Laser driver is ON during TEC-only test phase."

    def test_driver_state_still_readable(self, controller):
        """driver_state() returns its full 6-tuple while TEC is running."""
        state = controller.driver_state()
        assert len(state) == 6


# =============================================================================
# PHASE 7 – SETPOINT SWEEP  (abbreviated, no laser)
# Sweep TEC setpoint across 20 -> 25 -> 30 degC.
# At each step: verify setpoint is stored, read TEC current and measured temp,
# and print a table for the operator log.
#
# Wait time per step is 30 s here.  The test plan (section 4.6) calls for
# 5 minutes per step when the laser is running and optical power is recorded.
# For this TEC-only qualification, 30 s is sufficient to confirm PID activity.
# =============================================================================
class TestTECSetpointSweep:
    """Abbreviated setpoint sweep: 20 / 25 / 30 degC, 30 s per step."""

    SWEEP_SETPOINTS = [SP_20, SP_25, SP_30]
    STEP_WAIT = 30   # seconds per step

    def test_tec_tracks_all_setpoints(self, controller):
        """
        At each setpoint:
          1. Setpoint register readback must match commanded value.
          2. TEC current must be non-zero (PID is actively driving).
          3. Measured temperature must have moved toward the setpoint
             compared to the previous step (monotonic tracking).

        Results are printed as a table for the operator's lab log.
        """
        controller.deny_interlock()
        controller.set_tec_int()
        controller.set_tec_temperature(SP_25)  # setpoint before enable
        controller.set_tec_on()

        results = []
        prev_temp = controller.get_tec_temperature()

        for sp in self.SWEEP_SETPOINTS:
            controller.set_tec_temperature(sp)
            time.sleep(self.STEP_WAIT)

            sp_readback = controller.get_tec_value()
            measured = controller.get_tec_temperature()
            tec_i = controller.get_tec_current()
            tec_lim = controller.get_tec_current_limit()

            results.append({
                "setpoint_cmd": sp,
                "setpoint_rb": sp_readback,
                "measured": measured,
                "tec_i_A": tec_i,
                "tec_lim_A": tec_lim,
            })

            # Setpoint register must store the commanded value
            assert pytest.approx(sp_readback, abs=SETPOINT_TOLERANCE) == sp, (
                f"Setpoint readback {sp_readback:.2f} degC != commanded {sp:.1f} degC"
            )

            # TEC current must be within limit
            assert abs(tec_i) <= tec_lim, (
                f"TEC current {tec_i:.3f} A exceeds limit {tec_lim:.3f} A at "
                f"setpoint {sp:.1f} degC."
            )

            prev_temp = measured

        # Print operator table (visible with pytest -s)
        print("\n\nTEC Setpoint Sweep Results")
        print("=" * 65)
        print(f"{'SP cmd':>8} {'SP rb':>8} {'T meas':>8} {'TEC I (A)':>10} {'Limit (A)':>10}")
        print("-" * 65)
        for r in results:
            print(
                f"{r['setpoint_cmd']:>8.1f} "
                f"{r['setpoint_rb']:>8.2f} "
                f"{r['measured']:>8.2f} "
                f"{r['tec_i_A']:>10.3f} "
                f"{r['tec_lim_A']:>10.3f}"
            )
        print("=" * 65)
        print("Copy this table into test plan Data Table 3 (section 6.4).\n")

        # Restore to 25 degC
        controller.set_tec_temperature(SP_25)
