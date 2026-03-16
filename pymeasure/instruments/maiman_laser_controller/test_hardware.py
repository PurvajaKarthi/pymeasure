# =============================================================================
# test_hardware.py
#
# Hardware integration tests for the MaimanSF8XXX / SF8025-NM laser driver.
# Based on: PTB PIC Assembly Laser Test Plan Rev 0.0.1, Skylark CubeSat Mission
# DUT: QPhotonics QDFBLD-1550-2SM, 1550 nm DFB fiber-coupled laser diode
#
# REQUIRES: SF8025-NM physically connected via USB-to-RS485 adapter.
#
# If the device is NOT connected, every test in this file is automatically
# SKIPPED — not failed. You will see "s" in the output instead of "F".
#
# Run with:   pytest test_hardware.py -v
# Run only hardware tests: pytest test_hardware.py -v -m hardware
# =============================================================================

import time
import pytest
from Maiman import MaimanSF8XXX

# =============================================================================
# CONFIGURATION — change these to match your setup
# =============================================================================

COM_PORT      = "COM3"       # <-- Change to your actual COM port (e.g. "COM6")
                             #     Find it in: Device Manager → Ports (COM & LPT)
SLAVE_ADDRESS = 1            # <-- Modbus slave address of your SF8025-NM
BAUDRATE      = 115200       # Leave this unchanged
TIMEOUT       = 0.5          # Seconds — increase if you see timeout errors

# --- Test plan limits (from PTB Test Plan Rev 0.0.1) ---
MAX_FORWARD_CURRENT_MA = 38.0    # Absolute maximum for QDFBLD-1550-2SM — never exceed
THRESHOLD_CURRENT_MA   = 11.0    # Expected lasing threshold (datasheet typ.)
TEC_NOMINAL_TEMP_C     = 25.0    # Standard operating temperature
TEC_SWEEP_MIN_C        = 15.0    # Lower bound of temperature sweep
TEC_SWEEP_MAX_C        = 40.0    # Upper bound of temperature sweep
TEC_STABILITY_TOLERANCE_C = 0.1  # ±0.1°C stability criterion (test plan Sec 3.3)
TEC_STABILITY_WAIT_S   = 300     # 5 minutes wait per test plan Sec 4.6

# Forward voltage spec from test plan Table 7 (QDFBLD-1550-2SM datasheet)
VOLTAGE_MIN_V = 0.9    # Allow slight margin below datasheet 1.0 V
VOLTAGE_MAX_V = 1.5    # Allow slight margin above datasheet 1.4 V

# How long to wait between setting a current and reading back (test plan: 10 s)
CURRENT_STEP_WAIT_S = 10

# Current sweep parameters (test plan Sec 4.5)
CURRENT_SWEEP_START_MA = 0.0
CURRENT_SWEEP_END_MA   = 38.0
CURRENT_SWEEP_STEP_MA  = 2.0


# =============================================================================
# SECTION 1 — Skip logic
#
# This fixture tries to connect to the real device. If it fails (e.g. the
# device is not plugged in, or the COM port is wrong), it raises
# pytest.skip() which marks the test as skipped — not failed.
#
# All hardware tests use this fixture, so they all skip automatically
# when hardware is absent. You never need to comment tests out manually.
# =============================================================================

@pytest.fixture(scope="module")
def hw_laser():
    """
    Opens a real connection to the SF8025-NM.

    scope="module" means one connection is shared across ALL tests in this
    file — we don't open and close the COM port for every single test.
    This is faster and mirrors how you'd use the device in an experiment.
    """
    try:
        device = MaimanSF8XXX(
            COM_PORT,
            slave_address=SLAVE_ADDRESS,
            baudrate=BAUDRATE,
            timeout=TIMEOUT,
        )
        device.open()

        # Quick sanity check — if this read fails, the device isn't responding.
        _ = device.device_id

    except Exception as exc:
        pytest.skip(
            f"Hardware not available ({COM_PORT}, slave={SLAVE_ADDRESS}): {exc}"
        )

    yield device

    # --- Teardown: runs after ALL tests in this module finish ---
    # Always leave the laser in a safe state.
    try:
        device.disable()
        device.current_setpoint = 0.0
    except Exception:
        pass  # If we can't communicate, just close the port.
    finally:
        device.close()


# =============================================================================
# SECTION 2 — Safety fixture
#
# This runs before EACH individual test and ensures the laser is off.
# If a previous test left the laser enabled, this cleans it up.
# =============================================================================

@pytest.fixture(autouse=True)
def ensure_laser_off(hw_laser):
    """
    Before each test: disable the laser and wait briefly.
    After each test: disable again as a safety net.

    autouse=True means this runs automatically for every test in this file
    without needing to be listed as an argument.
    """
    hw_laser.disable()
    time.sleep(0.2)
    yield
    # Post-test cleanup
    hw_laser.disable()
    hw_laser.current_setpoint = 0.0


# =============================================================================
# SECTION 3 — Connection and identification tests
# =============================================================================

class TestHardwareConnection:
    """Verify basic communication with the SF8025-NM."""

    def test_device_responds(self, hw_laser):
        """The device should respond to a register read without raising an error."""
        device_id = hw_laser.device_id
        # Device ID should be a non-negative integer
        assert isinstance(device_id, int)
        assert device_id >= 0

    def test_serial_number_readable(self, hw_laser):
        """Serial number register should return a valid integer."""
        sn = hw_laser.serial_number
        assert isinstance(sn, int)
        assert sn >= 0

    def test_is_open_true(self, hw_laser):
        """Connection should be open."""
        assert hw_laser.is_open is True


# =============================================================================
# SECTION 4 — TEC temperature tests
#
# The test plan (Sec 3.3) requires TEC to be stable at 25°C ±0.1°C
# before any laser measurements begin.
# =============================================================================

class TestHardwareTEC:
    """TEC temperature control tests."""

    def test_tec_measured_temperature_is_plausible(self, hw_laser):
        """
        The TEC-measured temperature should be a physically plausible value.
        We allow a wide range (0°C to 60°C) to account for ambient conditions.
        If you read −200°C or +500°C, something is wrong with the thermistor.
        """
        temp = hw_laser.tec_temperature_measured
        assert 0.0 <= temp <= 60.0, (
            f"TEC temperature {temp:.1f}°C is outside plausible range. "
            f"Check thermistor connection."
        )

    def test_tec_setpoint_readback(self, hw_laser):
        """
        Write 25°C setpoint, read it back, confirm they match.
        This verifies the write→read round trip works correctly.
        """
        hw_laser.tec_temperature_setpoint = TEC_NOMINAL_TEMP_C
        time.sleep(0.5)  # Brief settling time for register to update
        readback = hw_laser.tec_temperature_setpoint
        assert readback == pytest.approx(TEC_NOMINAL_TEMP_C, abs=0.5), (
            f"Setpoint readback {readback:.1f}°C doesn't match written {TEC_NOMINAL_TEMP_C}°C"
        )

    def test_tec_stable_at_25C(self, hw_laser):
        """
        Per test plan Sec 3.2: TEC must be stable at 25°C ±0.1°C for
        5 minutes before measurements begin.

        NOTE: This test takes up to 5 minutes to run.
        Skip it with: pytest test_hardware.py -k "not tec_stable"
        """
        hw_laser.tec_temperature_setpoint = TEC_NOMINAL_TEMP_C

        stable = hw_laser.wait_for_tec_stable(
            tolerance_C=TEC_STABILITY_TOLERANCE_C,
            timeout_s=TEC_STABILITY_WAIT_S,
            poll_s=10.0,
        )
        assert stable, (
            f"TEC did not stabilize at {TEC_NOMINAL_TEMP_C}°C ±"
            f"{TEC_STABILITY_TOLERANCE_C}°C within {TEC_STABILITY_WAIT_S}s. "
            f"Last reading: {hw_laser.tec_temperature_measured:.2f}°C"
        )

    def test_tec_current_readable(self, hw_laser):
        """TEC drive current should be a finite float (positive or negative)."""
        current = hw_laser.tec_current_measured
        assert isinstance(current, float)
        # Sanity check — TEC current for a small diode won't exceed ±5 A
        assert -5.0 <= current <= 5.0, (
            f"TEC current {current:.2f} A is outside expected range."
        )


# =============================================================================
# SECTION 5 — Current setpoint round-trip tests
#
# Write a value, read it back, confirm they match.
# All at 0 mA drive (laser not enabled) for safety.
# =============================================================================

class TestHardwareCurrentSetpoint:
    """Write and read-back current setpoint register tests."""

    def test_set_zero_current(self, hw_laser):
        """Setting 0 mA and reading back should return 0.0 mA."""
        hw_laser.current_setpoint = 0.0
        time.sleep(0.2)
        assert hw_laser.current_setpoint == pytest.approx(0.0, abs=0.2)

    def test_set_10mA_readback(self, hw_laser):
        """
        Set 10 mA (just below threshold), read back.
        Tolerance of ±0.5 mA accounts for register resolution (0.1 mA steps).
        """
        hw_laser.current_setpoint = 10.0
        time.sleep(0.2)
        assert hw_laser.current_setpoint == pytest.approx(10.0, abs=0.5)

    def test_set_max_operating_current(self, hw_laser):
        """
        Set 38 mA (DUT maximum per test plan Table 7), read back.
        We do NOT enable the laser here — just verify the register accepts the value.
        """
        hw_laser.current_setpoint = MAX_FORWARD_CURRENT_MA
        time.sleep(0.2)
        assert hw_laser.current_setpoint == pytest.approx(MAX_FORWARD_CURRENT_MA, abs=0.5)

    def test_over_limit_rejected(self, hw_laser):
        """
        39 mA exceeds the DUT absolute maximum of 38 mA.
        The class must reject this BEFORE sending anything to the device.
        """
        with pytest.raises(ValueError):
            hw_laser.current_setpoint = 39.0

    def test_current_protection_threshold_readable(self, hw_laser):
        """
        The protection threshold should be readable and ≤38 mA.
        Per test plan Sec 3.3: "Set the current protection threshold to ≤38 mA
        before connecting the DUT."
        """
        threshold = hw_laser.current_protection_threshold
        assert threshold <= MAX_FORWARD_CURRENT_MA, (
            f"Protection threshold {threshold:.1f} mA exceeds DUT max "
            f"{MAX_FORWARD_CURRENT_MA} mA — DANGER: reconfigure before testing."
        )


# =============================================================================
# SECTION 6 — Enable / disable tests
#
# WARNING: These tests enable the laser diode.
# Ensure laser safety goggles are worn and the optical setup is safe
# before running these tests.
# =============================================================================

@pytest.mark.hardware
class TestHardwareEnableDisable:
    """
    Tests that enable/disable commands work correctly on real hardware.
    The @pytest.mark.hardware marker lets you run ONLY these with:
        pytest test_hardware.py -m hardware
    """

    def test_device_starts_stopped(self, hw_laser):
        """After connecting, the device should not be running."""
        assert hw_laser.is_running is False

    def test_enable_sets_running_bit(self, hw_laser):
        """
        After calling enable(), is_running should become True.
        We set a safe minimal current first (0 mA — no light, just verifying state).
        """
        hw_laser.current_setpoint = 0.0
        hw_laser.enable()
        time.sleep(0.5)
        assert hw_laser.is_running is True

    def test_disable_clears_running_bit(self, hw_laser):
        """After calling disable(), is_running should become False."""
        hw_laser.current_setpoint = 0.0
        hw_laser.enable()
        time.sleep(0.5)
        hw_laser.disable()
        time.sleep(0.5)
        assert hw_laser.is_running is False

    def test_interlock_state_readable(self, hw_laser):
        """
        Per test plan Sec 3.3: interlock pin (Analogue Pin 15) must be
        shorted to GND. If is_interlock_denied is True, the pin is open
        and the laser cannot fire — stop and fix the wiring.
        """
        interlock_denied = hw_laser.is_interlock_denied
        assert interlock_denied is False, (
            "INTERLOCK IS OPEN. Short Pin 15 (Analogue connector) to GND "
            "before attempting to enable the laser."
        )


# =============================================================================
# SECTION 7 — Procedure 1: P-I-V current sweep
#
# Implements test plan Sec 5.2:
# "Combined P-I-V and Internal PD Current Sweep"
#
# Drive current 0 → 38 mA in 2 mA steps, 10 s stabilisation each step.
# Records: current_setpoint, voltage_measured, current_measured.
# (Optical power and PD photocurrent require external instruments
#  not readable via Modbus — those are recorded manually.)
#
# NOTE: This test enables the laser. Wear safety goggles.
# NOTE: This test takes approximately 3.5 minutes to complete.
# =============================================================================

@pytest.mark.hardware
class TestCurrentSweep:
    """P-I-V sweep per test plan Procedure 1 (Sec 5.2)."""

    def test_piv_current_sweep(self, hw_laser):
        """
        Sweep current from 0 to 38 mA in 2 mA steps.
        At each step, verify:
          - Measured current is within 1 mA of setpoint
          - Forward voltage is within datasheet range once above threshold
          - No exceptions (communication errors) occur

        Data is printed to console for manual recording into test plan Table 18.
        Run with: pytest test_hardware.py -v -s   (the -s shows print output)
        """
        hw_laser.tec_temperature_setpoint = TEC_NOMINAL_TEMP_C

        # Wait for TEC stability before enabling laser (test plan Sec 5.2 step 1)
        stable = hw_laser.wait_for_tec_stable(
            tolerance_C=TEC_STABILITY_TOLERANCE_C,
            timeout_s=TEC_STABILITY_WAIT_S,
            poll_s=10.0,
        )
        assert stable, "TEC not stable — cannot proceed with sweep."

        hw_laser.current_setpoint = 0.0
        hw_laser.enable()
        time.sleep(1.0)

        print("\n\nData Table 1: P-I-V Sweep at 25°C")
        print(f"{'Current set (mA)':>18} {'Current meas (mA)':>18} "
              f"{'Voltage (V)':>12} {'TEC temp (°C)':>14}")
        print("-" * 68)

        current_ma = CURRENT_SWEEP_START_MA
        results = []

        while current_ma <= CURRENT_SWEEP_END_MA + 0.01:
            hw_laser.current_setpoint = current_ma

            # 10 seconds stabilisation per test plan Sec 4.5
            time.sleep(CURRENT_STEP_WAIT_S)

            measured_i  = hw_laser.current_measured
            measured_v  = hw_laser.voltage_measured
            measured_t  = hw_laser.tec_temperature_measured

            print(f"{current_ma:>18.1f} {measured_i:>18.2f} "
                  f"{measured_v:>12.3f} {measured_t:>14.2f}")

            results.append({
                "set_mA":      current_ma,
                "meas_mA":     measured_i,
                "voltage_V":   measured_v,
                "tec_temp_C":  measured_t,
            })

            # --- Assertions at each step ---

            # Measured current should be within 1 mA of setpoint
            assert abs(measured_i - current_ma) <= 1.0, (
                f"At {current_ma} mA set: measured {measured_i:.2f} mA "
                f"(deviation > 1 mA)"
            )

            # Above threshold, voltage should be in datasheet range
            if current_ma >= THRESHOLD_CURRENT_MA:
                assert VOLTAGE_MIN_V <= measured_v <= VOLTAGE_MAX_V, (
                    f"At {current_ma} mA: voltage {measured_v:.3f} V is outside "
                    f"[{VOLTAGE_MIN_V}, {VOLTAGE_MAX_V}] V spec."
                )

            # TEC temperature must remain stable during sweep
            assert abs(measured_t - TEC_NOMINAL_TEMP_C) <= 0.5, (
                f"TEC temperature drifted to {measured_t:.2f}°C during sweep."
            )

            current_ma += CURRENT_SWEEP_STEP_MA

        # Final safety: set back to 0 and disable
        hw_laser.current_setpoint = 0.0
        hw_laser.disable()

        # Verify threshold was observable (voltage rises above 0.5 V above threshold)
        above_threshold = [r for r in results if r["set_mA"] >= THRESHOLD_CURRENT_MA]
        assert any(r["voltage_V"] > 0.5 for r in above_threshold), (
            "No voltage response observed above threshold current — "
            "check laser diode connections."
        )


# =============================================================================
# SECTION 8 — Procedure 2: Power vs. Temperature sweep
#
# Implements test plan Sec 5.3.
# Fixed current: 38 mA. TEC swept 15°C → 40°C in 5°C steps, 5 min each.
#
# NOTE: This test takes approximately 35 minutes to complete.
# NOTE: Enables laser at 38 mA. Wear safety goggles.
# =============================================================================

@pytest.mark.hardware
@pytest.mark.slow
class TestTemperatureSweep:
    """Power vs. temperature sweep per test plan Procedure 2 (Sec 5.3)."""

    def test_power_vs_temperature_sweep(self, hw_laser):
        """
        Sweep TEC temperature 15°C → 40°C in 5°C steps at 38 mA drive current.
        At each step, verify:
          - TEC actually reaches the setpoint within ±0.1°C
          - TEC current reading is finite and within expected range
          - Communication remains stable throughout

        Run with: pytest test_hardware.py -v -s -m slow
        """
        # Set drive current but don't enable yet
        hw_laser.current_setpoint = MAX_FORWARD_CURRENT_MA

        # Start at lower temperature bound
        hw_laser.tec_temperature_setpoint = TEC_SWEEP_MIN_C

        print("\n\nData Table 3: Power vs. TEC Temperature at 38 mA")
        print(f"{'Setpoint (°C)':>14} {'Therm. (°C)':>12} "
              f"{'TEC current (A)':>16} {'Notes':>10}")
        print("-" * 58)

        temp_C = TEC_SWEEP_MIN_C

        while temp_C <= TEC_SWEEP_MAX_C + 0.1:
            hw_laser.tec_temperature_setpoint = temp_C

            # Wait 5 minutes per step (test plan Sec 4.6)
            stable = hw_laser.wait_for_tec_stable(
                tolerance_C=TEC_STABILITY_TOLERANCE_C,
                timeout_s=TEC_STABILITY_WAIT_S,
                poll_s=30.0,
            )

            measured_t   = hw_laser.tec_temperature_measured
            tec_current  = hw_laser.tec_current_measured

            note = "STABLE" if stable else "NOT STABLE"
            print(f"{temp_C:>14.1f} {measured_t:>12.2f} "
                  f"{tec_current:>16.3f} {note:>10}")

            # TEC must stabilise within tolerance
            assert stable, (
                f"TEC did not stabilise at {temp_C}°C within "
                f"{TEC_STABILITY_WAIT_S}s. Last reading: {measured_t:.2f}°C"
            )

            # TEC current should be finite and bounded
            assert -5.0 <= tec_current <= 5.0, (
                f"TEC current {tec_current:.3f} A out of range at {temp_C}°C"
            )

            temp_C += 5.0

        # Return to nominal temperature
        hw_laser.tec_temperature_setpoint = TEC_NOMINAL_TEMP_C
        hw_laser.disable()
        hw_laser.current_setpoint = 0.0
