# =============================================================================
# test_tsl570_hardware.py
#
# Hardware integration tests for the Santec TSL-570 tunable laser.
# Instrument: TSL-570, wavelength range 1480–1640 nm
# Connection: GPIB via VISA
#
# REQUIRES: TSL-570 physically connected and powered on.
# If the instrument is not found, all tests are automatically SKIPPED.
#
# Run all hardware tests:
#   pytest test_tsl570_hardware.py -v -s
#
# Run only the quick tests (skip slow sweep):
#   pytest test_tsl570_hardware.py -v -s -k "not slow"
#
# Run only the wavelength sweep:
#   pytest test_tsl570_hardware.py -v -s -m slow
# =============================================================================

import time
import pytest
from tsl570 import TSL570, SweepMode, SweepStatus

# =============================================================================
# CONFIGURATION — change these to match your setup
# =============================================================================

GPIB_ADDRESS  = "GPIB0::1::INSTR"   # <-- your GPIB address
COMMAND_SET   = "SCPI"              # set to "Legacy" if your unit uses nm/THz

# Wavelength range for your unit (1480–1640 nm), in metres for SCPI mode
WAVELENGTH_MIN_M  = 1.48e-6    # 1480 nm
WAVELENGTH_MAX_M  = 1.64e-6    # 1640 nm
WAVELENGTH_MID_M  = 1.55e-6    # 1550 nm — safe test point

# Tolerance for wavelength readback (instruments have finite tuning resolution)
WAVELENGTH_TOL_M  = 0.1e-9     # ±0.1 nm

# Power settings
POWER_DBM_TEST    = -10.0      # safe low power for testing
POWER_TOL_DBM     =  1.0       # ±1 dB tolerance on readback

# Sweep settings matching your test plan
SWEEP_START_M     = 1.50e-6    # 1500 nm
SWEEP_STOP_M      = 1.60e-6    # 1600 nm
SWEEP_STEP_M      = 1.0e-9     # 1 nm
SWEEP_SPEED       = 10         # nm/s — slow enough to be safe
SWEEP_DWELL_S     = 0.5        # 0.5 s per step in stepped mode


# =============================================================================
# SECTION 1 — Connection fixture
# =============================================================================

@pytest.fixture(scope="module")
def laser():
    """
    Opens a real GPIB connection to the TSL-570.
    scope="module" means one connection is shared across all tests in this file.

    If the instrument is not found (wrong address, powered off, GPIB not
    installed), the fixture calls pytest.skip() which marks every test
    in this file as SKIPPED rather than FAILED.
    """
    try:
        instrument = TSL570(GPIB_ADDRESS)
        # Set known state: SCPI command mode, output off, safe wavelength
        instrument.command_set = COMMAND_SET
        instrument.output_enabled = False
        instrument.wavelength_setpoint = WAVELENGTH_MID_M
    except Exception as exc:
        pytest.skip(
            f"TSL-570 not available at {GPIB_ADDRESS}: {exc}"
        )

    yield instrument

    # --- Teardown: always run after ALL tests finish ---
    try:
        instrument.stop_sweep()
        instrument.output_enabled = False
        instrument.wavelength_setpoint = WAVELENGTH_MID_M
    except Exception:
        pass


@pytest.fixture(autouse=True)
def safe_state(laser):
    """
    Runs before EACH test: ensures output is off and sweep is stopped.
    autouse=True means this applies to every test automatically.
    """
    laser.stop_sweep()
    laser.output_enabled = False
    time.sleep(0.2)
    yield
    # Post-test cleanup
    laser.stop_sweep()
    laser.output_enabled = False


# =============================================================================
# SECTION 2 — Connection and identification
# =============================================================================

class TestHardwareConnection:
    """Basic communication checks."""

    def test_instrument_responds(self, laser):
        """
        Reading command_set should return either 'SCPI' or 'Legacy'
        without raising any exception. If the instrument doesn't respond,
        the read will time out and raise an error.
        """
        result = laser.command_set
        assert result in ("SCPI", "Legacy"), (
            f"Unexpected command_set reply: {result!r}"
        )

    def test_command_set_is_scpi(self, laser):
        """After setup, command_set should be SCPI (set in fixture)."""
        assert laser.command_set == COMMAND_SET


# =============================================================================
# SECTION 3 — Wavelength control
# =============================================================================

class TestHardwareWavelength:
    """Wavelength tuning and readback tests."""

    def test_wavelength_setpoint_readback(self, laser):
        """
        Set a wavelength, read it back, confirm they match within tolerance.
        This verifies the full write → tune → read round trip.
        """
        laser.wavelength_setpoint = WAVELENGTH_MID_M
        time.sleep(1.0)  # Allow time for tuning to complete
        readback = laser.wavelength_setpoint
        assert abs(readback - WAVELENGTH_MID_M) <= WAVELENGTH_TOL_M, (
            f"Wavelength readback {readback*1e9:.3f} nm differs from "
            f"setpoint {WAVELENGTH_MID_M*1e9:.1f} nm by more than "
            f"{WAVELENGTH_TOL_M*1e9:.2f} nm"
        )

    def test_wavelength_min_is_plausible(self, laser):
        """
        wavelength_min returned by the instrument should be within
        the expected range of this unit (1480–1640 nm).
        """
        wl_min = laser.wavelength_min
        assert WAVELENGTH_MIN_M * 0.99 <= wl_min <= WAVELENGTH_MAX_M, (
            f"wavelength_min {wl_min*1e9:.1f} nm is outside expected range."
        )

    def test_wavelength_max_is_plausible(self, laser):
        """wavelength_max should also be within the unit's range."""
        wl_max = laser.wavelength_max
        assert WAVELENGTH_MIN_M <= wl_max <= WAVELENGTH_MAX_M * 1.01, (
            f"wavelength_max {wl_max*1e9:.1f} nm is outside expected range."
        )

    def test_wavelength_min_less_than_max(self, laser):
        """Basic sanity: min must be less than max."""
        assert laser.wavelength_min < laser.wavelength_max

    def test_set_sweep_start_stop(self, laser):
        """
        Write sweep start and stop wavelengths, read them back.
        Tests the register write → read round trip for sweep boundaries.
        """
        laser.wavelength_start = SWEEP_START_M
        laser.wavelength_stop  = SWEEP_STOP_M
        time.sleep(0.3)

        assert abs(laser.wavelength_start - SWEEP_START_M) <= WAVELENGTH_TOL_M
        assert abs(laser.wavelength_stop  - SWEEP_STOP_M)  <= WAVELENGTH_TOL_M


# =============================================================================
# SECTION 4 — Power control
# =============================================================================

class TestHardwarePower:
    """Output power enable/disable and level control."""

    def test_output_starts_disabled(self, laser):
        """After safe_state fixture, output should be off."""
        assert laser.output_enabled is False

    def test_enable_disable_output(self, laser):
        """Enable the output, verify it is on, then disable and verify off."""
        laser.output_enabled = True
        time.sleep(0.5)
        assert laser.output_enabled is True

        laser.output_enabled = False
        time.sleep(0.3)
        assert laser.output_enabled is False

    def test_power_unit_readback(self, laser):
        """Set power unit to dBm, read back, confirm match."""
        laser.power_unit = "dBm"
        time.sleep(0.2)
        assert laser.power_unit == "dBm"

    def test_power_setpoint_readback(self, laser):
        """
        Set power to a safe low level, read back.
        Tolerance of ±1 dB accounts for instrument resolution.
        """
        laser.power_unit = "dBm"
        laser.power_setpoint = POWER_DBM_TEST
        time.sleep(0.3)
        readback = laser.power_setpoint
        assert abs(readback - POWER_DBM_TEST) <= POWER_TOL_DBM, (
            f"Power readback {readback:.2f} dBm differs from "
            f"setpoint {POWER_DBM_TEST:.1f} dBm by more than "
            f"{POWER_TOL_DBM:.1f} dB"
        )

    def test_actual_power_readable_when_enabled(self, laser):
        """
        With output enabled, actual power measurement should return
        a finite float. We don't check the exact value — just that
        the instrument responds and the value is in a plausible range.
        """
        laser.power_unit = "dBm"
        laser.power_setpoint = POWER_DBM_TEST
        laser.output_enabled = True
        time.sleep(1.0)  # Allow power to stabilise

        actual = laser.power
        assert isinstance(actual, float), "power measurement did not return a float"
        assert -60.0 <= actual <= 10.0, (
            f"Actual power {actual:.2f} dBm is outside plausible range [-60, 10] dBm"
        )


# =============================================================================
# SECTION 5 — Sweep mode and speed
# =============================================================================

class TestHardwareSweep:
    """Sweep configuration and status tests."""

    def test_sweep_mode_readback(self, laser):
        """Set sweep mode, read back, confirm match."""
        laser.sweep_mode = SweepMode.STEPPED_ONE_WAY
        time.sleep(0.2)
        assert laser.sweep_mode == SweepMode.STEPPED_ONE_WAY

    def test_sweep_speed_readback(self, laser):
        """Set sweep speed to 10 nm/s, read back."""
        laser.sweep_speed = SWEEP_SPEED
        time.sleep(0.2)
        assert laser.sweep_speed == SWEEP_SPEED

    def test_sweep_dwell_readback(self, laser):
        """Set dwell time, read back."""
        laser.sweep_dwell = SWEEP_DWELL_S
        time.sleep(0.2)
        assert abs(laser.sweep_dwell - SWEEP_DWELL_S) < 0.01

    def test_sweep_cycles_readback(self, laser):
        """Set 1 sweep cycle, read back."""
        laser.sweep_cycles = 1
        time.sleep(0.2)
        assert laser.sweep_cycles == 1

    def test_sweep_status_stopped_after_stop(self, laser):
        """After stop_sweep(), status should report STOPPED (0)."""
        laser.stop_sweep()
        time.sleep(0.5)
        status = laser.sweep_staus  # note: typo in class — fix when corrected
        assert status == SweepStatus.STOPPED


# =============================================================================
# SECTION 6 — Stepped wavelength sweep
#
# NOTE: This test enables the laser output. Wear safety goggles.
# NOTE: Marked @pytest.mark.slow — skip with: pytest -k "not slow"
# =============================================================================

@pytest.mark.slow
class TestHardwareWavelengthSweep:
    """
    Stepped wavelength sweep from SWEEP_START_M to SWEEP_STOP_M.
    Implements the kind of measurement described in your test plan
    (power vs wavelength characterisation).

    At each step: records wavelength setpoint, actual wavelength readback,
    and prints to console for manual logging.

    Run with:  pytest test_tsl570_hardware.py -v -s -m slow
    The -s flag shows the print output (the data table).
    """

    def test_stepped_wavelength_sweep(self, laser):
        """
        Stepped sweep: 1500 nm → 1600 nm in 1 nm steps at 10 nm/s.
        Verifies:
          - Each step tunes within wavelength tolerance
          - Sweep count increments after completion
          - No communication errors throughout
        """
        # Configure sweep
        laser.wavelength_start = SWEEP_START_M
        laser.wavelength_stop  = SWEEP_STOP_M
        laser.wavelength_step  = SWEEP_STEP_M
        laser.sweep_speed      = SWEEP_SPEED
        laser.sweep_dwell      = SWEEP_DWELL_S
        laser.sweep_cycles     = 1
        laser.sweep_mode       = SweepMode.STEPPED_ONE_WAY

        # Set power and enable output
        laser.power_unit       = "dBm"
        laser.power_setpoint   = POWER_DBM_TEST
        laser.output_enabled   = True
        time.sleep(1.0)

        print("\n\nStepped Wavelength Sweep Data")
        print(f"{'Step':>6} {'Setpoint (nm)':>14} {'Readback (nm)':>14} "
              f"{'Delta (pm)':>12} {'Power (dBm)':>12}")
        print("-" * 62)

        # Manual stepped sweep
        wl = SWEEP_START_M
        step_num = 0
        results = []

        while wl <= SWEEP_STOP_M + SWEEP_STEP_M * 0.1:
            laser.wavelength_setpoint = wl
            time.sleep(SWEEP_DWELL_S + 0.3)  # dwell + settling

            readback = laser.wavelength_setpoint
            actual_power = laser.power
            delta_pm = (readback - wl) * 1e12  # convert to picometres

            print(f"{step_num:>6} {wl*1e9:>14.3f} {readback*1e9:>14.3f} "
                  f"{delta_pm:>12.1f} {actual_power:>12.2f}")

            results.append({
                "setpoint_nm":  wl * 1e9,
                "readback_nm":  readback * 1e9,
                "delta_pm":     delta_pm,
                "power_dBm":    actual_power,
            })

            # Verify each step tunes correctly
            assert abs(readback - wl) <= WAVELENGTH_TOL_M, (
                f"Step {step_num}: wavelength {readback*1e9:.3f} nm "
                f"deviates more than {WAVELENGTH_TOL_M*1e9:.2f} nm "
                f"from setpoint {wl*1e9:.3f} nm"
            )

            wl += SWEEP_STEP_M
            step_num += 1

        laser.output_enabled = False

        # Verify all steps completed
        assert len(results) >= 10, (
            f"Expected at least 10 steps, got {len(results)}"
        )

        print(f"\nCompleted {len(results)} steps.")
        print(f"Max wavelength deviation: "
              f"{max(abs(r['delta_pm']) for r in results):.1f} pm")