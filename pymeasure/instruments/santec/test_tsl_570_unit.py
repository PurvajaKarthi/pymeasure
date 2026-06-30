# =============================================================================
# test_tsl570_unit.py
#
# Unit tests for the Santec TSL-570 tunable laser class.
# Uses pymeasure's built-in expected_protocol() — no hardware needed.
#
# Run with:   pytest test_tsl570_unit.py -v
#
# NOTE: One typo exists in the TSL570 class as uploaded:
#   "sweep_staus" should be "sweep_status"
#   Tests in Section 5 use the AS-WRITTEN name "sweep_staus" so they
#   pass against the current file. Fix the typo and update that one
#   test name when you do.
#
# File naming: your class file must be named TSL570.py in the same folder.
# If it has a different name, change the import line below.
# =============================================================================

import pytest
from pymeasure.test import expected_protocol
from tsl570 import (
    TSL570,
    SweepMode, SweepPattern, SweepRouting, SweepStatus,
    mode_to_pattern, mode_to_routing, combine_pattern_routing,
)

# =============================================================================
# How expected_protocol works
# -----------------------------------------------------------------------------
# expected_protocol(InstrumentClass, [(sent, reply), ...]) creates a fake
# instrument. Each tuple is one exchange:
#   ("command string", None)       — write only, no reply expected
#   ("command string?", "1.55e-6") — query, fake instrument replies with string
#
# If your code sends a command that is NOT in the list, the test fails.
# If your code sends fewer commands than the list, the test also fails.
# This ensures the class sends exactly the right SCPI commands.
#
# SCPI unit conventions for TSL-570:
#   Wavelength  → metres      1550 nm = 1.55e-6 m
#   Frequency   → Hz          193 THz = 1.93e14 Hz
#   Power       → dBm or mW   (depends on power_unit setting)
# =============================================================================


# =============================================================================
# SECTION 1 — Command set
# =============================================================================

class TestCommandSet:
    """Tests for switching between Legacy and SCPI command sets."""

    def test_set_scpi_mode(self):
        """'SCPI' maps to integer 1 via map_values=True."""
        with expected_protocol(
            TSL570,
            [(":SYSTem:COMMunicate:CODe 1", None)],
        ) as laser:
            laser.command_set = "SCPI"

    def test_set_legacy_mode(self):
        """'Legacy' maps to integer 0."""
        with expected_protocol(
            TSL570,
            [(":SYSTem:COMMunicate:CODe 0", None)],
        ) as laser:
            laser.command_set = "Legacy"

    def test_read_returns_scpi(self):
        """Reply '1' maps back to 'SCPI' string."""
        with expected_protocol(
            TSL570,
            [(":SYSTem:COMMunicate:CODe?", "1")],
        ) as laser:
            assert laser.command_set == "SCPI"

    def test_read_returns_legacy(self):
        """Reply '0' maps back to 'Legacy' string."""
        with expected_protocol(
            TSL570,
            [(":SYSTem:COMMunicate:CODe?", "0")],
        ) as laser:
            assert laser.command_set == "Legacy"

    def test_invalid_command_set_raises(self):
        """Anything other than 'SCPI' or 'Legacy' should raise ValueError."""
        with expected_protocol(TSL570, []) as laser:
            with pytest.raises(ValueError):
                laser.command_set = "VISA"


# =============================================================================
# SECTION 2 — Output power
# =============================================================================

class TestPowerControl:
    """Tests for optical output enable, power unit, setpoint, and measurement."""

    def test_enable_output(self):
        """True maps to 1 via map_values=True."""
        with expected_protocol(
            TSL570,
            [(":POWer:STATe 1", None)],
        ) as laser:
            laser.output_enabled = True

    def test_disable_output(self):
        """False maps to 0."""
        with expected_protocol(
            TSL570,
            [(":POWer:STATe 0", None)],
        ) as laser:
            laser.output_enabled = False

    def test_read_output_enabled_true(self):
        """Reply '1' maps back to True."""
        with expected_protocol(
            TSL570,
            [(":POWer:STATe?", "1")],
        ) as laser:
            assert laser.output_enabled is True

    def test_read_output_enabled_false(self):
        """Reply '0' maps back to False."""
        with expected_protocol(
            TSL570,
            [(":POWer:STATe?", "0")],
        ) as laser:
            assert laser.output_enabled is False

    def test_set_power_unit_dbm(self):
        """'dBm' maps to 0."""
        with expected_protocol(
            TSL570,
            [(":POWer:UNIT 0", None)],
        ) as laser:
            laser.power_unit = "dBm"

    def test_set_power_unit_mw(self):
        """'mW' maps to 1."""
        with expected_protocol(
            TSL570,
            [(":POWer:UNIT 1", None)],
        ) as laser:
            laser.power_unit = "mW"

    def test_read_power_unit_dbm(self):
        """Reply '0' maps back to 'dBm'."""
        with expected_protocol(
            TSL570,
            [(":POWer:UNIT?", "0")],
        ) as laser:
            assert laser.power_unit == "dBm"

    def test_read_power_unit_mw(self):
        """Reply '1' maps back to 'mW'."""
        with expected_protocol(
            TSL570,
            [(":POWer:UNIT?", "1")],
        ) as laser:
            assert laser.power_unit == "mW"

    def test_invalid_power_unit_raises(self):
        """Only 'dBm' or 'mW' accepted — anything else raises ValueError."""
        with expected_protocol(TSL570, []) as laser:
            with pytest.raises(ValueError):
                laser.power_unit = "Watts"

    def test_set_power_setpoint_dbm(self):
        """
        Setting power to -3.0 dBm.
        The %e format produces scientific notation: -3.000000e+00
        """
        with expected_protocol(
            TSL570,
            [(":Power -3.000000e+00", None)],
        ) as laser:
            laser.power_setpoint = -3.0

    def test_read_power_setpoint(self):
        """Reading power setpoint parses the reply string to a float."""
        with expected_protocol(
            TSL570,
            [(":POWer?", "-3.0")],
        ) as laser:
            assert laser.power_setpoint == pytest.approx(-3.0)

    def test_measure_actual_power(self):
        """
        power is a measurement property (read-only).
        Sends ':POWer:ACTual?' and returns parsed float.
        """
        with expected_protocol(
            TSL570,
            [(":POWer:ACTual?", "2.5")],
        ) as laser:
            assert laser.power == pytest.approx(2.5)


# =============================================================================
# SECTION 3 — Wavelength control
#
# Your unit: 1480–1640 nm = 1.48e-6 to 1.64e-6 m in SCPI mode
# =============================================================================

class TestWavelengthControl:
    """Tests for wavelength setpoint, limits, and sweep range."""

    def test_set_wavelength_1550nm(self):
        """1550 nm = 1.55e-6 m → ':WAVelength 1.550000e-06'."""
        with expected_protocol(
            TSL570,
            [(":WAVelength 1.550000e-06", None)],
        ) as laser:
            laser.wavelength_setpoint = 1.55e-6

    def test_set_wavelength_lower_limit(self):
        """1480 nm is the lower limit of this unit."""
        with expected_protocol(
            TSL570,
            [(":WAVelength 1.480000e-06", None)],
        ) as laser:
            laser.wavelength_setpoint = 1.48e-6

    def test_set_wavelength_upper_limit(self):
        """1640 nm is the upper limit of this unit."""
        with expected_protocol(
            TSL570,
            [(":WAVelength 1.640000e-06", None)],
        ) as laser:
            laser.wavelength_setpoint = 1.64e-6

    def test_read_wavelength_setpoint(self):
        """Reply '1.55e-06' should parse to 1.55e-6 float."""
        with expected_protocol(
            TSL570,
            [(":WAVelength?", "1.55e-06")],
        ) as laser:
            assert laser.wavelength_setpoint == pytest.approx(1.55e-6)

    def test_read_wavelength_min(self):
        """wavelength_min is a read-only measurement."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:RANGe:MINimum?", "1.48e-06")],
        ) as laser:
            assert laser.wavelength_min == pytest.approx(1.48e-6)

    def test_read_wavelength_max(self):
        """wavelength_max is a read-only measurement."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:RANGe:MAXimum?", "1.64e-06")],
        ) as laser:
            assert laser.wavelength_max == pytest.approx(1.64e-6)

    def test_set_sweep_start(self):
        """Setting sweep start wavelength to 1500 nm."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:STARt 1.500000e-06", None)],
        ) as laser:
            laser.wavelength_start = 1.5e-6

    def test_set_sweep_stop(self):
        """Setting sweep stop wavelength to 1600 nm."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:STOP 1.600000e-06", None)],
        ) as laser:
            laser.wavelength_stop = 1.6e-6

    def test_set_sweep_step(self):
        """Setting sweep step to 1 nm = 1e-9 m."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:STEP 1.000000e-09", None)],
        ) as laser:
            laser.wavelength_step = 1e-9

    def test_read_sweep_start(self):
        """Reading sweep start wavelength."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:STARt?", "1.5e-06")],
        ) as laser:
            assert laser.wavelength_start == pytest.approx(1.5e-6)

    def test_read_sweep_stop(self):
        """Reading sweep stop wavelength."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:STOP?", "1.6e-06")],
        ) as laser:
            assert laser.wavelength_stop == pytest.approx(1.6e-6)


# =============================================================================
# SECTION 4 — Frequency control
#
# 1480 nm ≈ 202.7 THz = 2.027e14 Hz
# 1640 nm ≈ 182.8 THz = 1.828e14 Hz
# =============================================================================

class TestFrequencyControl:
    """Tests for optical frequency setpoint and sweep range."""

    def test_set_frequency_193THz(self):
        """193 THz = 1.93e14 Hz → ':FREQuency 1.930000e+14'."""
        with expected_protocol(
            TSL570,
            [(":FREQuency 1.930000e+14", None)],
        ) as laser:
            laser.frequency_setpoint = 1.93e14

    def test_read_frequency_setpoint(self):
        """Reply '1.93e+14' parses to 1.93e14 float."""
        with expected_protocol(
            TSL570,
            [(":FREQuency?", "1.93e+14")],
        ) as laser:
            assert laser.frequency_setpoint == pytest.approx(1.93e14)

    def test_read_frequency_min(self):
        """Lower wavelength limit (1640 nm) corresponds to min frequency."""
        with expected_protocol(
            TSL570,
            [(":FREQuency:SWEep:RANGe:MINimum?", "1.828e+14")],
        ) as laser:
            assert laser.frequency_min == pytest.approx(1.828e14)

    def test_read_frequency_max(self):
        """Upper wavelength limit (1480 nm) corresponds to max frequency."""
        with expected_protocol(
            TSL570,
            [(":FREQuency:SWEep:RANGe:MAXimum?", "2.027e+14")],
        ) as laser:
            assert laser.frequency_max == pytest.approx(2.027e14)

    def test_set_frequency_start(self):
        """Setting frequency sweep start."""
        with expected_protocol(
            TSL570,
            [(":FREQuency:SWEep:STARt 1.930000e+14", None)],
        ) as laser:
            laser.frequency_start = 1.93e14

    def test_set_frequency_stop(self):
        """Setting frequency sweep stop."""
        with expected_protocol(
            TSL570,
            [(":FREQuency:SWEep:STOP 1.940000e+14", None)],
        ) as laser:
            laser.frequency_stop = 1.94e14

    def test_set_frequency_step(self):
        """Setting sweep step to 10 GHz = 1e10 Hz."""
        with expected_protocol(
            TSL570,
            [(":FREQuency:SWEep:STEP 1.000000e+10", None)],
        ) as laser:
            laser.frequency_step = 1e10


# =============================================================================
# SECTION 5 — Sweep control
# =============================================================================

class TestSweepControl:
    """Tests for sweep start/stop, mode, speed, dwell, delay, and cycles."""

    def test_start_sweep(self):
        """start_sweep() always writes ':WAVelength:SWEep 1'."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep 1", None)],
        ) as laser:
            laser.start_sweep()

    def test_stop_sweep(self):
        """stop_sweep() always writes ':WAVelength:SWEep 0'."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep 0", None)],
        ) as laser:
            laser.stop_sweep()

    def test_start_repeat(self):
        """start_repeat() writes ':WAVelength:SWEep:REPeat'."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:REPeat", None)],
        ) as laser:
            laser.start_repeat()

    def test_read_sweep_status_stopped(self):
        """
        NOTE: property is spelled 'sweep_staus' (typo) in the current class file.
        Reply '0' → SweepStatus.STOPPED via get_process lambda.
        Update this test name when the typo is fixed to 'sweep_status'.
        """
        with expected_protocol(
            TSL570,
            [("WAVelength:SWEep?", "0")],
        ) as laser:
            assert laser.sweep_staus == SweepStatus.STOPPED

    def test_read_sweep_status_running(self):
        """Reply '1' → SweepStatus.RUNNING."""
        with expected_protocol(
            TSL570,
            [("WAVelength:SWEep?", "1")],
        ) as laser:
            assert laser.sweep_staus == SweepStatus.RUNNING

    def test_read_sweep_status_standing_by(self):
        """Reply '3' → SweepStatus.STANDING_BY_TRIGGER."""
        with expected_protocol(
            TSL570,
            [("WAVelength:SWEep?", "3")],
        ) as laser:
            assert laser.sweep_staus == SweepStatus.STANDING_BY_TRIGGER

    def test_set_sweep_mode_stepped_one_way(self):
        """SweepMode.STEPPED_ONE_WAY (value 0) → ':WAVelength:SWEep:MODe 0'."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:MODe 0", None)],
        ) as laser:
            laser.sweep_mode = SweepMode.STEPPED_ONE_WAY

    def test_set_sweep_mode_continuous_two_way(self):
        """SweepMode.CONTINUOUS_TWO_WAY (value 3) → ':WAVelength:SWEep:MODe 3'."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:MODe 3", None)],
        ) as laser:
            laser.sweep_mode = SweepMode.CONTINUOUS_TWO_WAY

    def test_read_sweep_mode(self):
        """Reply '2' → SweepMode.STEPPED_TWO_WAY."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:MODe?", "2")],
        ) as laser:
            assert laser.sweep_mode == SweepMode.STEPPED_TWO_WAY

    def test_sweep_pattern_reads_mode_first(self):
        """
        sweep_pattern is computed from sweep_mode.
        Mode 0 → STEPPED pattern.
        One read of sweep_mode register is expected.
        """
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:MODe?", "0")],
        ) as laser:
            assert laser.sweep_pattern == SweepPattern.STEPPED

    def test_sweep_pattern_continuous(self):
        """Mode 1 → CONTINUOUS pattern."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:MODe?", "1")],
        ) as laser:
            assert laser.sweep_pattern == SweepPattern.CONTINUOUS

    def test_sweep_routing_one_way(self):
        """Mode 0 → ONE_WAY routing."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:MODe?", "0")],
        ) as laser:
            assert laser.sweep_routing == SweepRouting.ONE_WAY

    def test_sweep_routing_two_way(self):
        """Mode 2 → TWO_WAY routing."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:MODe?", "2")],
        ) as laser:
            assert laser.sweep_routing == SweepRouting.TWO_WAY

    def test_set_sweep_pattern_reads_then_writes(self):
        """
        Setting sweep_pattern:
          1. Reads current mode to preserve routing component
          2. Writes new combined mode
        Current mode 2 (STEPPED_TWO_WAY) + CONTINUOUS pattern → mode 3
        """
        with expected_protocol(
            TSL570,
            [
                (":WAVelength:SWEep:MODe?", "2"),
                (":WAVelength:SWEep:MODe 3", None),
            ],
        ) as laser:
            laser.sweep_pattern = SweepPattern.CONTINUOUS

    def test_set_sweep_routing_reads_then_writes(self):
        """
        Setting sweep_routing:
          1. Reads current mode to preserve pattern component
          2. Writes new combined mode
        Current mode 1 (CONTINUOUS_ONE_WAY) + TWO_WAY routing → mode 3
        """
        with expected_protocol(
            TSL570,
            [
                (":WAVelength:SWEep:MODe?", "1"),
                (":WAVelength:SWEep:MODe 3", None),
            ],
        ) as laser:
            laser.sweep_routing = SweepRouting.TWO_WAY

    def test_set_sweep_speed_valid(self):
        """10 nm/s is a valid discrete speed value."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:SPEed 10", None)],
        ) as laser:
            laser.sweep_speed = 10

    def test_read_sweep_speed(self):
        """Reply '50' returns integer 50."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:SPEed?", "50")],
        ) as laser:
            assert laser.sweep_speed == 50

    def test_invalid_sweep_speed_raises(self):
        """15 nm/s is not in {1,2,5,10,20,50,100,200} → ValueError."""
        with expected_protocol(TSL570, []) as laser:
            with pytest.raises(ValueError):
                laser.sweep_speed = 15

    def test_set_sweep_dwell(self):
        """0.5 s dwell time, within range [0, 999.9]."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:DWELl 0.5", None)],
        ) as laser:
            laser.sweep_dwell = 0.5

    def test_read_sweep_dwell(self):
        """Reply '0.5' parses to 0.5 float."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:DWELl?", "0.5")],
        ) as laser:
            assert laser.sweep_dwell == pytest.approx(0.5)

    def test_invalid_sweep_dwell_above_max(self):
        """1000 s exceeds max of 999.9 → ValueError."""
        with expected_protocol(TSL570, []) as laser:
            with pytest.raises(ValueError):
                laser.sweep_dwell = 1000.0

    def test_invalid_sweep_dwell_negative(self):
        """Negative dwell time → ValueError."""
        with expected_protocol(TSL570, []) as laser:
            with pytest.raises(ValueError):
                laser.sweep_dwell = -1.0

    def test_set_sweep_delay(self):
        """1.0 s delay between consecutive scans."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:DELay 1.0", None)],
        ) as laser:
            laser.sweep_delay = 1.0

    def test_set_sweep_cycles(self):
        """5 sweep repetitions."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:CYCLes 5", None)],
        ) as laser:
            laser.sweep_cycles = 5

    def test_read_sweep_cycles(self):
        """Reply '5' returns 5."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:CYCLes?", "5")],
        ) as laser:
            assert laser.sweep_cycles == 5

    def test_invalid_sweep_cycles_above_max(self):
        """1000 exceeds max of 999 → ValueError."""
        with expected_protocol(TSL570, []) as laser:
            with pytest.raises(ValueError):
                laser.sweep_cycles = 1000

    def test_read_sweep_count(self):
        """sweep_count is read-only. Reply '3' returns 3."""
        with expected_protocol(
            TSL570,
            [(":WAVelength:SWEep:COUNt?", "3")],
        ) as laser:
            assert laser.sweep_count == pytest.approx(3)


# =============================================================================
# SECTION 6 — Enum helper functions (pure Python, zero instrument interaction)
# =============================================================================

class TestEnumHelpers:
    """Tests for mode_to_pattern, mode_to_routing, combine_pattern_routing."""

    def test_mode_0_stepped_one_way(self):
        assert mode_to_pattern(SweepMode.STEPPED_ONE_WAY) == SweepPattern.STEPPED
        assert mode_to_routing(SweepMode.STEPPED_ONE_WAY) == SweepRouting.ONE_WAY

    def test_mode_1_continuous_one_way(self):
        assert mode_to_pattern(SweepMode.CONTINUOUS_ONE_WAY) == SweepPattern.CONTINUOUS
        assert mode_to_routing(SweepMode.CONTINUOUS_ONE_WAY) == SweepRouting.ONE_WAY

    def test_mode_2_stepped_two_way(self):
        assert mode_to_pattern(SweepMode.STEPPED_TWO_WAY) == SweepPattern.STEPPED
        assert mode_to_routing(SweepMode.STEPPED_TWO_WAY) == SweepRouting.TWO_WAY

    def test_mode_3_continuous_two_way(self):
        assert mode_to_pattern(SweepMode.CONTINUOUS_TWO_WAY) == SweepPattern.CONTINUOUS
        assert mode_to_routing(SweepMode.CONTINUOUS_TWO_WAY) == SweepRouting.TWO_WAY

    def test_combine_stepped_one_way(self):
        result = combine_pattern_routing(SweepPattern.STEPPED, SweepRouting.ONE_WAY)
        assert result == SweepMode.STEPPED_ONE_WAY

    def test_combine_continuous_two_way(self):
        result = combine_pattern_routing(SweepPattern.CONTINUOUS, SweepRouting.TWO_WAY)
        assert result == SweepMode.CONTINUOUS_TWO_WAY

    def test_all_modes_roundtrip(self):
        """
        Every SweepMode should survive decompose → recompose unchanged.
        This catches any error in the combine/split maths across all 4 modes.
        """
        for mode in SweepMode:
            pattern = mode_to_pattern(mode)
            routing = mode_to_routing(mode)
            assert combine_pattern_routing(pattern, routing) == mode, (
                f"Round-trip failed for {mode}"
            )