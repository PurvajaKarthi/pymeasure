# =============================================================================
# test_unit.py
#
# Unit tests for the MaimanSF8XXX laser diode driver class.
#
# These tests run WITHOUT any hardware connected. They use a "mock" — a fake
# version of minimalmodbus — so no COM port or laser driver is needed.
#
# Run with:   pytest test_unit.py -v
# =============================================================================

# --- What these imports do ---------------------------------------------------
# pytest        : the test framework itself
# unittest.mock : Python's built-in toolkit for creating fake objects (mocks)
# MagicMock     : a fake object that accepts any method call you throw at it
# patch         : a tool that temporarily replaces a real object with a fake one
# pytest.approx : lets us compare floats with a small allowed tolerance,
#                 because 250.0 / 10 * 10 might give 249.9999... in floating point
# -----------------------------------------------------------------------------
import pytest
from unittest.mock import MagicMock, patch, PropertyMock

# We import our class. Python will look for Maiman.py in the same folder.
from Maiman import MaimanSF8XXX, _to_signed_16, _Reg, _Bit

# =============================================================================
# SECTION 1 — A shared "fixture"
#
# A fixture is a function that pytest runs BEFORE each test to set up the
# environment. Think of it like laying out your tools on the bench before you
# start work. The @pytest.fixture decorator tells pytest this is a fixture.
#
# This fixture creates a MaimanSF8XXX object with a fake (mocked) serial port.
# Every test that needs a laser object just asks for "laser" in its arguments
# and pytest automatically runs this fixture and passes the result in.
# =============================================================================

@pytest.fixture
def laser():
    """
    Creates a MaimanSF8XXX instance with a fully mocked minimalmodbus backend.

    The 'with patch(...)' line says: "for the duration of this block, whenever
    anything tries to import or use minimalmodbus.Instrument, give them this
    fake MagicMock object instead."

    We use 'yield' instead of 'return' so that the patch stays active for the
    entire duration of the test. When the test finishes, the 'with' block exits
    and the real minimalmodbus is restored.
    """
    with patch("minimalmodbus.Instrument") as mock_instrument_class:

        # mock_instrument_class is the fake *class*.
        # mock_instrument_class.return_value is the fake *instance* that gets
        # created when the class is called (i.e. what open() creates internally).
        mock_instr = MagicMock()
        mock_instrument_class.return_value = mock_instr

        # Give the fake instrument a fake serial port that appears open.
        mock_instr.serial = MagicMock()
        mock_instr.serial.is_open = True

        # Create the laser object and open it (which calls Instrument() internally).
        device = MaimanSF8XXX("COM3", slave_address=1)
        # ^^^^
        # COM3 is a placeholder. Change this string to match your actual port,
        # e.g. "COM6" or "/dev/ttyUSB0", before running hardware tests.
        # In unit tests it doesn't matter — no real port is opened.

        device.open()

        # 'yield' hands the device to the test. The patch stays active until
        # the test function returns.
        yield device, mock_instr


# =============================================================================
# SECTION 2 — Helper: teach the mock to return a specific register value
#
# read_register is the minimalmodbus method our _read() calls internally.
# We tell the mock: "when read_register is called, return this value."
# =============================================================================

def set_register_return(mock_instr, value):
    """Tell the fake instrument to return 'value' for any register read."""
    mock_instr.read_register.return_value = value


# =============================================================================
# SECTION 3 — Connection tests
# =============================================================================

class TestConnection:
    """Tests that opening and closing the connection work correctly."""

    def test_open_creates_instrument(self):
        """
        When open() is called, it should create a minimalmodbus.Instrument.
        We verify the mock class was actually called (i.e. Instrument() ran).
        """
        with patch("minimalmodbus.Instrument") as mock_class:
            mock_instr = MagicMock()
            mock_class.return_value = mock_instr
            mock_instr.serial = MagicMock()
            mock_instr.serial.is_open = True

            device = MaimanSF8XXX("COM3", slave_address=1)
            device.open()

            # assert_called_once() checks that Instrument() was called exactly once.
            mock_class.assert_called_once()

    def test_is_open_true_after_open(self, laser):
        """After open(), is_open should return True."""
        device, mock_instr = laser
        assert device.is_open is True

    def test_is_open_false_after_close(self, laser):
        """After close(), is_open should return False."""
        device, mock_instr = laser

        # Make the fake serial port report itself as closed after close() is called.
        mock_instr.serial.is_open = False
        device.close()

        assert device.is_open is False

    def test_read_raises_if_not_open(self):
        """
        Calling a property without opening the connection first should raise
        a ConnectionError — not silently return garbage or crash with a
        confusing message.
        """
        with patch("minimalmodbus.Instrument"):
            device = MaimanSF8XXX("COM3", slave_address=1)
            # We deliberately do NOT call device.open() here.

            # pytest.raises() is how you test that an exception IS raised.
            # If ConnectionError is NOT raised, the test fails.
            with pytest.raises(ConnectionError):
                _ = device.current_setpoint


# =============================================================================
# SECTION 4 — Current setpoint tests
#
# The laser's max forward current is 38 mA (from the test plan).
# The divider is 10 — so 38 mA is stored as integer 380 in the register.
# =============================================================================

class TestCurrentSetpoint:
    """Tests for reading and writing the LD current setpoint register."""

    def test_read_current_divides_by_10(self, laser):
        """
        The device stores current as raw integer × 10.
        Reading 2500 from the register should give 250.0 mA.
        This confirms the divider logic is correct.
        """
        device, mock_instr = laser
        set_register_return(mock_instr, 2500)

        result = device.current_setpoint

        # pytest.approx() allows a tiny floating-point tolerance (default 1e-6).
        assert result == pytest.approx(250.0)

    def test_read_zero_current(self, laser):
        """Register value 0 should give 0.0 mA."""
        device, mock_instr = laser
        set_register_return(mock_instr, 0)
        assert device.current_setpoint == pytest.approx(0.0)

    def test_read_threshold_current(self, laser):
        """
        The threshold current for this laser is ~11 mA (from test plan).
        Register value 110 should give 11.0 mA.
        """
        device, mock_instr = laser
        set_register_return(mock_instr, 110)
        assert device.current_setpoint == pytest.approx(11.0)

    def test_read_max_operating_current(self, laser):
        """
        Max forward current for QDFBLD-1550-2SM is 38 mA (from test plan).
        Register value 380 should give 38.0 mA.
        """
        device, mock_instr = laser
        set_register_return(mock_instr, 380)
        assert device.current_setpoint == pytest.approx(38.0)

    def test_write_current_multiplies_by_10(self, laser):
        """
        Writing 38.0 mA should send integer 380 to the register.
        We also need to mock the limit check — the setter reads current_max_limit
        first, so we tell the mock to return a safe limit (250.0 mA → 2500).
        """
        device, mock_instr = laser

        # The setter calls current_max_limit to validate range.
        # current_max_limit reads _Reg.CURRENT_MAX_LIMIT.
        # We tell the mock to always return 2500 (= 250.0 mA limit).
        mock_instr.read_register.return_value = 2500

        device.current_setpoint = 38.0

        # write_register should have been called with register 0x0300 and value 380.
        mock_instr.write_register.assert_called_with(
            _Reg.CURRENT_SET,   # 0x0300
            380,                # 38.0 mA × 10 = 380
            number_of_decimals=0,
            functioncode=6,
            signed=False,
        )

    def test_write_zero_current(self, laser):
        """Writing 0.0 mA should send integer 0 to the register."""
        device, mock_instr = laser
        mock_instr.read_register.return_value = 2500  # limit = 250 mA

        device.current_setpoint = 0.0

        mock_instr.write_register.assert_called_with(
            _Reg.CURRENT_SET, 0,
            number_of_decimals=0, functioncode=6, signed=False,
        )

    def test_current_above_limit_raises_value_error(self, laser):
        """
        Setting current above the hardware limit should raise ValueError.
        The controller limit is 250 mA (this specific SF8025-NM option).
        We mock the limit register to return 2500 (= 250.0 mA).
        Trying to set 251 mA should be rejected BEFORE anything is sent to the device.
        """
        device, mock_instr = laser
        mock_instr.read_register.return_value = 2500  # limit = 250.0 mA

        with pytest.raises(ValueError):
            device.current_setpoint = 251.0

    def test_current_above_laser_max_raises_value_error(self, laser):
        """
        The DUT's absolute maximum forward current is 38 mA.
        Even though the controller supports 250 mA, sending more than 38 mA
        would destroy the laser diode.
        This test confirms the limit comes from the DEVICE register, not hardcoding.
        Here we mock the limit register to 380 (= 38.0 mA) to simulate a
        pre-configured protection threshold, and verify 39 mA is rejected.
        """
        device, mock_instr = laser
        mock_instr.read_register.return_value = 380  # limit = 38.0 mA

        with pytest.raises(ValueError):
            device.current_setpoint = 39.0

    def test_negative_current_raises_value_error(self, laser):
        """Negative current makes no physical sense and should be rejected."""
        device, mock_instr = laser
        mock_instr.read_register.return_value = 2500

        with pytest.raises(ValueError):
            device.current_setpoint = -1.0


# =============================================================================
# SECTION 5 — Current measured (read-only)
# =============================================================================

class TestCurrentMeasured:
    """Tests for reading back the measured (actual) laser current."""

    def test_measured_current_divides_by_10(self, laser):
        """Raw register 380 → 38.0 mA measured."""
        device, mock_instr = laser
        set_register_return(mock_instr, 380)
        assert device.current_measured == pytest.approx(38.0)

    def test_measured_current_zero(self, laser):
        """When laser is off, measured current should read 0.0 mA."""
        device, mock_instr = laser
        set_register_return(mock_instr, 0)
        assert device.current_measured == pytest.approx(0.0)


# =============================================================================
# SECTION 6 — Voltage measured
#
# Forward voltage for QDFBLD-1550-2SM: typ. 1.0–1.4 V (from test plan).
# Divider is 1000 — register stores millivolts.
# 1.2 V → register value 1200.
# =============================================================================

class TestVoltageMeasured:
    """Tests for reading the forward voltage across the laser diode."""

    def test_voltage_divides_by_1000(self, laser):
        """Raw register 1200 → 1.2 V."""
        device, mock_instr = laser
        set_register_return(mock_instr, 1200)
        assert device.voltage_measured == pytest.approx(1.2)

    def test_voltage_lower_spec_limit(self, laser):
        """1.0 V is the minimum typical forward voltage (test plan Table 7)."""
        device, mock_instr = laser
        set_register_return(mock_instr, 1000)
        assert device.voltage_measured == pytest.approx(1.0)

    def test_voltage_upper_spec_limit(self, laser):
        """1.4 V is the maximum typical forward voltage (test plan Table 7)."""
        device, mock_instr = laser
        set_register_return(mock_instr, 1400)
        assert device.voltage_measured == pytest.approx(1.4)

    def test_voltage_zero_when_off(self, laser):
        """With laser disabled, forward voltage should read 0.0 V."""
        device, mock_instr = laser
        set_register_return(mock_instr, 0)
        assert device.voltage_measured == pytest.approx(0.0)


# =============================================================================
# SECTION 7 — TEC temperature
#
# TEC sweep range: +15°C to +40°C (test plan Table 14).
# Nominal setpoint: 25°C.
# Temperature stability criterion: ±0.1°C.
# Temperatures are SIGNED — they can be negative (stored as two's complement).
# Divider is 10 — so 25°C → register value 250.
# =============================================================================

class TestTECTemperature:
    """Tests for TEC temperature setpoint and measurement."""

    def test_tec_setpoint_read_25C(self, laser):
        """
        Nominal TEC setpoint is 25°C (from test plan).
        Register value 250 should give 25.0°C.
        """
        device, mock_instr = laser
        set_register_return(mock_instr, 250)
        assert device.tec_temperature_setpoint == pytest.approx(25.0)

    def test_tec_setpoint_read_15C(self, laser):
        """Lower bound of temperature sweep: 15°C → register 150."""
        device, mock_instr = laser
        set_register_return(mock_instr, 150)
        assert device.tec_temperature_setpoint == pytest.approx(15.0)

    def test_tec_setpoint_read_40C(self, laser):
        """Upper bound of temperature sweep: 40°C → register 400."""
        device, mock_instr = laser
        set_register_return(mock_instr, 400)
        assert device.tec_temperature_setpoint == pytest.approx(40.0)

    def test_tec_write_setpoint_25C(self, laser):
        """
        Writing 25.0°C should send integer 250 to the TEC setpoint register.
        We mock min/max reads to allow the write through.
        """
        device, mock_instr = laser

        # The setter reads tec_temperature_min and tec_temperature_max first.
        # We use side_effect to return different values for successive reads:
        # first call returns 150 (15.0°C min), second returns 400 (40.0°C max).
        mock_instr.read_register.side_effect = [150, 400]

        device.tec_temperature_setpoint = 25.0

        mock_instr.write_register.assert_called_with(
            _Reg.TEC_TEMPERATURE_SET,
            250,    # 25.0°C × 10 = 250
            number_of_decimals=0, functioncode=6, signed=False,
        )

    def test_tec_write_15C_lower_bound(self, laser):
        """Setting 15°C (lower sweep limit) should write 150."""
        device, mock_instr = laser
        mock_instr.read_register.side_effect = [150, 400]  # min=15, max=40

        device.tec_temperature_setpoint = 15.0

        mock_instr.write_register.assert_called_with(
            _Reg.TEC_TEMPERATURE_SET, 150,
            number_of_decimals=0, functioncode=6, signed=False,
        )

    def test_tec_write_40C_upper_bound(self, laser):
        """Setting 40°C (upper sweep limit) should write 400."""
        device, mock_instr = laser
        mock_instr.read_register.side_effect = [150, 400]

        device.tec_temperature_setpoint = 40.0

        mock_instr.write_register.assert_called_with(
            _Reg.TEC_TEMPERATURE_SET, 400,
            number_of_decimals=0, functioncode=6, signed=False,
        )

    def test_tec_setpoint_above_max_raises(self, laser):
        """Setting 41°C when max is 40°C should raise ValueError."""
        device, mock_instr = laser
        mock_instr.read_register.side_effect = [150, 400]  # min=15, max=40

        with pytest.raises(ValueError):
            device.tec_temperature_setpoint = 41.0

    def test_tec_setpoint_below_min_raises(self, laser):
        """Setting 14°C when min is 15°C should raise ValueError."""
        device, mock_instr = laser
        mock_instr.read_register.side_effect = [150, 400]

        with pytest.raises(ValueError):
            device.tec_temperature_setpoint = 14.0


# =============================================================================
# SECTION 8 — Signed temperature conversion (_to_signed_16)
#
# This is one of the most important correctness tests.
# Temperatures can be negative (e.g. cooling the diode below 0°C in future use).
# The device stores them as unsigned 16-bit two's complement integers.
# Without correct signed conversion, −10°C would read as 6552.6°C.
# =============================================================================

class TestSignedConversion:
    """
    Tests for the _to_signed_16() helper function.

    This function is tested in complete isolation — it doesn't need the laser
    object at all. We're just verifying the maths is correct.
    """

    def test_positive_temperature_unchanged(self):
        """Positive values below 32768 should pass through unchanged."""
        assert _to_signed_16(250) == 250    # 25.0°C raw

    def test_zero(self):
        """Zero should remain zero."""
        assert _to_signed_16(0) == 0

    def test_minus_one(self):
        """
        −1 in 16-bit two's complement is 65535 (all bits set to 1).
        65535 − 65536 = −1.
        """
        assert _to_signed_16(65535) == -1

    def test_minus_ten(self):
        """
        −10°C → register stores 65526.
        65526 − 65536 = −10.
        This is the exact example from our earlier explanation.
        """
        assert _to_signed_16(65526) == -10

    def test_minus_32768_most_negative(self):
        """
        32768 is the most negative value a signed 16-bit integer can represent.
        In two's complement: 32768 − 65536 = −32768.
        """
        assert _to_signed_16(32768) == -32768

    def test_32767_most_positive(self):
        """32767 is the largest positive signed 16-bit value — should be unchanged."""
        assert _to_signed_16(32767) == 32767

    def test_tec_measured_negative_temperature(self, laser):
        """
        End-to-end test: if the TEC register returns 65526 (= −10°C raw),
        tec_temperature_measured should return −1.0°C after dividing by 10.

        This simulates a scenario where the TEC is cooling below 0°C.
        """
        device, mock_instr = laser
        set_register_return(mock_instr, 65526)  # raw = −10, so −10 / 10 = −1.0°C
        assert device.tec_temperature_measured == pytest.approx(-1.0)

    def test_tec_measured_25C(self, laser):
        """Normal operating temperature 25°C → register 250 → 25.0°C."""
        device, mock_instr = laser
        set_register_return(mock_instr, 250)
        assert device.tec_temperature_measured == pytest.approx(25.0)


# =============================================================================
# SECTION 9 — Enable / disable (start / stop)
#
# From test plan and documentation:
# start_device() writes 0x0008 to STATE_OF_DEVICE register (function code 6)
# stop_device()  writes 0x0010 to STATE_OF_DEVICE register
# =============================================================================

class TestEnableDisable:
    """Tests that enable() and disable() send the correct Modbus commands."""

    def test_enable_sends_correct_command(self, laser):
        """
        enable() must write 0x0008 (= 8 decimal) to the STATE_OF_DEVICE register.
        This is the 'operation state ON' command per the documentation.
        """
        device, mock_instr = laser
        device.enable()

        mock_instr.write_register.assert_called_with(
            _Reg.STATE_OF_DEVICE,   # 0x0700
            0x0008,                 # = 8 decimal — start command
            number_of_decimals=0,
            functioncode=6,
            signed=False,
        )

    def test_disable_sends_correct_command(self, laser):
        """
        disable() must write 0x0010 (= 16 decimal) to STATE_OF_DEVICE.
        This is the 'operation state OFF' command.
        """
        device, mock_instr = laser
        device.disable()

        mock_instr.write_register.assert_called_with(
            _Reg.STATE_OF_DEVICE,
            0x0010,                 # = 16 decimal — stop command
            number_of_decimals=0,
            functioncode=6,
            signed=False,
        )

    def test_start_device_alias_works(self, laser):
        """start_device() is an alias for enable() — should send the same command."""
        device, mock_instr = laser
        device.start_device()

        mock_instr.write_register.assert_called_with(
            _Reg.STATE_OF_DEVICE, 0x0008,
            number_of_decimals=0, functioncode=6, signed=False,
        )

    def test_stop_device_alias_works(self, laser):
        """stop_device() is an alias for disable() — same command as disable()."""
        device, mock_instr = laser
        device.stop_device()

        mock_instr.write_register.assert_called_with(
            _Reg.STATE_OF_DEVICE, 0x0010,
            number_of_decimals=0, functioncode=6, signed=False,
        )


# =============================================================================
# SECTION 10 — State bitmask tests
#
# The STATE_OF_DEVICE register is a bitmask — individual bits have meaning:
#   Bit 1 (0x0002) = operation state (laser running)
#   Bit 2 (0x0004) = enable internal signal active
#   Bit 3 (0x0008) = current set internal
#   Bit 4 (0x0010) = interlock denied  ← IMPORTANT: laser locked out
#   Bit 5 (0x0020) = external NTC denied
# =============================================================================

class TestStateBitmasks:
    """Tests that status bits are correctly extracted from the state register."""

    def test_is_running_true_when_bit1_set(self, laser):
        """
        If bit 1 (0x0002) is set, the laser is running.
        We return 0x0002 from the state register → is_running should be True.
        """
        device, mock_instr = laser
        set_register_return(mock_instr, 0x0002)
        assert device.is_running is True

    def test_is_running_false_when_bit1_clear(self, laser):
        """If bit 1 is not set, laser is stopped → is_running should be False."""
        device, mock_instr = laser
        set_register_return(mock_instr, 0x0000)
        assert device.is_running is False

    def test_is_running_ignores_other_bits(self, laser):
        """
        Multiple bits can be set at once. is_running should only care about bit 1.
        0x0006 = bits 1 and 2 both set → is_running should still be True.
        """
        device, mock_instr = laser
        set_register_return(mock_instr, 0x0006)
        assert device.is_running is True

    def test_is_enable_internal_true(self, laser):
        """Bit 2 (0x0004) set → is_enable_internal True."""
        device, mock_instr = laser
        set_register_return(mock_instr, 0x0004)
        assert device.is_enable_internal is True

    def test_is_enable_internal_false(self, laser):
        """Bit 2 clear → is_enable_internal False."""
        device, mock_instr = laser
        set_register_return(mock_instr, 0x0002)  # only bit 1 set
        assert device.is_enable_internal is False

    def test_interlock_denied_when_bit4_set(self, laser):
        """
        SAFETY CRITICAL: if the interlock is open (test plan requires Pin 15
        shorted to GND), bit 4 (0x0010) is set → is_interlock_denied True.
        The laser cannot fire in this state.
        """
        device, mock_instr = laser
        set_register_return(mock_instr, 0x0010)
        assert device.is_interlock_denied is True

    def test_interlock_clear_when_grounded(self, laser):
        """When interlock pin is correctly shorted to GND, bit 4 should be clear."""
        device, mock_instr = laser
        set_register_return(mock_instr, 0x0002)  # running, interlock clear
        assert device.is_interlock_denied is False

    def test_all_bits_clear_on_idle(self, laser):
        """A freshly connected, idle device should have no status bits set."""
        device, mock_instr = laser
        set_register_return(mock_instr, 0x0000)
        assert device.is_running is False
        assert device.is_enable_internal is False
        assert device.is_interlock_denied is False

    def test_state_raw_returned_as_integer(self, laser):
        """state_raw should return the raw bitmask integer exactly as read."""
        device, mock_instr = laser
        set_register_return(mock_instr, 0x0006)
        assert device.state_raw == 0x0006


# =============================================================================
# SECTION 11 — Frequency and pulse duration
#
# Not used in CW (continuous wave) mode during this test plan, but the class
# supports them. We test that the scaling and range checks work correctly.
# =============================================================================

class TestFrequencyAndDuration:
    """Tests for pulse frequency and duration properties."""

    def test_frequency_read(self, laser):
        """Frequency divider is 1.0 — register 1000 → 1000.0 Hz."""
        device, mock_instr = laser
        set_register_return(mock_instr, 1000)
        assert device.frequency == pytest.approx(1000.0)

    def test_frequency_write(self, laser):
        """Writing 500.0 Hz should write integer 500 to the frequency register."""
        device, mock_instr = laser
        # side_effect: first read = min (0 Hz), second read = max (10000 Hz)
        mock_instr.read_register.side_effect = [0, 10000]

        device.frequency = 500.0

        mock_instr.write_register.assert_called_with(
            _Reg.FREQUENCY, 500,
            number_of_decimals=0, functioncode=6, signed=False,
        )

    def test_pulse_duration_read(self, laser):
        """Duration divider is 1.0 — register 100 → 100.0 µs."""
        device, mock_instr = laser
        set_register_return(mock_instr, 100)
        assert device.pulse_duration == pytest.approx(100.0)


# =============================================================================
# SECTION 12 — TEC current and voltage (read-only measurements)
#
# From test plan Table 14: TEC current is recorded during temperature sweep.
# TEC current is signed (can be negative — reversing direction to heat or cool).
# =============================================================================

class TestTECCurrentAndVoltage:
    """Tests for TEC drive current and voltage readback."""

    def test_tec_current_positive(self, laser):
        """TEC cooling: positive current 1.5 A → register 15 (divider 10)."""
        device, mock_instr = laser
        set_register_return(mock_instr, 15)
        assert device.tec_current_measured == pytest.approx(1.5)

    def test_tec_current_negative(self, laser):
        """
        TEC heating (reversed): negative current.
        Register stores −5 as 65531 (two's complement).
        65531 / 10 = −0.5 A (TEC driving heat into the diode).
        """
        device, mock_instr = laser
        set_register_return(mock_instr, 65531)   # two's complement of −5
        assert device.tec_current_measured == pytest.approx(-0.5)

    def test_tec_current_zero(self, laser):
        """TEC idle (no temperature error): current = 0.0 A."""
        device, mock_instr = laser
        set_register_return(mock_instr, 0)
        assert device.tec_current_measured == pytest.approx(0.0)

    def test_tec_voltage_read(self, laser):
        """TEC voltage 3.0 V → register 3000 (divider 1000)."""
        device, mock_instr = laser
        set_register_return(mock_instr, 3000)
        assert device.tec_voltage == pytest.approx(3.0)


# =============================================================================
# SECTION 13 — Device ID and serial number
# =============================================================================

class TestDeviceInfo:
    """Tests for device identification registers."""

    def test_device_id_read(self, laser):
        """Device ID register should return its raw integer value."""
        device, mock_instr = laser
        set_register_return(mock_instr, 8025)   # SF8025 model
        assert device.device_id == 8025

    def test_serial_number_read(self, laser):
        """Serial number register should return its raw integer value."""
        device, mock_instr = laser
        set_register_return(mock_instr, 12345)
        assert device.serial_number == 12345


# =============================================================================
# SECTION 14 — Context manager behaviour
# =============================================================================

class TestContextManager:
    """Tests that the 'with' statement opens and closes the connection cleanly."""

    def test_context_manager_opens_and_closes(self):
        """
        Using 'with MaimanSF8XXX(...) as laser:' should:
          1. Call open() on entry (is_open becomes True)
          2. Call close() on exit (is_open becomes False)
        """
        with patch("minimalmodbus.Instrument") as mock_class:
            mock_instr = MagicMock()
            mock_class.return_value = mock_instr
            mock_instr.serial = MagicMock()
            mock_instr.serial.is_open = True

            with MaimanSF8XXX("COM3", slave_address=1) as device:
                assert device.is_open is True
                # Simulate close() making is_open False
                mock_instr.serial.is_open = False

            assert device.is_open is False

    def test_context_manager_closes_on_exception(self):
        """
        Even if an exception occurs inside the 'with' block, close() must
        still be called. This prevents the COM port from being permanently locked.
        """
        with patch("minimalmodbus.Instrument") as mock_class:
            mock_instr = MagicMock()
            mock_class.return_value = mock_instr
            mock_instr.serial = MagicMock()
            mock_instr.serial.is_open = True

            with pytest.raises(RuntimeError):
                with MaimanSF8XXX("COM3", slave_address=1) as device:
                    raise RuntimeError("simulated crash inside test")

            # close() should have been called despite the exception
            mock_instr.serial.close.assert_called()
