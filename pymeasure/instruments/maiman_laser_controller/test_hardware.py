import pytest
import time
from pymeasure.instruments.maiman_laser_controller.controller import SF8xxx


# =========================
# CONFIGURATION
# =========================
COM_PORT = "COM3"   # must match BenchSoft
BAUDRATE = 115200


# =========================
# FIXTURE
# =========================
@pytest.fixture(scope="module")
def controller():
    try:
        dev = SF8xxx(COM_PORT, start_thread=False)
        if not dev.connected:
            pytest.skip("SF8xxx not connected")
        yield dev
    finally:
        if hasattr(dev, "dev") and dev.dev.is_open:
            dev.dev.close()


# =========================
# CONNECTION TESTS
# =========================
def test_connected(controller):
    assert controller.connected is True


def test_serial_number_readable(controller):
    sn = controller.serial_no
    assert isinstance(sn, int)
    assert sn > 0


# =========================
# DRIVER STATE TESTS
# =========================
def test_driver_state_readable(controller):
    state = controller.driver_state()
    assert len(state) == 6


def test_driver_current_readable(controller):
    current = controller.get_driver_current()
    assert isinstance(current, float)
    assert current >= 0.0


def test_driver_max_current_readable(controller):
    max_current = controller.get_driver_current_max()
    assert isinstance(max_current, float)
    assert max_current > 0.0


# =========================
# TEC TESTS
# =========================
def test_tec_temperature_readable(controller):
    temp = controller.get_tec_temperature()
    assert isinstance(temp, float)
    assert -20.0 <= temp <= 80.0


def test_tec_current_readable(controller):
    current = controller.get_tec_current()
    assert isinstance(current, float)


def test_tec_current_limit_readable(controller):
    limit = controller.get_tec_current_limit()
    assert isinstance(limit, float)
    assert limit > 0.0


# =========================
# LOCK / SAFETY TESTS
# =========================
def test_lock_state_readable(controller):
    state = controller.lock_state()
    assert len(state) == 6


# =========================
# SETPOINT ROUND-TRIP TESTS
# =========================
def test_set_driver_current(controller):
    controller.set_driver_current(5.0)
    readback = controller.get_driver_value()
    assert pytest.approx(readback, abs=0.5) == 5.0


def test_set_tec_temperature(controller):
    controller.set_tec_temperature(25.0)
    readback = controller.get_tec_value()
    assert pytest.approx(readback, abs=0.2) == 25.0


#========================
#Enable/Disable Tests
#========================

def test_driver_enable_with_interlock_open(controller):
    # Ensure safe current
    controller.set_driver_current(5)

    # Attempt to enable driver
    result = controller.set_driver_on()

    # Read driver state
    device, driver, current, enable, ntc, interlock = controller.driver_state()

    # Driver must remain OFF because interlock is open
    assert driver == 0

class TestTECThermalControlInterlockOpen:
    """
    Validate TEC behavior with laser connected and interlock open.
    Driver must remain OFF.
    """

    def test_tec_setpoint_and_response(self, controller):
        # Ensure driver is off
        _, driver, *_ = controller.driver_state()
        assert driver == 0

        # When using digital (UART) control, deny_interlock() tells the
        # controller to ignore the state of analogue pin 15, so the TEC
        # can be enabled without grounding the interlock pin.
        controller.deny_interlock()

        # Enable TEC if not already on
        controller.set_tec_int()
        controller.set_tec_on()
                  
        # Read initial temperature
        t_initial = controller.get_tec_temperature()

        # Change setpoint slightly
        controller.set_tec_temperature(t_initial + 2)

        time.sleep(2)

        # Read back temperature and setpoint
        t_measured = controller.get_tec_temperature()
        t_set = controller.get_tec_value()

        # Validate
        assert abs(t_set - (t_initial + 2)) < 0.3
        assert abs(t_measured - t_set) < 2.0