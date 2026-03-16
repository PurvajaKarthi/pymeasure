#
# This file is part of the PyMeasure package.
#
# Copyright (c) 2013-2024 PyMeasure Developers
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

"""
Maiman Electronics SF8XXX Laser Diode Driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Driver for the Maiman Electronics SF8XXX series laser diode driver with
integrated TEC controller. Communication uses Modbus RTU over RS-485 via
the `minimalmodbus` backend.

Supported models: SF8XXX-NM v2.2 (LD current up to 3000 mA)

.. code-block:: python

    from pymeasure.instruments.maiman import MaimanSF8XXX

    laser = MaimanSF8XXX("COM6", slave_address=1)
    laser.open()

    laser.current_setpoint = 250.0        # mA
    laser.frequency = 1000.0              # Hz
    laser.pulse_duration = 100.0          # µs

    laser.enable()

    print(laser.current_measured)         # mA
    print(laser.voltage_measured)         # V
    print(laser.pcb_temperature)          # °C
    print(laser.tec_temperature_measured) # °C
    print(laser.is_running)               # bool

    laser.disable()
    laser.close()
"""

import logging
import struct
import time
from typing import Optional

log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())

try:
    import minimalmodbus
    import serial
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "MaimanSF8XXX requires 'minimalmodbus' and 'pyserial'. "
        "Install them with:  pip install minimalmodbus pyserial"
    ) from e


# ---------------------------------------------------------------------------
# Modbus register map  (from modbus_config.yaml / library documentation)
# ---------------------------------------------------------------------------

class _Reg:
    """Register address constants for the SF8XXX device."""

    # LD parameters
    CURRENT_SET         = 0x0300
    CURRENT_MIN         = 0x0301
    CURRENT_MAX         = 0x0302
    CURRENT_MEASURED    = 0x0307
    CURRENT_MAX_LIMIT   = 0x0308
    CURRENT_PROTECTION  = 0x030A
    CURRENT_CALIBRATION = 0x030B

    FREQUENCY           = 0x0400
    FREQUENCY_MIN       = 0x0401
    FREQUENCY_MAX       = 0x0402

    DURATION            = 0x0500
    DURATION_MIN        = 0x0501
    DURATION_MAX        = 0x0502

    VOLTAGE_MEASURED    = 0x0600

    # Device state / status
    STATE_OF_DEVICE     = 0x0700

    # Device info
    DEVICE_ID           = 0x0800
    SERIAL_NUMBER       = 0x0801

    # TEC parameters
    TEC_TEMPERATURE_SET   = 0x0A00
    TEC_TEMPERATURE_MEAS  = 0x0A01
    TEC_TEMPERATURE_MIN   = 0x0A02
    TEC_TEMPERATURE_MAX   = 0x0A03
    TEC_CURRENT_MEASURED  = 0x0A16
    TEC_CURRENT_LIMIT     = 0x0A17
    TEC_VOLTAGE           = 0x0A18
    TEC_STATE             = 0x0A20

    # Lock / interlock
    LOCK_STATUS         = 0x0B00

    # PID (TEC tuning)
    TEC_PID_KP          = 0x0A30
    TEC_PID_KI          = 0x0A31
    TEC_PID_KD          = 0x0A32

    # NTC coefficients
    NTC_B25_100         = 0x0C00


# State bitmasks (state_of_device register)
class _Bit:
    OPERATION_STATE  = 0x0002   # bit 1 — 1=started, 0=stopped
    ENABLE_INTERNAL  = 0x0004   # bit 2
    CURRENT_INTERNAL = 0x0008   # bit 3
    INTERLOCK_DENIED = 0x0010   # bit 4
    EXT_NTC_DENIED   = 0x0020   # bit 5

# State write commands (written to STATE_OF_DEVICE register)
_CMD_START = 0x0008
_CMD_STOP  = 0x0010


# ---------------------------------------------------------------------------
# Scaling / dividers  (from device_config.yaml / documentation)
# ---------------------------------------------------------------------------

_DIV_CURRENT     = 10.0    # raw int → mA
_DIV_FREQUENCY   = 1.0     # raw int → Hz
_DIV_DURATION    = 1.0     # raw int → µs
_DIV_VOLTAGE     = 1000.0  # raw int → V
_DIV_TEMPERATURE = 10.0    # raw int → °C  (signed 16-bit)
_DIV_TEC_CURRENT = 10.0    # raw int → A   (signed 16-bit)


def _to_signed_16(val: int) -> int:
    """Convert a 16-bit unsigned Modbus value to a signed integer."""
    val = int(val) & 0xFFFF
    return val if val < 0x8000 else val - 0x10000


# ---------------------------------------------------------------------------
# Main instrument class
# ---------------------------------------------------------------------------

class MaimanSF8XXX:
    """Maiman Electronics SF8XXX series laser diode driver with TEC.

    Communicates over **Modbus RTU / RS-485** using the `minimalmodbus`
    library.  The class intentionally mirrors the PyMeasure ``Instrument``
    interface so it integrates cleanly into PyMeasure-based experiments
    while relying on a direct ``minimalmodbus.Instrument`` connection
    internally (PyMeasure does not yet have a native Modbus adapter).

    :param port: Serial port string (e.g. ``"COM6"`` or ``"/dev/ttyUSB0"``).
    :param slave_address: Modbus RTU slave address of the device (default 1).
    :param baudrate: Serial baud rate (default 115200).
    :param timeout: Read/write timeout in seconds (default 0.1).
    :param auto_open: If ``True``, open the connection in ``__init__``
        (default ``False``).

    .. note::
        Always call :meth:`open` before accessing properties, and
        :meth:`close` when finished.  Alternatively use the instance as a
        context manager::

            with MaimanSF8XXX("COM6") as laser:
                laser.current_setpoint = 100.0

    .. warning::
        Setting :attr:`current_setpoint` above the hardware
        :attr:`current_max_limit` will raise a :class:`ValueError`.
        Always verify limits before writing to the device.
    """

    name = "Maiman Electronics SF8XXX Laser Diode Driver"

    # ------------------------------------------------------------------
    # Construction / connection lifecycle
    # ------------------------------------------------------------------

    def __init__(
        self,
        port: str,
        slave_address: int = 1,
        baudrate: int = 115200,
        timeout: float = 0.1,
        auto_open: bool = False,
    ):
        self._port = port
        self._slave_address = slave_address
        self._baudrate = baudrate
        self._timeout = timeout
        self._instrument: Optional[minimalmodbus.Instrument] = None

        if auto_open:
            self.open()

    # context-manager support
    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()

    def open(self):
        """Open the Modbus RTU serial connection."""
        instr = minimalmodbus.Instrument(self._port, self._slave_address)
        instr.serial.baudrate = self._baudrate
        instr.serial.timeout = self._timeout
        instr.serial.bytesize = 8
        instr.serial.parity = serial.PARITY_NONE
        instr.serial.stopbits = 1
        instr.mode = minimalmodbus.MODE_RTU
        instr.close_port_after_each_call = False
        instr.precalculate_read_size = True
        self._instrument = instr
        log.info(
            "Opened %s on %s (slave=%d, baud=%d)",
            self.name, self._port, self._slave_address, self._baudrate,
        )

    def close(self):
        """Close the serial connection."""
        if self._instrument and self._instrument.serial.is_open:
            self._instrument.serial.close()
            log.info("Closed connection on %s", self._port)

    @property
    def is_open(self) -> bool:
        """Return ``True`` if the serial port is currently open."""
        return (
            self._instrument is not None
            and self._instrument.serial.is_open
        )

    def _check_open(self):
        if not self.is_open:
            raise ConnectionError(
                "Device is not connected. Call open() first."
            )

    # ------------------------------------------------------------------
    # Low-level register helpers
    # ------------------------------------------------------------------

    def _read(self, register: int, signed: bool = False) -> int:
        """Read a single 16-bit Modbus holding register."""
        self._check_open()
        try:
            raw = self._instrument.read_register(
                register,
                number_of_decimals=0,
                functioncode=3,
                signed=signed,
            )
            return int(raw)
        except Exception as exc:
            raise IOError(
                f"Read failed at register 0x{register:04X}: {exc}"
            ) from exc

    def _write(self, register: int, value: int):
        """Write a single 16-bit value to a Modbus holding register."""
        self._check_open()
        try:
            self._instrument.write_register(
                register,
                value,
                number_of_decimals=0,
                functioncode=6,
                signed=False,
            )
        except Exception as exc:
            raise IOError(
                f"Write failed at register 0x{register:04X} "
                f"(value={value}): {exc}"
            ) from exc

    def _read_signed(self, register: int) -> int:
        """Read a register and interpret as a signed 16-bit integer."""
        return _to_signed_16(self._read(register))

    # ------------------------------------------------------------------
    # Device identification
    # ------------------------------------------------------------------

    @property
    def device_id(self) -> int:
        """Return the numeric device ID stored in firmware."""
        return self._read(_Reg.DEVICE_ID)

    @property
    def serial_number(self) -> int:
        """Return the device serial number."""
        return self._read(_Reg.SERIAL_NUMBER)

    # ------------------------------------------------------------------
    # LD current  (mA)
    # ------------------------------------------------------------------

    @property
    def current_setpoint(self) -> float:
        """Laser diode current setpoint in **mA**.

        Valid range: 0 mA – :attr:`current_max` mA.

        :raises ValueError: If the requested value exceeds :attr:`current_max_limit`.
        """
        return self._read(_Reg.CURRENT_SET) / _DIV_CURRENT

    @current_setpoint.setter
    def current_setpoint(self, value_mA: float):
        limit = self.current_max_limit
        if value_mA < 0 or value_mA > limit:
            raise ValueError(
                f"current_setpoint {value_mA:.1f} mA is out of range "
                f"[0, {limit:.1f}] mA."
            )
        self._write(_Reg.CURRENT_SET, int(round(value_mA * _DIV_CURRENT)))

    @property
    def current_measured(self) -> float:
        """Measured laser diode output current in **mA** (read-only)."""
        return self._read(_Reg.CURRENT_MEASURED) / _DIV_CURRENT

    @property
    def current_min(self) -> float:
        """Minimum allowable current setpoint in **mA** (read-only)."""
        return self._read(_Reg.CURRENT_MIN) / _DIV_CURRENT

    @property
    def current_max(self) -> float:
        """Maximum allowable current setpoint in **mA** (read-only)."""
        return self._read(_Reg.CURRENT_MAX) / _DIV_CURRENT

    @property
    def current_max_limit(self) -> float:
        """Absolute hardware current limit in **mA** (read-only).

        This is the ceiling enforced by :attr:`current_setpoint`.
        """
        return self._read(_Reg.CURRENT_MAX_LIMIT) / _DIV_CURRENT

    @property
    def current_protection_threshold(self) -> float:
        """Current protection trip threshold in **mA** (read-only)."""
        return self._read(_Reg.CURRENT_PROTECTION) / _DIV_CURRENT

    @property
    def current_calibration(self) -> float:
        """Current set calibration offset in **mA** (read-only)."""
        return self._read(_Reg.CURRENT_CALIBRATION) / _DIV_CURRENT

    # ------------------------------------------------------------------
    # Pulse frequency  (Hz)
    # ------------------------------------------------------------------

    @property
    def frequency(self) -> float:
        """Pulse repetition frequency in **Hz**.

        Valid range: :attr:`frequency_min` – :attr:`frequency_max`.
        """
        return self._read(_Reg.FREQUENCY) / _DIV_FREQUENCY

    @frequency.setter
    def frequency(self, value_Hz: float):
        f_min = self.frequency_min
        f_max = self.frequency_max
        if value_Hz < f_min or value_Hz > f_max:
            raise ValueError(
                f"frequency {value_Hz:.1f} Hz is out of range "
                f"[{f_min:.1f}, {f_max:.1f}] Hz."
            )
        self._write(_Reg.FREQUENCY, int(round(value_Hz * _DIV_FREQUENCY)))

    @property
    def frequency_min(self) -> float:
        """Minimum frequency limit in **Hz** (read-only)."""
        return self._read(_Reg.FREQUENCY_MIN) / _DIV_FREQUENCY

    @property
    def frequency_max(self) -> float:
        """Maximum frequency limit in **Hz** (read-only)."""
        return self._read(_Reg.FREQUENCY_MAX) / _DIV_FREQUENCY

    # ------------------------------------------------------------------
    # Pulse duration  (µs)
    # ------------------------------------------------------------------

    @property
    def pulse_duration(self) -> float:
        """Pulse duration (on-time) in **µs**.

        Valid range: :attr:`pulse_duration_min` – :attr:`pulse_duration_max`.
        """
        return self._read(_Reg.DURATION) / _DIV_DURATION

    @pulse_duration.setter
    def pulse_duration(self, value_us: float):
        d_min = self.pulse_duration_min
        d_max = self.pulse_duration_max
        if value_us < d_min or value_us > d_max:
            raise ValueError(
                f"pulse_duration {value_us:.1f} µs is out of range "
                f"[{d_min:.1f}, {d_max:.1f}] µs."
            )
        self._write(_Reg.DURATION, int(round(value_us * _DIV_DURATION)))

    @property
    def pulse_duration_min(self) -> float:
        """Minimum pulse duration in **µs** (read-only)."""
        return self._read(_Reg.DURATION_MIN) / _DIV_DURATION

    @property
    def pulse_duration_max(self) -> float:
        """Maximum pulse duration in **µs** (read-only)."""
        return self._read(_Reg.DURATION_MAX) / _DIV_DURATION

    # ------------------------------------------------------------------
    # Voltage  (V)
    # ------------------------------------------------------------------

    @property
    def voltage_measured(self) -> float:
        """Measured forward voltage across the laser diode in **V** (read-only)."""
        return self._read(_Reg.VOLTAGE_MEASURED) / _DIV_VOLTAGE

    # ------------------------------------------------------------------
    # Board temperature  (°C)
    # ------------------------------------------------------------------

    @property
    def pcb_temperature(self) -> float:
        """PCB / driver board temperature in **°C** (read-only).

        Value is interpreted as a signed 16-bit integer.
        """
        return self._read_signed(_Reg.TEC_TEMPERATURE_MEAS) / _DIV_TEMPERATURE

    # ------------------------------------------------------------------
    # TEC  (temperature, current, voltage)
    # ------------------------------------------------------------------

    @property
    def tec_temperature_setpoint(self) -> float:
        """TEC target temperature in **°C**."""
        return self._read_signed(_Reg.TEC_TEMPERATURE_SET) / _DIV_TEMPERATURE

    @tec_temperature_setpoint.setter
    def tec_temperature_setpoint(self, value_C: float):
        t_min = self.tec_temperature_min
        t_max = self.tec_temperature_max
        if value_C < t_min or value_C > t_max:
            raise ValueError(
                f"tec_temperature_setpoint {value_C:.1f} °C is out of range "
                f"[{t_min:.1f}, {t_max:.1f}] °C."
            )
        raw = _to_signed_16(int(round(value_C * _DIV_TEMPERATURE))) & 0xFFFF
        self._write(_Reg.TEC_TEMPERATURE_SET, raw)

    @property
    def tec_temperature_measured(self) -> float:
        """Measured TEC cold-plate temperature in **°C** (read-only)."""
        return self._read_signed(_Reg.TEC_TEMPERATURE_MEAS) / _DIV_TEMPERATURE

    @property
    def tec_temperature_min(self) -> float:
        """Minimum TEC temperature setpoint in **°C** (read-only)."""
        return self._read_signed(_Reg.TEC_TEMPERATURE_MIN) / _DIV_TEMPERATURE

    @property
    def tec_temperature_max(self) -> float:
        """Maximum TEC temperature setpoint in **°C** (read-only)."""
        return self._read_signed(_Reg.TEC_TEMPERATURE_MAX) / _DIV_TEMPERATURE

    @property
    def tec_current_measured(self) -> float:
        """Measured TEC drive current in **A** (read-only, signed)."""
        return self._read_signed(_Reg.TEC_CURRENT_MEASURED) / _DIV_TEC_CURRENT

    @property
    def tec_current_limit(self) -> float:
        """TEC current limit in **A** (read-only)."""
        return self._read(_Reg.TEC_CURRENT_LIMIT) / _DIV_TEC_CURRENT

    @property
    def tec_voltage(self) -> float:
        """Measured TEC drive voltage in **V** (read-only, signed)."""
        return self._read_signed(_Reg.TEC_VOLTAGE) / _DIV_VOLTAGE

    # TEC PID tuning (read/write)

    @property
    def tec_pid_kp(self) -> int:
        """TEC PID proportional gain (raw integer)."""
        return self._read(_Reg.TEC_PID_KP)

    @tec_pid_kp.setter
    def tec_pid_kp(self, value: int):
        self._write(_Reg.TEC_PID_KP, int(value))

    @property
    def tec_pid_ki(self) -> int:
        """TEC PID integral gain (raw integer)."""
        return self._read(_Reg.TEC_PID_KI)

    @tec_pid_ki.setter
    def tec_pid_ki(self, value: int):
        self._write(_Reg.TEC_PID_KI, int(value))

    @property
    def tec_pid_kd(self) -> int:
        """TEC PID derivative gain (raw integer)."""
        return self._read(_Reg.TEC_PID_KD)

    @tec_pid_kd.setter
    def tec_pid_kd(self, value: int):
        self._write(_Reg.TEC_PID_KD, int(value))

    # ------------------------------------------------------------------
    # Device state register & bitmask helpers
    # ------------------------------------------------------------------

    @property
    def state_raw(self) -> int:
        """Raw bitmask value of the ``state_of_device`` register (read-only)."""
        return self._read(_Reg.STATE_OF_DEVICE)

    def _is_bit_set(self, bit_mask: int) -> bool:
        return bool(self.state_raw & bit_mask)

    @property
    def is_running(self) -> bool:
        """``True`` if the laser operation state bit is active (started)."""
        return self._is_bit_set(_Bit.OPERATION_STATE)

    @property
    def is_enable_internal(self) -> bool:
        """``True`` if the internal enable signal is active."""
        return self._is_bit_set(_Bit.ENABLE_INTERNAL)

    @property
    def is_current_set_internal(self) -> bool:
        """``True`` if internal current-set mode is active."""
        return self._is_bit_set(_Bit.CURRENT_INTERNAL)

    @property
    def is_interlock_denied(self) -> bool:
        """``True`` if the general interlock is tripped (output blocked)."""
        return self._is_bit_set(_Bit.INTERLOCK_DENIED)

    @property
    def is_external_ntc_denied(self) -> bool:
        """``True`` if the external NTC interlock condition is active."""
        return self._is_bit_set(_Bit.EXT_NTC_DENIED)

    @property
    def lock_status(self) -> int:
        """Raw lock / interlock status register value (read-only)."""
        return self._read(_Reg.LOCK_STATUS)

    # ------------------------------------------------------------------
    # Start / stop
    # ------------------------------------------------------------------

    def enable(self):
        """Start laser diode operation (sets the operation-state bit).

        Sends command ``0x0008`` to the ``state_of_device`` register.

        .. warning::
            Ensure the current setpoint, frequency, and pulse duration are
            configured before calling this method.
        """
        self._write(_Reg.STATE_OF_DEVICE, _CMD_START)
        log.info("Laser enabled (start command sent).")

    def disable(self):
        """Stop laser diode operation (clears the operation-state bit).

        Sends command ``0x0010`` to the ``state_of_device`` register.
        """
        self._write(_Reg.STATE_OF_DEVICE, _CMD_STOP)
        log.info("Laser disabled (stop command sent).")

    # Keep the library's original names as aliases for familiarity
    start_device = enable
    stop_device  = disable

    # ------------------------------------------------------------------
    # Convenience: wait for stable TEC temperature
    # ------------------------------------------------------------------

    def wait_for_tec_stable(
        self,
        tolerance_C: float = 0.5,
        timeout_s: float = 120.0,
        poll_s: float = 1.0,
    ) -> bool:
        """Block until the TEC reaches the setpoint within *tolerance_C*.

        :param tolerance_C: Acceptable deviation from setpoint in °C.
        :param timeout_s: Maximum wait time in seconds.
        :param poll_s: Polling interval in seconds.
        :returns: ``True`` if stable within timeout, ``False`` otherwise.
        """
        deadline = time.time() + timeout_s
        setpoint = self.tec_temperature_setpoint
        while time.time() < deadline:
            measured = self.tec_temperature_measured
            if abs(measured - setpoint) <= tolerance_C:
                log.info(
                    "TEC stable: %.2f °C (setpoint %.2f °C)", measured, setpoint
                )
                return True
            log.debug(
                "TEC: %.2f °C → target %.2f °C (Δ=%.2f °C)",
                measured, setpoint, abs(measured - setpoint),
            )
            time.sleep(poll_s)
        log.warning("TEC did not stabilize within %.1f s.", timeout_s)
        return False

    # ------------------------------------------------------------------
    # Human-readable summary
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        status = "open" if self.is_open else "closed"
        return (
            f"<MaimanSF8XXX port={self._port!r} "
            f"slave={self._slave_address} [{status}]>"
        )