"""
Synchronous Alicat flow controller driver for threaded hardware manager.

This is a direct port of the async Alicat driver to synchronous operation.
Uses pymodbus synchronous client instead of async client.
"""

from __future__ import annotations

import struct
import threading
import time
from typing import Any, Dict, Final, Tuple

from pymodbus.client import ModbusTcpClient


# Register addresses
REG_SETPOINT_F32: Final[int] = 1349
REG_VALVEDRIVE_F32: Final[int] = 1351
REG_PRESSURE_F32: Final[int] = 1353
REG_TEMPERATURE_F32: Final[int] = 1359
REG_VOL_FLOW_F32: Final[int] = 1361
REG_MASS_FLOW_F32: Final[int] = 1363
BLOCK_START: Final[int] = 1349
BLOCK_LEN: Final[int] = 16
SETPOINT_MIN: Final[float] = 0.0
SETPOINT_MAX: Final[float] = 1000.0
REG_PAIR_LEN: Final[int] = 2
device_id_DEFAULT: Final[int] = 1


def _regs_to_float(hi: int, lo: int) -> float:
    """Convert two 16-bit Modbus registers to IEEE-754 float."""
    raw = (hi << 16) | lo
    return struct.unpack(">f", raw.to_bytes(4, "big"))[0]


def _float_to_regs(val: float) -> Tuple[int, int]:
    """Convert IEEE-754 float to two 16-bit Modbus registers."""
    raw = struct.unpack(">I", struct.pack(">f", val))[0]
    return (raw >> 16) & 0xFFFF, raw & 0xFFFF


class Alicat():
    """Synchronous Alicat flow controller driver."""

    supports_writes = True

    def __init__(
        self,
        name: str,
        host: str,
        logger: Any,
        port: int = 502,
        device_id: int = device_id_DEFAULT,
        dummy: bool = False,
    ):
        """
        Initialize Alicat Modbus TCP client.

        Args:
            name: Device identifier
            host: IP address or hostname of device
            logger: Logger instance
            port: Modbus TCP port (default: 502)
            device_id: Modbus device ID (default: 1)
            dummy: If True, return dummy data instead of real device reads
        """
        self.name = name
        self._host = host
        self._port = port
        self._device_id = device_id
        self._logger = logger
        self._dummy = dummy

        self._cli: ModbusTcpClient | None = None
        self._lock = threading.Lock()
        self.connected = False

    def initialize(self) -> None:
        """
        Establish Modbus TCP connection to device.

        Raises:
            ConnectionError: If unable to connect (unless dummy mode)
        """
        if self._dummy:
            self._logger.debug(f"[{self.name}] Running in dummy mode - skipping connection")
            self.connected = True
            return

        if self._cli and self._cli.connected:
            self.connected = True
            return

        self._logger.debug(f"[{self.name}] Connecting to {self._host}:{self._port}")
        self._cli = ModbusTcpClient(host=self._host, port=self._port)
        
        if not self._cli.connect():
            self._logger.error(f"[{self.name}] Unable to connect")
            raise ConnectionError(f"Unable to connect to Alicat at {self._host}:{self._port}")

        self.connected = True
        self._logger.debug(f"[{self.name}] Connected successfully")

    def close(self) -> None:
        """Close Modbus TCP connection."""
        if self._cli:
            self._cli.close()
            self._cli = None
        self.connected = False
        self._logger.debug(f"[{self.name}] Disconnected")

    def _read_input_f32(self, address: int) -> float:
        """
        Read 32-bit float from input registers (FC4).

        Args:
            address: Starting register address

        Returns:
            Float value from two consecutive registers
        """
        with self._lock:
            if not self._cli:
                raise RuntimeError("Not connected")

            rr = self._cli.read_input_registers(
                address=address, count=REG_PAIR_LEN, device_id=self._device_id
            )

            if rr.isError():
                raise IOError(f"FC4 read failed @{address}: {rr}")

            return _regs_to_float(rr.registers[0], rr.registers[1])

    def _read_holding_f32(self, address: int) -> float:
        """
        Read 32-bit float from holding registers (FC3).

        Args:
            address: Starting register address

        Returns:
            Float value from two consecutive registers
        """
        with self._lock:
            if not self._cli:
                raise RuntimeError("Not connected")

            rr = self._cli.read_holding_registers(
                address=address, count=REG_PAIR_LEN, device_id=self._device_id
            )

            if rr.isError():
                raise IOError(f"FC3 read failed @{address}: {rr}")

            return _regs_to_float(rr.registers[0], rr.registers[1])

    def _write_holding_f32(self, address: int, value: float) -> None:
        """
        Write 32-bit float to holding registers (FC16).

        Args:
            address: Starting register address
            value: Float value to write
        """
        with self._lock:
            if not self._cli:
                raise RuntimeError("Not connected")

            hi, lo = _float_to_regs(value)
            wr = self._cli.write_registers(
                address=address, values=[hi, lo], device_id=self._device_id
            )

            if wr.isError():
                raise IOError(f"FC16 write failed @{address}: {wr}")



    def read(self, include_pid: bool = False) -> Dict[str, Any]:
        """
        Read all device telemetry in a single block read.

        Args:
            include_pid: If True, also read PID values (adds ~300ms due to
                        Alicat command/response protocol). Default False
                        for faster polling.
        """
        if self._dummy:
            return {
                "connected": True,
                "setpoint": 22.0,
                "pressure": 8.92,
                "temperature": 306.0,
                "volumetric_flow": 21.0,
                "mass_flow": 23.0,
                "valve_drive": 27.0,
            }

        with self._lock:
            if not self._cli:
                raise RuntimeError("Not connected")

            rr = self._cli.read_input_registers(
                address=BLOCK_START, count=BLOCK_LEN, device_id=self._device_id
            )

            if rr.isError():
                raise IOError(f"Block read failed: {rr}")

            r = rr.registers

        def f32(idx: int) -> float:
            return _regs_to_float(r[idx], r[idx + 1])

        result = {
            "connected": self.connected,
            "setpoint": f32(0),
            "valve_drive": f32(2),
            "pressure": f32(4),
            "temperature": f32(10),
            "volumetric_flow": f32(12),
            "mass_flow": f32(14),
        }

        if include_pid:
            pid_values = self.read_pid()
            result.update(pid_values)

        return result

    def read_setpoint(self) -> float:
        """Read current flow setpoint in SLPM."""
        return self._read_holding_f32(REG_SETPOINT_F32)

    def read_mass_flow(self) -> float:
        """Read mass flow rate in SLPM."""
        return self._read_input_f32(REG_MASS_FLOW_F32)

    def read_pressure(self) -> float:
        """Read current pressure in bar."""
        return self._read_input_f32(REG_PRESSURE_F32)

    def read_temperature(self) -> float:
        """Read current temperature in Celsius."""
        return self._read_input_f32(REG_TEMPERATURE_F32)

    def set_flow_rate(self, flow_rate: float) -> Dict[str, Any]:
        """
        Set flow rate setpoint.

        Args:
            flow_rate: Target flow rate in SLPM (0.0-1000.0)

        Returns:
            Result dict with status and new setpoint
        """
        if self._dummy:
            self._logger.debug(f"[{self.name}] Dummy mode: setpoint -> {flow_rate}")
            return {"status": "ok", "setpoint": flow_rate}

        if not (SETPOINT_MIN <= flow_rate <= SETPOINT_MAX):
            self._logger.warning(f"[{self.name}] Flow rate out of range: {flow_rate}")

        self._write_holding_f32(REG_SETPOINT_F32, flow_rate)
        self._logger.debug(f"[{self.name}] Setpoint set to {flow_rate} SLPM")

        return {"status": "ok", "setpoint": flow_rate}

    def read_pid(self) -> Dict[str, int]:
        """
        Read PID controller parameters.

        Returns:
            Dict with keys 'P', 'D', 'I' containing gain values
        """
        if self._dummy:
            return {"P": 50, "D": 120, "I": 0}

        with self._lock:
            if not self._cli:
                raise RuntimeError("Not connected")

            self._cli.write_registers(
                address=999, values=[14, 0], device_id=self._device_id
            )
            time.sleep(0.1)  
            result = self._cli.read_input_registers(
                address=1000, count=1, device_id=self._device_id
            )
            p_gain = result.registers[0] if not result.isError() else 0

            self._cli.write_registers(
                address=999, values=[14, 1], device_id=self._device_id
            )
            time.sleep(0.1)
            result = self._cli.read_input_registers(
                address=1000, count=1, device_id=self._device_id
            )
            d_gain = result.registers[0] if not result.isError() else 0

            self._cli.write_registers(
                address=999, values=[14, 2], device_id=self._device_id
            )
            time.sleep(0.1)
            result = self._cli.read_input_registers(
                address=1000, count=1, device_id=self._device_id
            )
            i_gain = result.registers[0] if not result.isError() else 0

            return {"P": p_gain, "D": d_gain, "I": i_gain}

    def set_pid(self, P: int = None, D: int = None, I: int = None) -> Dict[str, Any]:
        """
        Set PID controller parameters.

        Args:
            P: Proportional gain
            D: Derivative gain
            I: Integral gain

        Returns:
            Result dict with status
        """
        if self._dummy:
            return {"status": "ok", "P": P, "D": D, "I": I}

        with self._lock:
            if not self._cli:
                raise RuntimeError("Not connected")

            results = {}

            if P is not None:
                result = self._cli.write_registers(
                    address=999, values=[8, int(P)], device_id=self._device_id
                )
                if result.isError():
                    self._logger.warning(f"[{self.name}] P gain set failed: {result}")
                else:
                    self._logger.debug(f"[{self.name}] P gain set to {P}")
                    results["P"] = P
                time.sleep(0.1)

            if D is not None:
                result = self._cli.write_registers(
                    address=999, values=[9, int(D)], device_id=self._device_id
                )
                if result.isError():
                    self._logger.warning(f"[{self.name}] D gain set failed: {result}")
                else:
                    self._logger.debug(f"[{self.name}] D gain set to {D}")
                    results["D"] = D
                time.sleep(0.1)

            if I is not None:
                result = self._cli.write_registers(
                    address=999, values=[10, int(I)], device_id=self._device_id
                )
                if result.isError():
                    self._logger.warning(f"[{self.name}] I gain set failed: {result}")
                else:
                    self._logger.debug(f"[{self.name}] I gain set to {I}")
                    results["I"] = I
                time.sleep(0.1)

            return {"status": "ok", **results}


    def write(self, data: Dict[str, Any]) -> Any:
        """
        Args:
            data: Dict with command parameters

        Returns:
            Result from the operation
        """
        if "setpoint" in data:
            return self.set_flow_rate(data["setpoint"])

        if any(k in data for k in ("P", "D", "I")):
            return self.set_pid(
                P=data.get("P"),
                D=data.get("D"),
                I=data.get("I"),
            )

        return {"status": "no_op", "message": "No recognized command in data"}
