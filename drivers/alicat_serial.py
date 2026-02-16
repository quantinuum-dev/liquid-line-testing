"""
Synchronous Alicat flow controller driver using direct serial communication.

Implements the Alicat ASCII protocol directly via pyserial for minimal latency.
"""

from __future__ import annotations

import threading
from typing import Any, Dict

import serial



class AlicatSerial():
    """Direct pyserial Alicat flow controller driver."""

    supports_writes = True

    # Standard Alicat gas table (subset of common gases)
    GAS_TABLE = {
        "Air": 0, "Ar": 1, "CH4": 2, "CO": 3, "CO2": 4, "C2H6": 5,
        "H2": 6, "He": 7, "N2": 8, "N2O": 9, "Ne": 10, "O2": 11,
        "C3H8": 12, "SF6": 13, "C4H10": 14, "C2H2": 15, "C2H4": 16,
        "NH3": 18, "Kr": 20, "Xe": 21, "CF4": 25,
    }

    def __init__(
        self,
        name: str,
        address: str,
        logger: Any,
        baudrate: int = 19200,
        unit_id: str = "A",
        dummy: bool = False,
    ):
        """
        Initialize Alicat serial driver.

        Args:
            name: Device identifier
            address: Serial port address (e.g., '/dev/ttyUSB0' or 'COM3')
            logger: Logger instance
            baudrate: Baud rate for communication (default: 19200)
            unit_id: Unit identifier for the Alicat device (default: 'A')
            dummy: If True, return dummy data instead of real device reads
        """
        self.name = name
        self._address = address
        self._baudrate = baudrate
        self._unit_id = unit_id
        self._logger = logger
        self._dummy = dummy

        self._serial: serial.Serial | None = None
        self._lock = threading.Lock()
        self.connected = False

    def _command(self, cmd: str, timeout: float = 0.5) -> str:
        """
        Send command and read response.

        Args:
            cmd: Command string (without unit ID prefix or CR)
            timeout: Read timeout in seconds

        Returns:
            Response string (stripped)
        """
        if self._serial is None:
            raise RuntimeError("Not connected")

        full_cmd = f"{self._unit_id}{cmd}\r"
        self._serial.reset_input_buffer()
        self._serial.write(full_cmd.encode("ascii"))

        # Read until CR
        self._serial.timeout = timeout
        response = self._serial.read_until(b"\r").decode("ascii").strip()

        if not response:
            raise TimeoutError(f"No response to command: {cmd}")

        return response

    def _parse_status_response(self, response: str) -> Dict[str, Any]:
        """
        Parse standard status response.

        Format: "{Unit} {Pressure} {Temperature} {Vol Flow} {Mass Flow} {Setpoint} {Gas}"
        """
        parts = response.split()
        if len(parts) < 7:
            raise ValueError(f"Invalid response format: {response}")

        return {
            "pressure": float(parts[1]),
            "temperature": float(parts[2]),
            "volumetric_flow": float(parts[3]),
            "mass_flow": float(parts[4]),
            "setpoint": float(parts[5]),
            "gas": parts[6] if len(parts) > 6 else "Unknown",
        }

    def initialize(self) -> None:
        """
        Establish serial connection to device.

        Raises:
            ConnectionError: If unable to connect (unless dummy mode)
        """
        if self._dummy:
            self._logger.info(f"[{self.name}] Running in dummy mode")
            self.connected = True
            return

        self._logger.info(f"[{self.name}] Connecting to {self._address} @ {self._baudrate} baud")

        with self._lock:
            try:
                self._serial = serial.Serial(
                    port=self._address,
                    baudrate=self._baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1.0,
                )
                # Verify connection with a read
                self._command("")
                self.connected = True
                self._logger.info(f"[{self.name}] Connected successfully")
            except Exception as e:
                self.close()
                raise ConnectionError(f"[{self.name}] Failed to connect: {e}")

    def close(self) -> None:
        """Clean up serial connection."""
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        self.connected = False

    def shutdown(self) -> None:
        """Close serial connection."""
        with self._lock:
            self.close()
        self._logger.info(f"[{self.name}] Disconnected")

    def read(self, include_pid: bool = False) -> Dict[str, Any]:
        """
        Read all device telemetry.

        Args:
            include_pid: If True, also read PID values.

        Returns:
            Dict containing device telemetry values
        """
        if self._dummy:
            return {
                "connected": True,
                "setpoint": 22.0,
                "valve_drive": 27.0,
                "pressure": 8.92,
                "temperature": 23.5,
                "volumetric_flow": 22.0,
                "mass_flow": 23.0,
            }

        with self._lock:
            try:
                response = self._command("")
                data = self._parse_status_response(response)

                result = {
                    "connected": self.connected,
                    "setpoint": data["setpoint"],
                    "pressure": data["pressure"],
                    "temperature": data["temperature"],
                    "volumetric_flow": data["volumetric_flow"],
                    "mass_flow": data["mass_flow"],
                    "valve_drive": 0.0,  # Not in standard response
                }

                if include_pid:
                    pid = self._read_pid_unlocked()
                    result.update(pid)

                return result

            except Exception as e:
                self._logger.error(f"[{self.name}] Error reading: {e}")
                self.connected = False
                raise

    def _read_pid_unlocked(self) -> Dict[str, int]:
        """Read PID values (caller must hold lock)."""
        try:
            p_resp = self._command("R21")  # P gain register
            d_resp = self._command("R22")  # D gain register
            i_resp = self._command("R23")  # I gain register

            # Response format: "{Unit} {Value}"
            return {
                "P": int(p_resp.split()[1]) if len(p_resp.split()) > 1 else 0,
                "D": int(d_resp.split()[1]) if len(d_resp.split()) > 1 else 0,
                "I": int(i_resp.split()[1]) if len(i_resp.split()) > 1 else 0,
            }
        except Exception as e:
            self._logger.error(f"[{self.name}] Error reading PID: {e}")
            return {"P": 0, "D": 0, "I": 0}

    def read_pid(self) -> Dict[str, int]:
        """Read PID controller parameters."""
        if self._dummy:
            return {"P": 50, "D": 120, "I": 0}

        with self._lock:
            return self._read_pid_unlocked()

    def set_flow_rate(self, flow_rate: float) -> Dict[str, Any]:
        """Set flow rate setpoint."""
        if self._dummy:
            self._logger.info(f"[{self.name}] Dummy: setpoint -> {flow_rate}")
            return {"status": "ok", "setpoint": flow_rate}

        with self._lock:
            try:
                self._command(f"S{flow_rate:.4f}")
                self._logger.info(f"[{self.name}] Setpoint set to {flow_rate} SLPM")
                return {"status": "ok", "setpoint": flow_rate}
            except Exception as e:
                self._logger.error(f"[{self.name}] Failed to set flow rate: {e}")
                raise

    def set_pressure(self, pressure: float) -> Dict[str, Any]:
        """Set pressure setpoint."""
        if self._dummy:
            self._logger.info(f"[{self.name}] Dummy: pressure -> {pressure}")
            return {"status": "ok", "pressure": pressure}

        with self._lock:
            try:
                self._command(f"P{pressure:.4f}")
                self._logger.info(f"[{self.name}] Pressure set to {pressure}")
                return {"status": "ok", "pressure": pressure}
            except Exception as e:
                self._logger.error(f"[{self.name}] Failed to set pressure: {e}")
                raise

    def set_gas(self, gas_type: str | int) -> Dict[str, Any]:
        """Set gas type by name or number."""
        if self._dummy:
            self._logger.info(f"[{self.name}] Dummy: gas -> {gas_type}")
            return {"status": "ok", "gas": gas_type}

        with self._lock:
            try:
                if isinstance(gas_type, str):
                    gas_num = self.GAS_TABLE.get(gas_type, 0)
                else:
                    gas_num = int(gas_type)

                self._command(f"G{gas_num}")
                self._logger.info(f"[{self.name}] Gas set to {gas_type}")
                return {"status": "ok", "gas": gas_type}
            except Exception as e:
                self._logger.error(f"[{self.name}] Failed to set gas: {e}")
                raise

    def set_pid(self, P: int = None, D: int = None, I: int = None) -> Dict[str, Any]:
        """Set PID controller parameters."""
        if self._dummy:
            return {"status": "ok", "P": P, "D": D, "I": I}

        with self._lock:
            try:
                results = {"status": "ok"}

                if P is not None:
                    self._command(f"W21={int(P)}")
                    results["P"] = P
                    self._logger.info(f"[{self.name}] P gain set to {P}")

                if D is not None:
                    self._command(f"W22={int(D)}")
                    results["D"] = D
                    self._logger.info(f"[{self.name}] D gain set to {D}")

                if I is not None:
                    self._command(f"W23={int(I)}")
                    results["I"] = I
                    self._logger.info(f"[{self.name}] I gain set to {I}")

                return results

            except Exception as e:
                self._logger.error(f"[{self.name}] Failed to set PID: {e}")
                raise

    def write(self, data: Dict[str, Any]) -> Any:
        """
        Execute a write command.

        Supports: setpoint, pressure, gas, P, D, I
        """
        if "setpoint" in data:
            return self.set_flow_rate(data["setpoint"])

        if "pressure" in data:
            return self.set_pressure(data["pressure"])

        if "gas" in data:
            return self.set_gas(data["gas"])

        if any(k in data for k in ("P", "D", "I")):
            return self.set_pid(
                P=data.get("P"),
                D=data.get("D"),
                I=data.get("I"),
            )

        return {"status": "no_op", "message": "No recognized command"}
