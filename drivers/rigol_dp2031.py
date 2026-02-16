#!/usr/bin/env python3
"""
Rigol DP2301 Power Supply Driver
"""

import pyvisa
import time
from typing import Optional, Tuple, Union
import logging


class RigolDP2301Error(Exception):
    """Custom exception for DP2301 communication errors."""
    pass


class RigolDP2301:
    """
    Driver for Rigol DP2301 series power supplies.

    """
    
    MAX_CHANNELS = 3
    COMMAND_DELAY = 0.01  
    TIMEOUT_MS = 5000  
    
    def __init__(self, resource_string: str):
        """
        Initialize the driver.
        
        Args:
            resource_string: VISA resource string (e.g., 'TCPIP0::10.0.5.106::INSTR')
        """
        self.resource_string = resource_string
        self.instrument: Optional[pyvisa.Resource] = None
        self.is_connected = False

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
    
    def connect(self) -> None:
        """
        Connect to the power supply.
        
        Raises:
            RigolDP2301Error: If connection fails
        """
        try:
            rm = pyvisa.ResourceManager()
            self.instrument = rm.open_resource(self.resource_string)
            self.instrument.timeout = self.TIMEOUT_MS
            self.is_connected = True 
            
            idn = self.query('*IDN?')
            self.logger.info(f"Connected to: {idn.strip()}")
            
            self.write('*CLS')
            
        except Exception as e:
            self.is_connected = False
            if self.instrument is not None:
                try:
                    self.instrument.close()
                except:
                    pass
                self.instrument = None
            raise RigolDP2301Error(f"Failed to connect: {str(e)}")
    
    def disconnect(self) -> None:
        """Disconnect from the power supply."""
        if self.instrument is not None:
            try:
                self.instrument.close()
            except Exception:
                pass  # Ignore errors during disconnect
            finally:
                self.instrument = None
                self.is_connected = False
    
    def write(self, command: str) -> None:
        """
        Send a SCPI command to the instrument.
        
        Args:
            command: SCPI command string
            
        Raises:
            RigolDP2301Error: If not connected or write fails
        """
        if not self.is_connected or self.instrument is None:
            raise RigolDP2301Error("Not connected to instrument")
        
        try:
            self.instrument.write(command)
            time.sleep(self.COMMAND_DELAY)
        except Exception as e:
            raise RigolDP2301Error(f"Command failed '{command}': {str(e)}")
    
    def query(self, command: str) -> str:
        """
        Send a SCPI query and return the response.
        
        Args:
            command: SCPI query string
            
        Returns:
            Response from instrument
            
        Raises:
            RigolDP2301Error: If not connected or query fails
        """
        if not self.is_connected or self.instrument is None:
            raise RigolDP2301Error("Not connected to instrument")
        
        try:
            response = self.instrument.query(command)
            time.sleep(self.COMMAND_DELAY)
            return response
        except Exception as e:
            raise RigolDP2301Error(f"Query failed '{command}': {str(e)}")
    
    def _validate_channel(self, channel: int) -> None:
        """
        Validate channel number.
        
        Args:
            channel: Channel number (1-3)
            
        Raises:
            RigolDP2301Error: If channel is invalid
        """
        if not isinstance(channel, int) or channel < 1 or channel > self.MAX_CHANNELS:
            raise RigolDP2301Error(f"Invalid channel: {channel}. Must be 1-{self.MAX_CHANNELS}")
    
    def set_voltage(self, channel: int, voltage: float) -> None:
        """
        Set the voltage for a specified channel.
        
        Args:
            channel: Channel number (1-3)
            voltage: Voltage in volts
            
        Raises:
            RigolDP2301Error: If channel is invalid or command fails
        """
        self._validate_channel(channel)
        
        command = f":SOUR{channel}:VOLT {voltage:.3f}"
        self.write(command)
        
        self.logger.debug(f"Set CH{channel} voltage to {voltage:.3f}V")
    
    def set_current_limit(self, channel: int, current: float) -> None:
        """
        Set the current limit for a specified channel.
        
        Args:
            channel: Channel number (1-3) 
            current: Current in amperes
            
        Raises:
            RigolDP2301Error: If channel is invalid or command fails
        """
        self._validate_channel(channel)
        
        command = f":SOUR{channel}:CURR {current:.3f}"
        self.write(command)
        
        self.logger.debug(f"Set CH{channel} current limit to {current:.3f}A")
    
    def get_voltage_setting(self, channel: int) -> float:
        """
        Get the voltage setting for a specified channel.
        
        Args:
            channel: Channel number (1-3)
            
        Returns:
            Voltage setting in volts
            
        Raises:
            RigolDP2301Error: If channel is invalid or query fails
        """
        self._validate_channel(channel)
        
        command = f":SOUR{channel}:VOLT?"
        response = self.query(command)
        
        try:
            voltage = float(response.strip())
            return voltage
        except ValueError as e:
            raise RigolDP2301Error(f"Invalid voltage response: {response}")
    
    def get_current_setting(self, channel: int) -> float:
        """
        Get the current limit setting for a specified channel.
        
        Args:
            channel: Channel number (1-3)
            
        Returns:
            Current setting in amperes
            
        Raises:
            RigolDP2301Error: If channel is invalid or query fails
        """
        self._validate_channel(channel)
        
        command = f":SOUR{channel}:CURR?"
        response = self.query(command)
        
        try:
            current = float(response.strip())
            return current
        except ValueError as e:
            raise RigolDP2301Error(f"Invalid current response: {response}")
    
    def read_voltage(self, channel: int) -> float:
        """
        Read the actual voltage output from a specified channel.
        
        Args:
            channel: Channel number (1-3)
            
        Returns:
            Voltage in volts
            
        Raises:
            RigolDP2301Error: If channel is invalid or query fails
        """
        self._validate_channel(channel)
        
        command = f":MEAS:VOLT? CH{channel}"
        response = self.query(command)
        
        try:
            voltage = float(response.strip())
            return voltage
        except ValueError as e:
            raise RigolDP2301Error(f"Invalid voltage measurement: {response}")
    
    def read_current(self, channel: int) -> float:
        """
        Read the actual current output from a specified channel.
        
        Args:
            channel: Channel number (1-3)
            
        Returns:
            Current in amperes
            
        Raises:
            RigolDP2301Error: If channel is invalid or query fails
        """
        self._validate_channel(channel)
        
        command = f":MEAS:CURR? CH{channel}"
        response = self.query(command)
        
        try:
            current = float(response.strip())
            return current
        except ValueError as e:
            raise RigolDP2301Error(f"Invalid current measurement: {response}")
    
    def read_power(self, channel: int) -> float:
        """
        Read the actual power output from a specified channel.
        
        Args:
            channel: Channel number (1-3)
            
        Returns:
            Power in watts
            
        Raises:
            RigolDP2301Error: If channel is invalid or query fails
        """
        self._validate_channel(channel)
        
        command = f":MEAS:POW? CH{channel}"
        response = self.query(command)
        
        try:
            power = float(response.strip())
            return power
        except ValueError as e:
            raise RigolDP2301Error(f"Invalid power measurement: {response}")
    
    def read_measurements(self, channel: int) -> Tuple[float, float, float]:
        """
        Read voltage, current, and power from a specified channel in one operation.
        
        Args:
            channel: Channel number (1-3)
            
        Returns:
            Tuple of (voltage, current, power)
            
        Raises:
            RigolDP2301Error: If channel is invalid or query fails
        """
        self._validate_channel(channel)
        
        command = f":MEAS:ALL? CH{channel}"
        response = self.query(command)
        
        try:
            values = response.strip().split(',')
            if len(values) != 3:
                raise ValueError(f"Expected 3 values, got {len(values)}")
            
            voltage = float(values[0])
            current = float(values[1])
            power = float(values[2])
            
            return voltage, current, power
            
        except (ValueError, IndexError) as e:
            raise RigolDP2301Error(f"Invalid measurement response: {response}")
    
    def enable_output(self, channel: int, enabled: bool) -> None:
        """
        Enable or disable output for a specified channel.
        
        Args:
            channel: Channel number (1-3)
            enabled: True to enable, False to disable
            
        Raises:
            RigolDP2301Error: If channel is invalid or command fails
        """
        self._validate_channel(channel)
        
        state = "ON" if enabled else "OFF"
        command = f":OUTP CH{channel},{state}"
        self.write(command)
        
        self.logger.info(f"CH{channel} output {'enabled' if enabled else 'disabled'}")
    
    def is_output_enabled(self, channel: int) -> bool:
        """
        Check if output is enabled for a specified channel.
        
        Args:
            channel: Channel number (1-3)
            
        Returns:
            True if enabled, False if disabled
            
        Raises:
            RigolDP2301Error: If channel is invalid or query fails
        """
        self._validate_channel(channel)
        
        command = f":OUTP? CH{channel}"
        response = self.query(command)
        
        try:
            return bool(int(response.strip()))
        except ValueError as e:
            raise RigolDP2301Error(f"Invalid output state response: {response}")
    
    
    def get_error(self) -> str:
        """
        Get the next error from the instrument error queue.
        
        Returns:
            Error string from instrument
        """
        try:
            return self.query(":SYST:ERR?")
        except Exception:
            return "Unable to query error"
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()


# Example usage and testing
if __name__ == "__main__":
    """
    Example usage of the RigolDP2301 driver.
    """
    
    # Configuration  
    POWER_SUPPLY_IP = "10.0.5.106"  # Update with your IP
    RESOURCE_STRING = f"TCPIP0::{POWER_SUPPLY_IP}::INSTR"
    
    try:
        # Using context manager (recommended)
        with RigolDP2301(RESOURCE_STRING) as ps:
            print("=== Rigol DP2301 Driver Test ===")
            
            # Test channel 1
            channel = 1
            
            # Set initial conditions
            print(f"\nSetting up CH{channel}:")
            ps.set_voltage(channel, 5.0)
            ps.set_current_limit(channel, 1.0)
            
            print(f"Voltage setting: {ps.get_voltage_setting(channel):.3f}V")
            print(f"Current setting: {ps.get_current_setting(channel):.3f}A")
            
            # Enable output
            ps.enable_output(channel, True)
            print(f"Output enabled: {ps.is_output_enabled(channel)}")
            
            # Read measurements
            print(f"\nMeasurements from CH{channel}:")
            voltage = ps.read_voltage(channel)
            current = ps.read_current(channel)
            power = ps.read_power(channel)
            
            print(f"Voltage: {voltage:.3f}V")
            print(f"Current: {current:.3f}A") 
            print(f"Power: {power:.3f}W")
            
            # Alternative: read all at once
            v, i, p = ps.read_measurements(channel)
            print(f"\nAll measurements: V={v:.3f}V, I={i:.3f}A, P={p:.3f}W")
            
            # Disable output
            ps.enable_output(channel, False)
            print(f"\nOutput disabled: {not ps.is_output_enabled(channel)}")
            
    except RigolDP2301Error as e:
        print(f"Power supply error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")