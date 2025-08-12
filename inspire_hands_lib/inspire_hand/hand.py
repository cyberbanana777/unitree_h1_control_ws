"""
Main module for controlling the Inspire Hand RH56dfq.
"""

from enum import IntEnum
from typing import List, Optional, Dict, Union, Tuple
import time
from contextlib import contextmanager

from .modbus import ModbusClient
from .exceptions import InspireHandError, ConnectionError, CommandError


class Register:
    """Register addresses for the Inspire Hand."""
    # System information
    HAND_ID = 1000        # Dexterous Hand ID
    REDU_RATIO = 1002     # Baud rate setting
    CLEAR_ERROR = 1004    # Error clearance
    SAVE = 1005           # Saving data in Flash
    RESET_PARA = 1006     # Restoring factory defaults
    FORCE_SENSOR_CALIB = 1009  # Force sensor calibration
    
    # Current protection registers
    CURRENT_LIMIT = 1020  # Actuator current protection values (12 bytes)
    
    # Default settings
    DEFAULT_SPEED_SET = 1032   # Power-on speed values (12 bytes)
    DEFAULT_FORCE_SET = 1044   # Power-on force control threshold values (12 bytes)
    
    # Control registers
    POS_SET = 1474        # Actuator position values (12 bytes)
    ANGLE_SET = 1486      # Angle values (12 bytes)
    FORCE_SET = 1498      # Force control threshold values (12 bytes)
    SPEED_SET = 1522      # Speed values (12 bytes)
    
    # Status registers
    POS_ACT = 1534        # Actual actuator position values (12 bytes)
    ANGLE_ACT = 1546      # Actual angle values (12 bytes)
    FORCE_ACT = 1582      # Actual force values (12 bytes)
    CURRENT = 1594        # Actuator current values (12 bytes)
    ERROR = 1606          # Error codes (6 bytes)
    STATUS = 1612         # Status information (6 bytes)
    TEMP = 1618           # Actuator temperature values (6 bytes)


class FingerID(IntEnum):
    """Finger IDs for the Inspire Hand."""
    LITTLE = 0
    RING = 1
    MIDDLE = 2
    INDEX = 3
    THUMB_BEND = 4
    THUMB_ROTATE = 5
    ALL = 6  # Special ID for controlling all fingers at once


class FingerStatus(IntEnum):
    """Status codes for fingers."""
    UNCLENCHING = 0
    GRASPING = 1
    REACHED_TARGET = 2
    REACHED_FORCE = 3
    CURRENT_PROTECTION = 5
    LOCKED_ROTOR = 6
    FAULT = 7


class ErrorCode:
    """Error codes for the actuators."""
    LOCKED_ROTOR = 0x01       # Bit 0
    OVER_TEMPERATURE = 0x02   # Bit 1
    OVER_CURRENT = 0x04       # Bit 2
    ABNORMAL_OPERATION = 0x08 # Bit 3
    COMMUNICATION_ERROR = 0x10 # Bit 4


class Finger:
    """Represents a single finger on the Inspire Hand."""
    
    def __init__(self, hand, finger_id: int):
        """
        Initialize a finger.
        
        Args:
            hand: Parent InspireHand object
            finger_id: ID of this finger
        """
        self.hand = hand
        self.id = finger_id
        self._name = FingerID(finger_id).name.lower().replace('_', ' ')
    
    @property
    def name(self) -> str:
        """Get the name of this finger."""
        return self._name
    
    @property
    def angle(self) -> int:
        """Get the current angle of this finger."""
        angles = self.hand.get_finger_angles()
        return angles[self.id]
    
    @angle.setter
    def angle(self, value: int) -> None:
        """
        Set the angle of this finger.
        
        Args:
            value: Angle value (0-1000, 0=closed, 1000=open)
        """
        self.hand.set_finger_angle(self.id, value)
    
    @property
    def force(self) -> int:
        """Get the current force applied by this finger."""
        forces = self.hand.get_finger_forces()
        return forces[self.id]
    
    @property
    def status(self) -> FingerStatus:
        """Get the current status of this finger."""
        statuses = self.hand.get_finger_statuses()
        return statuses[self.id]
    
    @property
    def error(self) -> int:
        """Get the current error code of this finger."""
        errors = self.hand.get_finger_errors()
        return errors[self.id]
    
    @property
    def temperature(self) -> int:
        """Get the current temperature of this finger's actuator."""
        temps = self.hand.get_finger_temperatures()
        return temps[self.id]
    
    def open(self) -> None:
        """Open this finger (set angle to 1000)."""
        self.angle = 1000
    
    def close(self) -> None:
        """Close this finger (set angle to 0)."""
        self.angle = 0
    
    def move(self, angle: int) -> None:
        """
        Move this finger to the specified angle.
        
        Args:
            angle: Angle value (0-1000, 0=closed, 1000=open)
        """
        self.angle = angle


class InspireHand:
    """Interface for controlling the Inspire Hand RH56dfq."""
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200, slave_id: int = 1, debug: bool = False):
        """
        Initialize the Inspire Hand interface.
        
        Args:
            port: Serial port path
            baudrate: Serial baudrate
            slave_id: Modbus slave ID of the hand
            debug: Whether to print debug information
        """
        self.modbus = ModbusClient(port, baudrate, slave_id)
        self.modbus.debug = debug
        
        # Create finger objects
        self.little_finger = Finger(self, FingerID.LITTLE)
        self.ring_finger = Finger(self, FingerID.RING)
        self.middle_finger = Finger(self, FingerID.MIDDLE)
        self.index_finger = Finger(self, FingerID.INDEX)
        self.thumb_bend = Finger(self, FingerID.THUMB_BEND)
        self.thumb_rotate = Finger(self, FingerID.THUMB_ROTATE)
        
        # Create a list of all fingers for easy iteration
        self.fingers = [
            self.little_finger,
            self.ring_finger,
            self.middle_finger,
            self.index_finger,
            self.thumb_bend,
            self.thumb_rotate
        ]
        
        self._connected = False
    
    @contextmanager
    def connect(self):
        """Context manager for connecting to the hand."""
        try:
            self.open()
            yield self
        finally:
            self.close()
    
    def open(self) -> None:
        """Connect to the hand."""
        if not self._connected:
            self.modbus.connect()
            self._connected = True
    
    def close(self) -> None:
        """Disconnect from the hand."""
        if self._connected:
            self.modbus.disconnect()
            self._connected = False
    
    @property
    def is_connected(self) -> bool:
        """Check if the hand is connected."""
        return self._connected
    
    def _check_connection(self) -> None:
        """Check if the hand is connected, raise an exception if not."""
        if not self._connected:
            raise ConnectionError("Not connected to hand")
    
    # Basic control functions
    
    def open_all_fingers(self) -> None:
        """Open all fingers (set all angles to 1000)."""
        self._check_connection()
        values = [1000] * 6
        self.modbus.write_multiple_registers(Register.ANGLE_SET, values)
    
    def close_all_fingers(self) -> None:
        """Close all fingers (set all angles to 0)."""
        self._check_connection()
        values = [0] * 6
        self.modbus.write_multiple_registers(Register.ANGLE_SET, values)
    
    def set_finger_angle(self, finger_id: int, angle: int) -> None:
        """
        Set the angle of a specific finger.
        
        Args:
            finger_id: Finger ID (0-5)
            angle: Angle value (0-1000, 0=closed, 1000=open)
        """
        self._check_connection()
        
        if not 0 <= finger_id <= 5:
            raise ValueError("Invalid finger ID. Must be 0-5.")
            
        if not 0 <= angle <= 1000:
            raise ValueError("Invalid angle. Must be 0-1000.")
        
        register_address = Register.ANGLE_SET + (finger_id * 2)
        self.modbus.write_single_register(register_address, angle)
    
    def set_all_finger_speeds(self, speed: int) -> None:
        """
        Set the speed for all fingers.
        
        Args:
            speed: Speed value (0-1000)
        """
        self._check_connection()
        
        if not 0 <= speed <= 1000:
            raise ValueError("Invalid speed. Must be 0-1000.")
        
        values = [speed] * 6
        self.modbus.write_multiple_registers(Register.SPEED_SET, values)
    
    def set_finger_speed(self, finger_id: int, speed: int) -> None:
        """
        Set the speed of a specific finger.
        
        Args:
            finger_id: Finger ID (0-5)
            speed: Speed value (0-1000)
        """
        self._check_connection()
        
        if not 0 <= finger_id <= 5:
            raise ValueError("Invalid finger ID. Must be 0-5.")
            
        if not 0 <= speed <= 1000:
            raise ValueError("Invalid speed. Must be 0-1000.")
        
        register_address = Register.SPEED_SET + (finger_id * 2)
        self.modbus.write_single_register(register_address, speed)
    
    def set_all_finger_forces(self, force: int) -> None:
        """
        Set the force threshold for all fingers.
        
        Args:
            force: Force threshold value (0-1000)
        """
        self._check_connection()
        
        if not 0 <= force <= 1000:
            raise ValueError("Invalid force. Must be 0-1000.")
        
        values = [force] * 6
        self.modbus.write_multiple_registers(Register.FORCE_SET, values)
    
    def set_finger_force(self, finger_id: int, force: int) -> None:
        """
        Set the force threshold of a specific finger.
        
        Args:
            finger_id: Finger ID (0-5)
            force: Force threshold value (0-1000)
        """
        self._check_connection()
        
        if not 0 <= finger_id <= 5:
            raise ValueError("Invalid finger ID. Must be 0-5.")
            
        if not 0 <= force <= 1000:
            raise ValueError("Invalid force. Must be 0-1000.")
        
        register_address = Register.FORCE_SET + (finger_id * 2)
        self.modbus.write_single_register(register_address, force)
    
    # Status functions
    
    def get_finger_angles(self) -> List[int]:
        """
        Read the actual angles of all fingers.
        
        Returns:
            List of angles for all fingers
        """
        self._check_connection()
        return self.modbus.read_holding_registers(Register.ANGLE_ACT, 6)
    
    def get_finger_forces(self) -> List[int]:
        """
        Read the actual forces applied by all fingers.
        
        Returns:
            List of forces for all fingers
        """
        self._check_connection()
        return self.modbus.read_holding_registers(Register.FORCE_ACT, 6)
    
    def get_finger_statuses(self) -> List[FingerStatus]:
        """
        Read the status of all fingers.
        
        Returns:
            List of finger status enums
        """
        self._check_connection()
        # STATUS registers are single byte each, but we have to read them as 16-bit registers
        values = self.modbus.read_holding_registers(Register.STATUS, 3)
        # Extract the status bytes from the 16-bit values
        statuses = []
        for value in values:
            statuses.append(FingerStatus(value & 0xFF))
            statuses.append(FingerStatus((value >> 8) & 0xFF))
        return statuses[:6]  # Return only the 6 finger statuses
    
    def get_finger_errors(self) -> List[int]:
        """
        Read the error codes of all fingers.
        
        Returns:
            List of error codes for all fingers
        """
        self._check_connection()
        # ERROR registers are single byte each, but we have to read them as 16-bit registers
        values = self.modbus.read_holding_registers(Register.ERROR, 3)
        # Extract the error bytes from the 16-bit values
        errors = []
        for value in values:
            errors.append(value & 0xFF)
            errors.append((value >> 8) & 0xFF)
        return errors[:6]  # Return only the 6 finger errors
    
    def get_finger_temperatures(self) -> List[int]:
        """
        Read the temperature of all finger actuators.
        
        Returns:
            List of temperatures for all fingers
        """
        self._check_connection()
        # TEMP registers are single byte each, but we have to read them as 16-bit registers
        values = self.modbus.read_holding_registers(Register.TEMP, 3)
        # Extract the temperature bytes from the 16-bit values
        temps = []
        for value in values:
            temps.append(value & 0xFF)
            temps.append((value >> 8) & 0xFF)
        return temps[:6]  # Return only the 6 finger temperatures
    
    # Utility functions
    
    def reset(self) -> None:
        """Reset the hand (clear errors)."""
        self._check_connection()
        self.modbus.write_single_register(Register.CLEAR_ERROR, 1)
    
    def calibrate_force_sensors(self) -> None:
        """
        Calibrate the force sensors.
        
        Note:
            This takes about 6 seconds and the hand must be maintained in a no-load state
            (fingers cannot touch any object) during calibration.
        """
        self._check_connection()
        self.modbus.write_single_register(Register.FORCE_SENSOR_CALIB, 1)
        time.sleep(6)  # Wait for calibration to complete
    
    def save_settings(self) -> None:
        """Save the current settings to flash memory."""
        self._check_connection()
        self.modbus.write_single_register(Register.SAVE, 1)
    
    def restore_factory_defaults(self) -> None:
        """Restore factory default settings."""
        self._check_connection()
        self.modbus.write_single_register(Register.RESET_PARA, 1)
    
    # Gesture functions
    
    def pinch(self, force: int = 500) -> None:
        """
        Make a pinch gesture (thumb and index finger).
        
        Args:
            force: Force threshold (0-1000)
        """
        self._check_connection()
        
        # Open all fingers
        self.open_all_fingers()
        time.sleep(1)
        
        # Set force thresholds
        self.set_all_finger_forces(force)
        
        # Close thumb and index finger
        self.thumb_bend.close()
        self.index_finger.close()
    
    def point(self) -> None:
        """Make a pointing gesture (index finger extended, others closed)."""
        self._check_connection()
        
        # First open all fingers
        self.open_all_fingers()
        time.sleep(0.5)
        
        # Close all fingers except index
        self.little_finger.close()
        self.ring_finger.close()
        self.middle_finger.close()
        self.thumb_bend.close()
    
    def thumbs_up(self) -> None:
        """Make a thumbs up gesture."""
        self._check_connection()
        
        # First open all fingers
        self.open_all_fingers()
        time.sleep(0.5)
        
        # Close all fingers except thumb
        self.little_finger.close()
        self.ring_finger.close()
        self.middle_finger.close()
        self.index_finger.close()
        
        # Adjust thumb position
        self.thumb_rotate.angle = 500  # Midway rotation
    
    def grip(self, force: int = 500) -> None:
        """
        Make a grip gesture (close all fingers).
        
        Args:
            force: Force threshold (0-1000)
        """
        self._check_connection()
        
        # Set force thresholds
        self.set_all_finger_forces(force)
        
        # Close all fingers
        self.close_all_fingers() 