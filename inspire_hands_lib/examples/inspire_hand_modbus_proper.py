#!/usr/bin/env python3
import serial
import time
import struct

class InspireHandModbus:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, slave_id=1):
        """Initialize connection to the Inspire Hand using Modbus RTU protocol."""
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.ser = None
        
    def connect(self):
        """Connect to the hand."""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            
            if self.ser.is_open:
                print(f"Connected to Inspire Hand on {self.port}")
                return True
            return False
        except Exception as e:
            print(f"Error connecting to hand: {e}")
            return False
            
    def disconnect(self):
        """Disconnect from the hand."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected from Inspire Hand")
            
    def _calculate_crc(self, data):
        """Calculate Modbus CRC16."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
        # Return CRC as bytes (little-endian)
        return crc.to_bytes(2, byteorder='little')
            
    def read_holding_registers(self, address, num_registers=1):
        """
        Read holding registers (Modbus function code 0x03).
        
        Args:
            address: Starting register address
            num_registers: Number of registers to read
            
        Returns:
            List of register values if successful, None otherwise
        """
        if not self.ser or not self.ser.is_open:
            print("Not connected to hand")
            return None
            
        # Create Modbus RTU frame for reading registers
        packet = bytearray([
            self.slave_id,      # Slave ID
            0x03,               # Function code: Read Holding Registers
            (address >> 8) & 0xFF,  # Address high byte
            address & 0xFF,         # Address low byte
            (num_registers >> 8) & 0xFF,  # Number of registers high byte
            num_registers & 0xFF         # Number of registers low byte
        ])
        
        # Add CRC
        packet.extend(self._calculate_crc(packet))
        
        # Send packet
        print(f"Sending read request: {' '.join([f'0x{b:02X}' for b in packet])}")
        
        try:
            self.ser.write(packet)
            time.sleep(0.2)  # Wait for response
            
            # Check for response
            if self.ser.in_waiting:
                response = self.ser.read(self.ser.in_waiting)
                print(f"Response: {' '.join([f'0x{b:02X}' for b in response])}")
                
                # Parse the response
                if len(response) >= 5 and response[0] == self.slave_id and response[1] == 0x03:
                    byte_count = response[2]
                    values = []
                    
                    for i in range(0, byte_count, 2):
                        if i + 3 < len(response):
                            value = (response[i+3] << 8) | response[i+4]
                            values.append(value)
                            
                    return values
                else:
                    print("Invalid response")
                    return None
            else:
                print("No response received")
                return None
        except Exception as e:
            print(f"Error reading registers: {e}")
            return None
    
    def write_single_register(self, address, value):
        """
        Write to a single holding register (Modbus function code 0x06).
        
        Args:
            address: Register address
            value: Value to write (0-65535)
            
        Returns:
            True if successful, False otherwise
        """
        if not self.ser or not self.ser.is_open:
            print("Not connected to hand")
            return False
            
        # Create Modbus RTU frame for writing a register
        packet = bytearray([
            self.slave_id,      # Slave ID
            0x06,               # Function code: Write Single Register
            (address >> 8) & 0xFF,  # Address high byte
            address & 0xFF,         # Address low byte
            (value >> 8) & 0xFF,    # Value high byte
            value & 0xFF            # Value low byte
        ])
        
        # Add CRC
        packet.extend(self._calculate_crc(packet))
        
        # Send packet
        print(f"Sending write request: {' '.join([f'0x{b:02X}' for b in packet])}")
        
        try:
            self.ser.write(packet)
            time.sleep(0.2)  # Wait for response
            
            # Check for response
            if self.ser.in_waiting:
                response = self.ser.read(self.ser.in_waiting)
                print(f"Response: {' '.join([f'0x{b:02X}' for b in response])}")
                
                # Check if response is valid
                if len(response) >= 8 and response[0] == self.slave_id and response[1] == 0x06:
                    return True
                else:
                    print("Invalid response")
                    return False
            else:
                print("No response received")
                return False
        except Exception as e:
            print(f"Error writing register: {e}")
            return False
    
    def write_multiple_registers(self, address, values):
        """
        Write to multiple holding registers (Modbus function code 0x10).
        
        Args:
            address: Starting register address
            values: List of values to write
            
        Returns:
            True if successful, False otherwise
        """
        if not self.ser or not self.ser.is_open:
            print("Not connected to hand")
            return False
            
        # Create Modbus RTU frame for writing multiple registers
        num_registers = len(values)
        byte_count = num_registers * 2
        
        packet = bytearray([
            self.slave_id,      # Slave ID
            0x10,               # Function code: Write Multiple Registers
            (address >> 8) & 0xFF,  # Address high byte
            address & 0xFF,         # Address low byte
            (num_registers >> 8) & 0xFF,  # Number of registers high byte
            num_registers & 0xFF,         # Number of registers low byte
            byte_count               # Byte count
        ])
        
        # Add data values
        for value in values:
            packet.append((value >> 8) & 0xFF)  # Value high byte
            packet.append(value & 0xFF)         # Value low byte
        
        # Add CRC
        packet.extend(self._calculate_crc(packet))
        
        # Send packet
        print(f"Sending write multiple request: {' '.join([f'0x{b:02X}' for b in packet])}")
        
        try:
            self.ser.write(packet)
            time.sleep(0.2)  # Wait for response
            
            # Check for response
            if self.ser.in_waiting:
                response = self.ser.read(self.ser.in_waiting)
                print(f"Response: {' '.join([f'0x{b:02X}' for b in response])}")
                
                # Check if response is valid
                if len(response) >= 8 and response[0] == self.slave_id and response[1] == 0x10:
                    return True
                else:
                    print("Invalid response")
                    return False
            else:
                print("No response received")
                return False
        except Exception as e:
            print(f"Error writing registers: {e}")
            return False
    
    # High-level control functions based on the manual
    
    def open_all_fingers(self):
        """Open all fingers (set all angles to 1000)."""
        print("\n=== Opening all fingers ===")
        # ANGLE_SET(m) registers start at address 1486 (0x05CE)
        # Set all 6 angles to 1000 (fully open)
        values = [1000, 1000, 1000, 1000, 1000, 1000]
        return self.write_multiple_registers(1486, values)
    
    def close_all_fingers(self):
        """Close all fingers (set all angles to 0)."""
        print("\n=== Closing all fingers ===")
        # ANGLE_SET(m) registers start at address 1486 (0x05CE)
        # Set all 6 angles to 0 (fully closed)
        values = [0, 0, 0, 0, 0, 0]
        return self.write_multiple_registers(1486, values)
    
    def set_finger_angle(self, finger_id, angle):
        """
        Set the angle of a specific finger.
        
        Args:
            finger_id: Finger ID (0-5)
                0: Little finger
                1: Ring finger
                2: Middle finger
                3: Index finger
                4: Thumb bending
                5: Thumb rotation
            angle: Angle value (0-1000)
                0: Fully closed
                1000: Fully open
        """
        if not 0 <= finger_id <= 5:
            print("Invalid finger ID. Must be 0-5.")
            return False
            
        if not 0 <= angle <= 1000:
            print("Invalid angle. Must be 0-1000.")
            return False
        
        print(f"\n=== Setting finger {finger_id} to angle {angle} ===")
        # ANGLE_SET(m) registers start at address 1486 (0x05CE)
        register_address = 1486 + (finger_id * 2)
        return self.write_single_register(register_address, angle)
    
    def set_all_finger_speeds(self, speed):
        """
        Set the speed for all fingers.
        
        Args:
            speed: Speed value (0-1000)
        """
        if not 0 <= speed <= 1000:
            print("Invalid speed. Must be 0-1000.")
            return False
        
        print(f"\n=== Setting all finger speeds to {speed} ===")
        # SPEED_SET(m) registers start at address 1522 (0x05F2)
        values = [speed] * 6
        return self.write_multiple_registers(1522, values)
    
    def set_all_finger_forces(self, force):
        """
        Set the force threshold for all fingers.
        
        Args:
            force: Force threshold value (0-1000)
        """
        if not 0 <= force <= 1000:
            print("Invalid force. Must be 0-1000.")
            return False
        
        print(f"\n=== Setting all finger force thresholds to {force} ===")
        # FORCE_SET(m) registers start at address 1498 (0x05DA)
        values = [force] * 6
        return self.write_multiple_registers(1498, values)
    
    def read_finger_angles(self):
        """Read the actual angles of all fingers."""
        print("\n=== Reading actual finger angles ===")
        # ANGLE_ACT(m) registers start at address 1546 (0x060A)
        # 6 DOF × 2 bytes = 12 registers
        values = self.read_holding_registers(1546, 6)
        if values:
            print("Finger angles:")
            print(f"  Little finger: {values[0]}")
            print(f"  Ring finger: {values[1]}")
            print(f"  Middle finger: {values[2]}")
            print(f"  Index finger: {values[3]}")
            print(f"  Thumb bending: {values[4]}")
            print(f"  Thumb rotation: {values[5]}")
        return values
    
    def read_finger_forces(self):
        """Read the actual forces applied to all fingers."""
        print("\n=== Reading actual finger forces ===")
        # FORCE_ACT(m) registers start at address 1582 (0x062E)
        # 6 DOF × 2 bytes = 12 registers
        values = self.read_holding_registers(1582, 6)
        if values:
            print("Finger forces:")
            print(f"  Little finger: {values[0]} g")
            print(f"  Ring finger: {values[1]} g")
            print(f"  Middle finger: {values[2]} g")
            print(f"  Index finger: {values[3]} g")
            print(f"  Thumb bending: {values[4]} g")
            print(f"  Thumb rotation: {values[5]} g")
        return values

def main():
    # Create hand controller
    controller = InspireHandModbus()
    
    # Connect to hand
    if controller.connect():
        try:
            # Menu interface
            while True:
                print("\nInspire Hand Control Menu:")
                print("1. Open all fingers")
                print("2. Close all fingers")
                print("3. Set finger angle")
                print("4. Set all finger speeds")
                print("5. Set all finger forces")
                print("6. Read finger angles")
                print("7. Read finger forces")
                print("0. Exit")
                
                choice = input("Enter your choice (0-7): ")
                
                if choice == "1":
                    controller.open_all_fingers()
                elif choice == "2":
                    controller.close_all_fingers()
                elif choice == "3":
                    finger_names = ["Little finger", "Ring finger", "Middle finger", 
                                   "Index finger", "Thumb bending", "Thumb rotation"]
                    print("\nSelect finger:")
                    for i, name in enumerate(finger_names):
                        print(f"{i}. {name}")
                    
                    finger_id = int(input("Enter finger ID (0-5): "))
                    if 0 <= finger_id <= 5:
                        angle = int(input("Enter angle (0-1000, 0=closed, 1000=open): "))
                        controller.set_finger_angle(finger_id, angle)
                    else:
                        print("Invalid finger ID")
                elif choice == "4":
                    speed = int(input("Enter speed for all fingers (0-1000): "))
                    controller.set_all_finger_speeds(speed)
                elif choice == "5":
                    force = int(input("Enter force threshold for all fingers (0-1000): "))
                    controller.set_all_finger_forces(force)
                elif choice == "6":
                    controller.read_finger_angles()
                elif choice == "7":
                    controller.read_finger_forces()
                elif choice == "0":
                    break
                else:
                    print("Invalid choice. Please try again.")
                    
                time.sleep(1)
                
        finally:
            # Always disconnect
            controller.disconnect()
    else:
        print("Failed to connect to Inspire Hand")

if __name__ == "__main__":
    main() 