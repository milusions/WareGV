"""
motor_driver.py
Hardware abstraction layer for ST3215 servos.
Handles unit conversions and sign conventions (Anti-clockwise = Positive).
"""

import time
from python_st3215 import ST3215

class PortGroupDriver:
    def __init__(self, port, servo_ids, steps_per_rev=4096):
        self.port = port
        self.servo_ids = servo_ids
        self.steps_per_rev = steps_per_rev
        self.servos = {}
        self.original_modes = {}
        self.controller = ST3215(self.port)
        print(f"[PortGroupDriver INIT] Initialized for port: {self.port} with Servo IDs: {self.servo_ids}, Steps/Rev: {self.steps_per_rev}")

    def __enter__(self):
        """Context manager setup: initializes the controller and sets servos to continuous mode."""
        print(f"[PortGroupDriver ENTER] Entering context for port {self.port}...")
        self.controller.__enter__()
        
        for sid in self.servo_ids:
            servo = self.controller.wrap_servo(sid)
            self.servos[sid] = servo
            
            # Torque must be off before changing operating mode
            self.original_modes[sid] = servo.eeprom.read_operating_mode()
            
            servo.sram.torque_disable()
            servo.eeprom.write_operating_mode(1) # Mode 1 = Constant Speed
            servo.sram.torque_enable()
            servo.sram.write_running_speed(0)
            
        print(f"[PortGroupDriver ENTER] Port {self.port} successfully configured and ready.")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager teardown: stops servos and restores original operating modes."""
        print(f"[PortGroupDriver EXIT] Shutting down port {self.port}...")
        for sid in self.servo_ids:
            try:
                self.servos[sid].sram.write_running_speed(0)
                time.sleep(0.1) # Brief delay to ensure stop command registers
                self.servos[sid].sram.torque_disable()
                self.servos[sid].eeprom.write_operating_mode(self.original_modes[sid])
            except Exception as e:
                print(f"[PortGroupDriver ERROR] Error cleaning up Servo {sid} on {self.port}: {e}")
            
        self.controller.__exit__(exc_type, exc_val, exc_tb)
        print(f"[PortGroupDriver EXIT] Port {self.port} controller closed cleanly.")

    def set_rpm(self, servo_id, rpm):
        """
        Calculates step speed and updates the servo.
        ST3215 Spec: Bit 15=0 is CCW (Positive), Bit 15=1 is CW (Negative).
        """
        magnitude = int(abs(rpm) * self.steps_per_rev / 60)
        
        if magnitude == 0:
            encoded_steps = 0
        elif rpm > 0:
            # Positive RPM -> CCW -> Bit 15 is 0
            encoded_steps = magnitude
        else:
            # Negative RPM -> CW -> Bit 15 is 1
            # Passed as a signed negative integer so the library's [-32766, 32766] bounds-check accepts it
            encoded_steps = -32768 + magnitude
            if encoded_steps < -32766:
                encoded_steps = -32766 # Prevent library crash at near-zero speeds
                
        # print(f"[HW WRITE] Port: {self.port} | Servo {servo_id} -> Target RPM: {rpm:.2f} | Encoded Steps: {encoded_steps}")
        try:
            self.servos[servo_id].sram.write_running_speed(encoded_steps)
            return True # Successfully sent
        except Exception as e:
            print(f"[HW WRITE ERROR] Failed to write speed to Servo {servo_id} on {self.port}: {e}")
            return False # Failed to send

    def get_rpm(self, servo_id):
        """Reads current speed and converts to RPM. Positive is CCW, Negative is CW."""
        try:
            speed_steps = self.servos[servo_id].sram.read_current_speed()
            if speed_steps is not None:
                # The python_st3215 library unpacks this as a 16-bit signed int.
                # Mask it back to raw unsigned 16-bit to safely read Bit 15 without two's-complement weirdness.
                unsigned_val = speed_steps & 0xFFFF
                
                # Bit 15 indicates direction (1 = CW/Negative, 0 = CCW/Positive)
                if unsigned_val & 0x8000:
                    magnitude = unsigned_val & 0x7FFF
                    rpm = -(magnitude * 60) / self.steps_per_rev
                else:
                    magnitude = unsigned_val
                    rpm = (magnitude * 60) / self.steps_per_rev
                    
                return rpm
        except Exception as e:
            print(f"[HW READ ERROR] Failed to read speed from Servo {servo_id} on {self.port}: {e}")
        return 0.0