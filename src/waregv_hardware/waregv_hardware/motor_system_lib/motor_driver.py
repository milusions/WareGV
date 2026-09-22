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

    def __enter__(self):
        """Context manager setup: initializes the controller and sets servos to continuous mode."""
        self.controller.__enter__()
        
        for sid in self.servo_ids:
            servo = self.controller.wrap_servo(sid)
            self.servos[sid] = servo
            
            # Torque must be off before changing operating mode[cite: 1]
            self.original_modes[sid] = servo.eeprom.read_operating_mode()
            servo.sram.torque_disable()
            servo.eeprom.write_operating_mode(1) # Mode 1 = Constant Speed[cite: 1]
            servo.sram.torque_enable()
            servo.sram.write_running_speed(0)
            
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager teardown: stops servos and restores original operating modes."""
        for sid in self.servo_ids:
            self.servos[sid].sram.write_running_speed(0)
            time.sleep(0.1) # Brief delay to ensure stop command registers
            self.servos[sid].sram.torque_disable()
            self.servos[sid].eeprom.write_operating_mode(self.original_modes[sid])
            
        self.controller.__exit__(exc_type, exc_val, exc_tb)

    def set_rpm(self, servo_id, rpm):
        """Calculates step speed and updates the servo. Anti-clockwise is positive."""
        steps = -int((rpm * self.steps_per_rev) / 60)
        self.servos[servo_id].sram.write_running_speed(steps)

    def get_rpm(self, servo_id):
        """Reads current speed and converts to RPM. Anti-clockwise is positive."""
        speed_steps = self.servos[servo_id].sram.read_current_speed()
        if speed_steps is not None:
            return -(speed_steps * 60) / self.steps_per_rev
        return 0.0