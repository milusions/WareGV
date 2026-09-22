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
            print(f"[PortGroupDriver SETUP] Servo {sid} on {self.port}: Original Operating Mode = {self.original_modes[sid]}")
            
            servo.sram.torque_disable()
            print(f"[PortGroupDriver SETUP] Servo {sid} on {self.port}: Torque DISABLED for mode configuration.")
            
            servo.eeprom.write_operating_mode(1) # Mode 1 = Constant Speed
            print(f"[PortGroupDriver SETUP] Servo {sid} on {self.port}: Operating mode set to 1 (Constant Speed).")
            
            servo.sram.torque_enable()
            print(f"[PortGroupDriver SETUP] Servo {sid} on {self.port}: Torque ENABLED.")
            
            servo.sram.write_running_speed(0)
            print(f"[PortGroupDriver SETUP] Servo {sid} on {self.port}: Running speed initialized to 0 steps.")
            
        print(f"[PortGroupDriver ENTER] Port {self.port} successfully configured and ready.")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager teardown: stops servos and restores original operating modes."""
        print(f"[PortGroupDriver EXIT] Shutting down port {self.port}...")
        for sid in self.servo_ids:
            try:
                self.servos[sid].sram.write_running_speed(0)
                print(f"[PortGroupDriver TEARDOWN] Servo {sid} on {self.port}: Speed set to 0.")
                time.sleep(0.1) # Brief delay to ensure stop command registers
                
                self.servos[sid].sram.torque_disable()
                print(f"[PortGroupDriver TEARDOWN] Servo {sid} on {self.port}: Torque disabled.")
                
                self.servos[sid].eeprom.write_operating_mode(self.original_modes[sid])
                print(f"[PortGroupDriver TEARDOWN] Servo {sid} on {self.port}: Restored original mode {self.original_modes[sid]}.")
            except Exception as e:
                print(f"[PortGroupDriver ERROR] Error cleaning up Servo {sid} on {self.port}: {e}")
            
        self.controller.__exit__(exc_type, exc_val, exc_tb)
        print(f"[PortGroupDriver EXIT] Port {self.port} controller closed cleanly.")

    def set_rpm(self, servo_id, rpm):
        """Calculates step speed and updates the servo. Anti-clockwise is positive."""
        steps = -int((rpm * self.steps_per_rev) / 60)
        print(f"[HW WRITE] Port: {self.port} | Servo {servo_id} -> Target RPM: {rpm:.2f} | Calculated Steps: {steps}")
        try:
            self.servos[servo_id].sram.write_running_speed(steps)
        except Exception as e:
            print(f"[HW WRITE ERROR] Failed to write speed to Servo {servo_id} on {self.port}: {e}")

    def get_rpm(self, servo_id):
        """Reads current speed and converts to RPM. Anti-clockwise is positive."""
        try:
            speed_steps = self.servos[servo_id].sram.read_current_speed()
            if speed_steps is not None:
                rpm = -(speed_steps * 60) / self.steps_per_rev
                # print(f"[HW READ] Port: {self.port} | Servo {servo_id} -> Raw Steps: {speed_steps} | Measured RPM: {rpm:.2f}")
                return rpm
        except Exception as e:
            print(f"[HW READ ERROR] Failed to read speed from Servo {servo_id} on {self.port}: {e}")
        return 0.0