import math
from python_st3215 import ST3215


def spin_servo(servo, rad_per_sec):
        steps_per_second = -1*int(rad_per_sec * 4096 / (2 * math.pi))
     
        servo.sram.torque_disable()
        servo.eeprom.write_operating_mode(1)
        servo.sram.torque_enable()
        servo.sram.write_running_speed(steps_per_second)