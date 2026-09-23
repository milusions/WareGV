import math
from waregv_hardware.motor_system_lib.motor_driver import spin_servo

from python_st3215 import ST3215

CORRECTION_MATRIX = [1, -1, 1, -1]

FRONT_LEFT_MOTOR_HANDLE = None
FRONT_RIGHT_MOTOR_HANDLE = None
REAR_LEFT_MOTOR_HANDLE = None
REAR_RIGHT_MOTOR_HANDLE = None

LEFT_CONTROLLER = None
RIGHT_CONTROLLER = None


def init(left_port, right_port, forward_motor, rear_motor):
    global FRONT_LEFT_MOTOR_HANDLE, FRONT_RIGHT_MOTOR_HANDLE
    global REAR_LEFT_MOTOR_HANDLE, REAR_RIGHT_MOTOR_HANDLE
    global LEFT_CONTROLLER, RIGHT_CONTROLLER

    LEFT_CONTROLLER = ST3215("/dev/tty" + left_port)
    RIGHT_CONTROLLER = ST3215("/dev/tty" + right_port)

    FRONT_LEFT_MOTOR_HANDLE = LEFT_CONTROLLER.wrap_servo(forward_motor)
    REAR_LEFT_MOTOR_HANDLE = LEFT_CONTROLLER.wrap_servo(rear_motor)

    FRONT_RIGHT_MOTOR_HANDLE = RIGHT_CONTROLLER.wrap_servo(forward_motor)
    REAR_RIGHT_MOTOR_HANDLE = RIGHT_CONTROLLER.wrap_servo(rear_motor)


def control_motors(speed_list):
    """
    Control the motors based on the provided speed list.

    :param speed_list: A list of speeds for each motor in the order [front_left, front_right, rear_left, rear_right].
    """
    if len(speed_list) != 4:
        raise ValueError("Speed list must contain exactly 4 values.")

    adjusted_speeds = [speed * CORRECTION_MATRIX[i] for i, speed in enumerate(speed_list)]

    spin_servo(FRONT_LEFT_MOTOR_HANDLE, adjusted_speeds[0])
    spin_servo(FRONT_RIGHT_MOTOR_HANDLE, adjusted_speeds[1])
    spin_servo(REAR_LEFT_MOTOR_HANDLE, adjusted_speeds[2])
    spin_servo(REAR_RIGHT_MOTOR_HANDLE, adjusted_speeds[3])


def get_positions():
    """
    Read current position of each motor.

    :return: [front_left, front_right, rear_left, rear_right]
    """
    return [
        FRONT_LEFT_MOTOR_HANDLE.sram.read_current_location(),
        FRONT_RIGHT_MOTOR_HANDLE.sram.read_current_location(),
        REAR_LEFT_MOTOR_HANDLE.sram.read_current_location(),
        REAR_RIGHT_MOTOR_HANDLE.sram.read_current_location(),
    ]


def get_velocities():
    """
    Read current speed of each motor.

    :return: [front_left, front_right, rear_left, rear_right]
    """
    return [
        FRONT_LEFT_MOTOR_HANDLE.sram.read_current_speed(),
        FRONT_RIGHT_MOTOR_HANDLE.sram.read_current_speed(),
        REAR_LEFT_MOTOR_HANDLE.sram.read_current_speed(),
        REAR_RIGHT_MOTOR_HANDLE.sram.read_current_speed(),
    ]


def get_velocities_rad():
    """
    Read current speed of each motor, converted to rad/s
    (matching the sign convention used by spin_servo).

    :return: [front_left, front_right, rear_left, rear_right]
    """
    steps_per_sec = get_velocities()
    return [-1 * s * (2 * math.pi) / 4096 for s in steps_per_sec]


def shutdown():
    """
    Close both serial connections. Call this when done.
    """
    if LEFT_CONTROLLER is not None:
        LEFT_CONTROLLER.close()
    if RIGHT_CONTROLLER is not None:
        RIGHT_CONTROLLER.close()
