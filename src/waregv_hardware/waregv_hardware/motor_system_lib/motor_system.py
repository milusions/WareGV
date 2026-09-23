import math
import os
import time
from collections import deque

import yaml

from waregv_hardware.motor_system_lib.motor_driver import spin_servo, configure_servo

from python_st3215 import ST3215

CORRECTION_MATRIX = [1, -1, 1, -1]

FRONT_LEFT_MOTOR_HANDLE = None
FRONT_RIGHT_MOTOR_HANDLE = None
REAR_LEFT_MOTOR_HANDLE = None
REAR_RIGHT_MOTOR_HANDLE = None

LEFT_CONTROLLER = None
RIGHT_CONTROLLER = None

# Populated by init() from the YAML config file.
_CONFIG = None
_AXIS_STATES = None

# Velocity log: truncated (overwritten) once per bootup in init(), then
# appended to on every control_motors() call.
_LOG_PATH = os.path.expanduser("~/waregv/waregv_ws/logs/motor_speed.log")
_LOG_FILE = None

# Defaults used only if a key is missing from the config file.
_DEFAULT_LIMITS = {
    "max_velocity": 9.0,
    "min_velocity": -9.0,
    "max_angular_velocity": 9.0,
    "min_angular_velocity": -9.0,
    "max_acceleration": 5.0,
    "min_acceleration": -5.0,
    "max_jerk": 10.0,
    "min_jerk": -10.0,
}
_DEFAULT_FILTER_WINDOW = 5

# Defaults for the inertia/braking compensation pulse (see _update_brake_pulse).
_DEFAULT_BRAKE = {
    "enabled": True,
    "trigger_rate_threshold": 15.0,   # rad/s^2 -- how fast the *target* must be changing to count as "sudden"
    "min_velocity_to_trigger": 1.0,   # rad/s -- don't bother braking if we were barely moving anyway
    "gain": 0.3,                      # fraction of the pre-change velocity used as the counter-pulse magnitude
    "duration": 0.15,                 # seconds the pulse lasts, linearly decaying to zero
}


class _AxisState:
    """Per-motor smoothing state: current velocity/acceleration + filter buffer."""

    def __init__(self, window):
        self.buffer = deque(maxlen=window)
        self.current_velocity = 0.0
        self.current_acceleration = 0.0
        # Inertia-compensation ("brake pulse") state.
        self.prev_target = 0.0
        self.brake_time_left = 0.0
        self.brake_magnitude = 0.0


def _load_config(config_path):
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f) or {}

    limits = dict(_DEFAULT_LIMITS)
    limits.update(cfg.get("motor_limits", {}))

    filt = cfg.get("filter", {})
    window = int(filt.get("moving_average_window", _DEFAULT_FILTER_WINDOW))

    brake = dict(_DEFAULT_BRAKE)
    brake.update(cfg.get("brake_compensation", {}))

    return {
        "motor_limits": limits,
        "filter": {"moving_average_window": window},
        "brake_compensation": brake,
    }


def init(left_port, right_port, forward_motor, rear_motor, config_path):
    """
    :param config_path: full path to motor_system_config.yaml
                         (e.g. <package_share_dir>/config/motor_system_config.yaml)
    """
    global FRONT_LEFT_MOTOR_HANDLE, FRONT_RIGHT_MOTOR_HANDLE
    global REAR_LEFT_MOTOR_HANDLE, REAR_RIGHT_MOTOR_HANDLE
    global LEFT_CONTROLLER, RIGHT_CONTROLLER
    global _CONFIG, _AXIS_STATES, _LOG_FILE

    _CONFIG = _load_config(config_path)
    window = _CONFIG["filter"]["moving_average_window"]
    _AXIS_STATES = [_AxisState(window) for _ in range(4)]

    # "w" truncates the file, so each bootup starts a fresh log instead of
    # appending indefinitely across runs.
    os.makedirs(os.path.dirname(_LOG_PATH), exist_ok=True)
    _LOG_FILE = open(_LOG_PATH, "w")
    _LOG_FILE.write("timestamp,fl_setpoint,fl_actual,fr_setpoint,fr_actual,rl_setpoint,rl_actual,rr_setpoint,rr_actual\n")
    _LOG_FILE.flush()

    LEFT_CONTROLLER = ST3215("/dev/tty" + left_port)
    RIGHT_CONTROLLER = ST3215("/dev/tty" + right_port)

    FRONT_LEFT_MOTOR_HANDLE = LEFT_CONTROLLER.wrap_servo(forward_motor)
    REAR_LEFT_MOTOR_HANDLE = LEFT_CONTROLLER.wrap_servo(rear_motor)

    FRONT_RIGHT_MOTOR_HANDLE = RIGHT_CONTROLLER.wrap_servo(forward_motor)
    REAR_RIGHT_MOTOR_HANDLE = RIGHT_CONTROLLER.wrap_servo(rear_motor)

    # One-time mode/torque setup -- never repeated on every control cycle.
    configure_servo(FRONT_LEFT_MOTOR_HANDLE)
    configure_servo(FRONT_RIGHT_MOTOR_HANDLE)
    configure_servo(REAR_LEFT_MOTOR_HANDLE)
    configure_servo(REAR_RIGHT_MOTOR_HANDLE)


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _apply_motion_profile(state, target_velocity, dt, limits):
    """
    Jerk-limited (S-curve) ramp from state.current_velocity toward
    target_velocity. This is what actually removes the "stop / think /
    rotate" behaviour: instead of jumping straight to whatever the topic
    last published, the commanded speed is only allowed to change at a
    bounded acceleration, and that acceleration itself is only allowed to
    change at a bounded jerk.
    """
    if dt <= 0.0:
        return state.current_velocity

    max_vel = limits["max_velocity"]
    min_vel = limits["min_velocity"]
    max_acc = limits["max_acceleration"]
    min_acc = limits["min_acceleration"]
    max_jerk = limits["max_jerk"]
    min_jerk = limits["min_jerk"]

    target_velocity = _clamp(target_velocity, min_vel, max_vel)

    # Acceleration needed to reach the target in one step, clamped to limits.
    desired_acc = (target_velocity - state.current_velocity) / dt
    desired_acc = _clamp(desired_acc, min_acc, max_acc)

    # Jerk needed to reach that acceleration, clamped to limits.
    desired_jerk = (desired_acc - state.current_acceleration) / dt
    desired_jerk = _clamp(desired_jerk, min_jerk, max_jerk)

    state.current_acceleration += desired_jerk * dt
    state.current_acceleration = _clamp(state.current_acceleration, min_acc, max_acc)

    state.current_velocity += state.current_acceleration * dt
    state.current_velocity = _clamp(state.current_velocity, min_vel, max_vel)

    return state.current_velocity


def _update_brake_pulse(state, target, dt, brake_cfg):
    """
    PX4-style inertia compensation.

    When the *commanded target* changes abruptly -- e.g. full throttle to
    zero -- the wheel still has momentum and will coast forward for a bit
    even as spin_servo tells it to stop. A flight controller handles the
    equivalent situation (stick snapped back to center) by briefly
    commanding a touch of reverse thrust to cancel that momentum instead of
    just cutting power and waiting for drag to do it. This does the same
    thing in velocity space:

      1. Detect a sudden change in the *target* (its rate of change exceeds
         trigger_rate_threshold), while the wheel is still actually moving
         and the new target represents a real deceleration/reversal
         relative to current velocity.
      2. On that edge, arm a short counter-pulse: a velocity offset opposite
         in sign to the current velocity, sized as a fraction (gain) of how
         fast we were going.
      3. That offset is added on top of the normal jerk-limited profile
         output and linearly decays to zero over `duration` seconds, so it
         nudges the wheel to actively cancel momentum rather than coast,
         then gets out of the way and lets the ordinary ramp finish the job.

    This only ever *adds* a temporary correction on top of the existing
    motion profile -- it never replaces or bypasses the accel/jerk limits,
    so it can't reintroduce the original torque-toggling jitter.

    :return: a velocity offset (rad/s) to add to this tick's profile output.
    """
    if dt <= 0.0:
        return 0.0

    target_rate = (target - state.prev_target) / dt
    state.prev_target = target

    if brake_cfg.get("enabled", True) and state.brake_time_left <= 0.0:
        sudden_change = abs(target_rate) > brake_cfg["trigger_rate_threshold"]
        already_moving = abs(state.current_velocity) > brake_cfg["min_velocity_to_trigger"]
        # A real deceleration/reversal: the target now opposes current
        # velocity, or is at least much smaller in magnitude than it.
        decelerating = (target * state.current_velocity < 0.0) or (
            abs(target) < 0.5 * abs(state.current_velocity)
        )

        if sudden_change and already_moving and decelerating:
            state.brake_time_left = brake_cfg["duration"]
            state.brake_magnitude = -math.copysign(
                abs(state.current_velocity) * brake_cfg["gain"], state.current_velocity
            )

    if state.brake_time_left > 0.0:
        decay = state.brake_time_left / brake_cfg["duration"]
        offset = state.brake_magnitude * decay
        state.brake_time_left = max(0.0, state.brake_time_left - dt)
        return offset

    return 0.0


def _moving_average(state, value):
    state.buffer.append(value)
    return sum(state.buffer) / len(state.buffer)


def control_motors(speed_list, dt):
    """
    Smooth, filtered motor control.

    :param speed_list: target speeds [front_left, front_right, rear_left, rear_right]
                        in rad/s, BEFORE the left/right correction matrix.
    :param dt: time in seconds since this was last called. Must reflect the
               actual control-loop period (not the raw topic publish period),
               since it drives the acceleration/jerk integration.
    :return: the actual smoothed, filtered, rounded speeds that were sent to
             the servos, in the same [fl, fr, rl, rr] order.
    """
    if len(speed_list) != 4:
        raise ValueError("Speed list must contain exactly 4 values.")
    if _CONFIG is None or _AXIS_STATES is None:
        raise RuntimeError("motor_system.init() must be called before control_motors().")

    limits = _CONFIG["motor_limits"]
    brake_cfg = _CONFIG["brake_compensation"]
    adjusted_speeds = [speed * CORRECTION_MATRIX[i] for i, speed in enumerate(speed_list)]

    output_speeds = []
    for i, target in enumerate(adjusted_speeds):
        state = _AXIS_STATES[i]
        ramped = _apply_motion_profile(state, target, dt, limits)
        brake_offset = _update_brake_pulse(state, target, dt, brake_cfg)
        combined = _clamp(ramped + brake_offset, limits["min_velocity"], limits["max_velocity"])
        filtered = _moving_average(state, combined)
        output_speeds.append(round(filtered, 2))

    spin_servo(FRONT_LEFT_MOTOR_HANDLE, output_speeds[0])
    spin_servo(FRONT_RIGHT_MOTOR_HANDLE, output_speeds[1])
    spin_servo(REAR_LEFT_MOTOR_HANDLE, output_speeds[2])
    spin_servo(REAR_RIGHT_MOTOR_HANDLE, output_speeds[3])

    if _LOG_FILE is not None:
        ts = f"{time.time():.3f}"
        row = [ts]
        for sp, act in zip(adjusted_speeds, output_speeds):
            row.append(f"{sp:.2f}")
            row.append(f"{act}")
        _LOG_FILE.write(",".join(row) + "\n")
        _LOG_FILE.flush()

    return output_speeds


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
    Close both serial connections and the velocity log file. Call this when done.
    """
    global _LOG_FILE
    if LEFT_CONTROLLER is not None:
        LEFT_CONTROLLER.close()
    if RIGHT_CONTROLLER is not None:
        RIGHT_CONTROLLER.close()
    if _LOG_FILE is not None:
        _LOG_FILE.close()
        _LOG_FILE = None