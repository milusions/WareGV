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

_DEFAULT_BRAKE = {
    "enabled": True,
    "trigger_rate_threshold": 15.0,
    "min_velocity_to_trigger": 1.0,
    "gain": 0.3,
    "duration": 0.15,
}

_DEFAULT_BEZIER = {
    "duration": 0.25,
}


class _AxisState:
    """Per-motor smoothing state: current velocity/acceleration + filter buffer."""

    def __init__(self, window):
        self.buffer = deque(maxlen=window)
        self.current_velocity = 0.0
        self.current_acceleration = 0.0
        self.prev_target = 0.0
        self.brake_time_left = 0.0
        self.brake_magnitude = 0.0
        self.bezier_start_velocity = 0.0
        self.bezier_target = 0.0
        self.bezier_elapsed = 0.0


def _load_config(config_path):
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f) or {}

    limits = dict(_DEFAULT_LIMITS)
    limits.update(cfg.get("motor_limits", {}))

    filt = cfg.get("filter", {})
    window = int(filt.get("moving_average_window", _DEFAULT_FILTER_WINDOW))

    brake = dict(_DEFAULT_BRAKE)
    brake.update(cfg.get("brake_compensation", {}))

    bezier = dict(_DEFAULT_BEZIER)
    bezier.update(cfg.get("bezier_profile", {}))

    return {
        "motor_limits": limits,
        "filter": {"moving_average_window": window},
        "brake_compensation": brake,
        "bezier_profile": bezier,
    }


def _safe_configure(servo):
    """Trap startup configuration errors so the node survives."""
    try:
        if servo is not None:
            configure_servo(servo)
    except Exception:
        pass


def init(left_port, right_port, forward_motor, rear_motor, config_path):
    global FRONT_LEFT_MOTOR_HANDLE, FRONT_RIGHT_MOTOR_HANDLE
    global REAR_LEFT_MOTOR_HANDLE, REAR_RIGHT_MOTOR_HANDLE
    global LEFT_CONTROLLER, RIGHT_CONTROLLER
    global _CONFIG, _AXIS_STATES, _LOG_FILE

    _CONFIG = _load_config(config_path)
    window = _CONFIG["filter"]["moving_average_window"]
    _AXIS_STATES = [_AxisState(window) for _ in range(4)]

    os.makedirs(os.path.dirname(_LOG_PATH), exist_ok=True)
    _LOG_FILE = open(_LOG_PATH, "w")
    _LOG_FILE.write(
        "timestamp,"
        "fl_setpoint,fl_actual,fl_torque,"
        "fr_setpoint,fr_actual,fr_torque,"
        "rl_setpoint,rl_actual,rl_torque,"
        "rr_setpoint,rr_actual,rr_torque\n"
    )
    _LOG_FILE.flush()

    try:
        LEFT_CONTROLLER = ST3215("/dev/tty" + left_port)
    except Exception:
        LEFT_CONTROLLER = None
        
    try:
        RIGHT_CONTROLLER = ST3215("/dev/tty" + right_port)
    except Exception:
        RIGHT_CONTROLLER = None

    if LEFT_CONTROLLER:
        FRONT_LEFT_MOTOR_HANDLE = LEFT_CONTROLLER.wrap_servo(forward_motor)
        REAR_LEFT_MOTOR_HANDLE = LEFT_CONTROLLER.wrap_servo(rear_motor)

    if RIGHT_CONTROLLER:
        FRONT_RIGHT_MOTOR_HANDLE = RIGHT_CONTROLLER.wrap_servo(forward_motor)
        REAR_RIGHT_MOTOR_HANDLE = RIGHT_CONTROLLER.wrap_servo(rear_motor)

    _safe_configure(FRONT_LEFT_MOTOR_HANDLE)
    _safe_configure(FRONT_RIGHT_MOTOR_HANDLE)
    _safe_configure(REAR_LEFT_MOTOR_HANDLE)
    _safe_configure(REAR_RIGHT_MOTOR_HANDLE)


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _smoothstep(t):
    t = _clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _apply_bezier_profile(state, target_velocity, dt, limits, bezier_cfg):
    if dt <= 0.0:
        return state.current_velocity

    max_vel = limits["max_velocity"]
    min_vel = limits["min_velocity"]
    target_velocity = _clamp(target_velocity, min_vel, max_vel)
    duration = max(1e-6, bezier_cfg["duration"])

    if target_velocity != state.bezier_target:
        state.bezier_start_velocity = state.current_velocity
        state.bezier_target = target_velocity
        state.bezier_elapsed = 0.0

    state.bezier_elapsed += dt
    s = _smoothstep(state.bezier_elapsed / duration)

    velocity = state.bezier_start_velocity + (state.bezier_target - state.bezier_start_velocity) * s
    velocity = _clamp(velocity, min_vel, max_vel)
    state.current_velocity = velocity
    return velocity


def _update_brake_pulse(state, target, dt, brake_cfg):
    if dt <= 0.0:
        return 0.0

    target_rate = (target - state.prev_target) / dt
    changed_this_tick = target != state.prev_target
    state.prev_target = target

    if brake_cfg.get("enabled", True) and state.brake_time_left <= 0.0 and changed_this_tick:
        sudden_change = abs(target_rate) > brake_cfg["trigger_rate_threshold"]
        already_moving = abs(state.current_velocity) > brake_cfg["min_velocity_to_trigger"]
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


def _safe_spin(servo, rad_per_sec):
    """Bulletproof motor spin command. Ignores transient serial drops."""
    try:
        if servo is not None:
            spin_servo(servo, rad_per_sec)
    except Exception:
        pass


def control_motors(speed_list, dt):
    if len(speed_list) != 4:
        raise ValueError("Speed list must contain exactly 4 values.")
    if _CONFIG is None or _AXIS_STATES is None:
        raise RuntimeError("motor_system.init() must be called before control_motors().")

    limits = _CONFIG["motor_limits"]
    brake_cfg = _CONFIG["brake_compensation"]
    bezier_cfg = _CONFIG["bezier_profile"]
    adjusted_speeds = [speed * CORRECTION_MATRIX[i] for i, speed in enumerate(speed_list)]

    output_speeds = []
    for i, target in enumerate(adjusted_speeds):
        state = _AXIS_STATES[i]
        ramped = _apply_bezier_profile(state, target, dt, limits, bezier_cfg)
        brake_offset = _update_brake_pulse(state, target, dt, brake_cfg)
        combined = _clamp(ramped + brake_offset, limits["min_velocity"], limits["max_velocity"])
        filtered = _moving_average(state, combined)
        output_speeds.append(round(filtered, 2))

    _safe_spin(FRONT_LEFT_MOTOR_HANDLE, output_speeds[0])
    _safe_spin(FRONT_RIGHT_MOTOR_HANDLE, output_speeds[1])
    _safe_spin(REAR_LEFT_MOTOR_HANDLE, output_speeds[2])
    _safe_spin(REAR_RIGHT_MOTOR_HANDLE, output_speeds[3])

    if _LOG_FILE is not None:
        torques = get_torques()
        ts = f"{time.time():.3f}"
        row = [ts]
        for sp, act, torque in zip(adjusted_speeds, output_speeds, torques):
            row.append(f"{sp:.2f}")
            row.append(f"{act}")
            row.append("NA" if torque is None else f"{torque}")
        try:
            _LOG_FILE.write(",".join(row) + "\n")
            _LOG_FILE.flush()
        except Exception:
            pass

    return output_speeds


def _safe_read(servo, read_method_name, fallback=0):
    """Bulletproof hardware read. Returns fallback if hardware faults."""
    try:
        if servo is not None:
            method = getattr(servo.sram, read_method_name)
            return method()
    except Exception:
        pass
    return fallback


def get_positions():
    return [
        _safe_read(FRONT_LEFT_MOTOR_HANDLE, 'read_current_location', 0),
        _safe_read(FRONT_RIGHT_MOTOR_HANDLE, 'read_current_location', 0),
        _safe_read(REAR_LEFT_MOTOR_HANDLE, 'read_current_location', 0),
        _safe_read(REAR_RIGHT_MOTOR_HANDLE, 'read_current_location', 0),
    ]


def get_velocities():
    return [
        _safe_read(FRONT_LEFT_MOTOR_HANDLE, 'read_current_speed', 0),
        _safe_read(FRONT_RIGHT_MOTOR_HANDLE, 'read_current_speed', 0),
        _safe_read(REAR_LEFT_MOTOR_HANDLE, 'read_current_speed', 0),
        _safe_read(REAR_RIGHT_MOTOR_HANDLE, 'read_current_speed', 0),
    ]


def get_velocities_rad():
    steps_per_sec = get_velocities()
    return [-1 * s * (2 * math.pi) / 4096 for s in steps_per_sec]


def get_torques():
    return [
        _safe_read(FRONT_LEFT_MOTOR_HANDLE, 'read_current_load', None),
        _safe_read(FRONT_RIGHT_MOTOR_HANDLE, 'read_current_load', None),
        _safe_read(REAR_LEFT_MOTOR_HANDLE, 'read_current_load', None),
        _safe_read(REAR_RIGHT_MOTOR_HANDLE, 'read_current_load', None),
    ]


def shutdown():
    """Failsafe motor stop and cleanly close serial connections."""
    global _LOG_FILE
    
    # Active braking failsafe on teardown
    _safe_spin(FRONT_LEFT_MOTOR_HANDLE, 0.0)
    _safe_spin(FRONT_RIGHT_MOTOR_HANDLE, 0.0)
    _safe_spin(REAR_LEFT_MOTOR_HANDLE, 0.0)
    _safe_spin(REAR_RIGHT_MOTOR_HANDLE, 0.0)

    try:
        if LEFT_CONTROLLER is not None:
            LEFT_CONTROLLER.close()
        if RIGHT_CONTROLLER is not None:
            RIGHT_CONTROLLER.close()
        if _LOG_FILE is not None:
            _LOG_FILE.close()
            _LOG_FILE = None
    except Exception:
        pass