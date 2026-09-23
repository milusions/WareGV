import math


def configure_servo(servo):
    """
    One-time setup for a servo. Puts it into continuous-speed (wheel) mode.

    IMPORTANT: call this exactly once per servo, at startup (see motor_system.init).
    Do NOT call this on every control cycle -- toggling torque off/on and
    rewriting the operating mode interrupts whatever motion is already in
    progress. That was the root cause of the jitter: even when the commanded
    speed was constant, every incoming topic message re-triggered this
    disable -> reconfigure -> enable sequence, which made the servo stop and
    restart on every publish instead of spinning continuously.
    """
    servo.sram.torque_disable()
    servo.eeprom.write_operating_mode(1)
    servo.sram.torque_enable()


def spin_servo(servo, rad_per_sec):
    """
    Per-cycle speed update. Only writes the running-speed register.

    This is safe to call at high frequency / with a constant value: it does
    not touch torque or operating mode, so it never interrupts the servo's
    current motion. It just tells it to keep spinning at (possibly updated)
    speed.
    """
    steps_per_second = -1 * int(rad_per_sec * 4096 / (2 * math.pi))
    servo.sram.write_running_speed(steps_per_second)