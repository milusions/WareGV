#!/usr/bin/env python3
"""
BNO055 -> /imu_chassis : GYRO Z ONLY, built for reliability.

  * Chip runs in GYRONLY mode: no fusion, no accel/mag, no calibration dependency.
  * Each poll reads ONLY the 2 bytes of gyro Z (registers 0x18/0x19) in one transaction.
  * Majority vote: `votes` back-to-back reads per tick, median is used. A single corrupt
    read cannot get through; a tick with no agreement is dropped (not published).
  * Range check (|z| <= 8 rad/s).
  * Watchdog: mode register is checked every 0.5 s; two mismatches in a row (= chip reset
    or bus corruption) -> mode re-applied, nothing published for 1 s.
  * Timestamp = midpoint of the read.
  * Same unit constant as the Adafruit library (900 LSB per rad/s), so the gyro_scale you
    already calibrated stays valid.

Published: sensor_msgs/Imu on /imu_chassis. Only angular_velocity.z is meaningful;
orientation and linear_acceleration covariance[0] = -1 (ROS "not provided").

Parameters: rate_hz (50), votes (3), i2c_bus (1), address (0x28), frame_id, gyro_var
"""
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Imu
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055

GYR_Z_LSB = 0x18
LSB_PER_RAD_S = 900.0          # same constant the Adafruit library uses
MODE = adafruit_bno055.GYRONLY_MODE
MAX_RATE = 8.0                 # rad/s, physical sanity limit
VOTE_AGREE = 0.05              # rad/s, 2-sample agreement tolerance


class GyroZRelay(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('votes', 3)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('address', 0x28)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('gyro_var', 1e-4)
        g = lambda n: self.get_parameter(n).value

        self.votes = max(1, int(g('votes')))
        self.addr = int(g('address'))
        rate = float(g('rate_hz'))

        self.i2c = I2C(int(g('i2c_bus')))
        self._init_chip()

        self.msg = Imu()
        self.msg.header.frame_id = str(g('frame_id'))
        self.msg.orientation_covariance = [-1.0] + [0.0] * 8
        self.msg.linear_acceleration_covariance = [-1.0] + [0.0] * 8
        self.msg.angular_velocity_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                                                float(g('gyro_var'))]

        self.pub = self.create_publisher(Imu, '/imu_chassis', 10)
        self.n_ok = self.n_bad = self.n_reset = self.bad_mode = 0
        self.quiet_until = 0.0
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(0.5, self.watchdog)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'Gyro-Z relay: GYRONLY, {rate:.0f} Hz, votes={self.votes}')

    # ---------- chip ----------
    def _init_chip(self):
        for attempt in range(5):
            try:
                self.bno = adafruit_bno055.BNO055_I2C(self.i2c, address=self.addr)
                self.bno.mode = MODE
                time.sleep(0.1)
                return
            except Exception as e:
                self.get_logger().warn(f'BNO055 init attempt {attempt + 1} failed: {e}')
                time.sleep(0.5)
        raise RuntimeError('BNO055 not responding')

    def _read_z(self) -> float:
        buf = bytearray(2)
        with self.bno.i2c_device as dev:
            dev.write_then_readinto(bytes([GYR_Z_LSB]), buf)
        return struct.unpack('<h', bytes(buf))[0] / LSB_PER_RAD_S

    # ---------- main loop ----------
    def tick(self):
        if time.monotonic() < self.quiet_until:
            return
        t0 = self.get_clock().now()
        vals = []
        for _ in range(self.votes):
            try:
                vals.append(self._read_z())
            except Exception:
                pass
        t1 = self.get_clock().now()

        need = 2 if self.votes >= 2 else 1
        if len(vals) < need:
            self.n_bad += 1
            return
        vals.sort()
        z = vals[len(vals) // 2] if len(vals) % 2 else 0.5 * (vals[len(vals) // 2 - 1] + vals[len(vals) // 2])
        if len(vals) == 2 and abs(vals[1] - vals[0]) > VOTE_AGREE:
            self.n_bad += 1                    # two reads disagree, cannot tell which is right
            return
        if abs(z) > MAX_RATE:
            self.n_bad += 1
            return

        self.msg.header.stamp = (t0 + Duration(nanoseconds=(t1 - t0).nanoseconds // 2)).to_msg()
        self.msg.angular_velocity.z = z
        self.pub.publish(self.msg)
        self.n_ok += 1

    def watchdog(self):
        try:
            m = self.bno.mode
        except Exception:
            return
        if m == MODE:
            self.bad_mode = 0
            return
        self.bad_mode += 1
        if self.bad_mode < 2:
            return
        self.bad_mode = 0
        self.n_reset += 1
        self.get_logger().error(f'BNO055 mode={m:#04x}, expected {MODE:#04x}: chip reset or bus '
                                f'corruption (count={self.n_reset}). Re-applying mode.')
        try:
            self.bno.mode = MODE
        except Exception as e:
            self.get_logger().error(f'Re-apply failed ({e}); full re-init')
            try:
                self._init_chip()
            except Exception as e2:
                self.get_logger().error(f'Re-init failed: {e2}')
        self.quiet_until = time.monotonic() + 1.0

    def report(self):
        total = self.n_ok + self.n_bad
        self.get_logger().info(
            f'published={self.n_ok} dropped={self.n_bad} '
            f'({100.0 * self.n_bad / total if total else 0.0:.2f}%) resets={self.n_reset}')


def main(args=None):
    rclpy.init(args=args)
    node = GyroZRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()