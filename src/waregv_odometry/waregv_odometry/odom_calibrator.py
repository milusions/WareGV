#!/usr/bin/env python3
"""
Odometry calibration helper.

  python3 odom_calib.py auto straight --target 0.6  --current 0.036
  python3 odom_calib.py auto turn     --target 90   --current 0.192
  python3 odom_calib.py auto turn     --target 360  --current 0.192
  python3 odom_calib.py measure       (you drive with teleop; Enter = start / stop)

'auto' drives slowly on /cmd_vel until /odom reaches the target, stops, then asks
what you physically measured and prints the corrected parameter.

  straight: new wheel_radius     = current * measured / odom
  turn    : new wheel_separation = current * odom / measured
"""
import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Tracker(Node):
    def __init__(self, odom_topic, cmd_topic):
        super().__init__('odom_calib')
        self.x = self.y = self.yaw = 0.0   # yaw is UNWRAPPED
        self._last = None
        self.have = False
        self.create_subscription(Odometry, odom_topic, self.cb, 50)
        self.cmd = self.create_publisher(Twist, cmd_topic, 10)

    def cb(self, m):
        q = m.pose.pose.orientation
        y = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
        self.yaw = y if self._last is None else self.yaw + wrap(y - self._last)
        self._last = y
        self.x, self.y = m.pose.pose.position.x, m.pose.pose.position.y
        self.have = True

    def send(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd.publish(t)

    def stop(self):
        for _ in range(10):
            self.send(0, 0)
            time.sleep(0.05)


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['auto', 'measure'])
    ap.add_argument('kind', nargs='?', choices=['straight', 'turn'], default='straight')
    ap.add_argument('--target', type=float, default=0.6, help='m (straight) or deg (turn)')
    ap.add_argument('--current', type=float, default=None, help='current radius or separation')
    ap.add_argument('--vmax', type=float, default=0.10)
    ap.add_argument('--wmax', type=float, default=0.15)
    ap.add_argument('--gyro', action='store_true', help='turn test: tune gyro_scale instead of wheel_separation')
    ap.add_argument('--odom', default='/odom')
    ap.add_argument('--cmd', default='/cmd_vel')
    a = ap.parse_args()

    if not a.current:
        ap.error('--current is required (radius, separation, or gyro_scale)')
    rclpy.init()
    n = Tracker(a.odom, a.cmd)
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    while not n.have:
        time.sleep(0.05)

    x0, y0, yaw0 = n.x, n.y, n.yaw

    if a.mode == 'measure':
        input('Place robot at start mark, press Enter, then drive... ')
        x0, y0, yaw0 = n.x, n.y, n.yaw
        input('Press Enter when stopped at the end mark... ')
    else:
        input(f'Mark robot position/heading on the floor, press Enter to run {a.kind} {a.target}... ')
        x0, y0, yaw0 = n.x, n.y, n.yaw
        goal = a.target if a.kind == 'straight' else math.radians(a.target)
        sgn = 1.0 if goal >= 0 else -1.0
        goal = abs(goal)
        try:
            while True:
                done = (math.hypot(n.x - x0, n.y - y0) if a.kind == 'straight'
                        else abs(n.yaw - yaw0))
                rem = goal - done
                if rem <= (0.002 if a.kind == 'straight' else math.radians(0.3)):
                    break
                if a.kind == 'straight':
                    n.send(sgn * clip(1.5 * rem, 0.03, a.vmax), 0.0)
                else:
                    n.send(0.0, sgn * clip(1.5 * rem, 0.15, a.wmax))
                time.sleep(0.02)
        finally:
            n.stop()
        time.sleep(1.0)

    dist = math.hypot(n.x - x0, n.y - y0)
    dyaw = math.degrees(n.yaw - yaw0)
    print(f'\nODOM reports: distance = {dist * 100:.2f} cm, rotation = {dyaw:.2f} deg')

    if a.kind == 'straight':
        s = input('Tape-measured distance in cm (blank to skip): ').strip()
        if s and a.current:
            meas = float(s) / 100.0
            print(f'-> wheel_radius: {a.current} -> {a.current * meas / dist:.5f}')
    else:
        s = input('ACTUAL rotation in deg (360 test: 360 + overshoot, or 360 - undershoot; blank to skip): ').strip()
        if s:
            meas = abs(float(s))
            ratio = abs(dyaw) / meas
            print(f'odom/actual ratio = {ratio:.4f}')
            if not (0.7 < ratio < 2.5):
                print('Ratio implausible; check your measured angle. No suggestion made.')
            elif a.gyro:
                print(f'-> gyro_scale: {a.current} -> {a.current / ratio:.5f}')
            else:
                print(f'-> wheel_separation: {a.current} -> {a.current * ratio:.5f}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()