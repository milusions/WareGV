#!/usr/bin/env python3
"""
Bulletproof Odometry & IMU Calibrator for ROS 2  (v2)
=====================================================
Strategy:
  * Joint states arrive at ~10 Hz, IMU at ~100 Hz.
  * For every consecutive pair of joint samples (t0, t1):
      - dv_avg = trapezoidal mean of wheel diff velocity over [t0, t1]
      - wz_avg = arithmetic mean of IMU yaw-rate over [t0, t1]
      - reject if |wz_avg| or |dv_avg| too small, or signs disagree,
        or implied separation is absurd (>|sanity| x current).
  * Robust least-squares through origin on all pairs:
        wheel_separation = sum(dv * wz) / sum(wz^2)
    with 3 passes of 3-sigma outlier rejection.
  * Live estimate printed ~1 Hz. Final printed on Ctrl+C.

Notes:
  * If joint & IMU header stamps are on different clocks, set
    use_arrival_time:=true and both streams will be timestamped by
    node arrival time. Small constant latency, but consistent.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from rclpy.qos import qos_profile_sensor_data


class OdomImuCalibrator(Node):

    def __init__(self):
        super().__init__('odom_imu_calibrator')

        # ---------- user parameters ----------
        self.declare_parameter('wheel_radius', 0.036)
        self.declare_parameter('current_wheel_separation', 0.192)
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)

        # ---------- tuning ----------
        self.declare_parameter('min_joint_dt_s', 0.02)
        self.declare_parameter('max_joint_dt_s', 0.50)
        self.declare_parameter('min_imu_in_window', 2)
        self.declare_parameter('min_abs_imu_wz', 0.10)     # rad/s
        self.declare_parameter('min_abs_diff_vel', 0.005)  # m/s
        self.declare_parameter('sep_sanity_factor', 4.0)
        self.declare_parameter('imu_buffer_len', 2000)
        self.declare_parameter('pair_buffer_len', 6000)
        self.declare_parameter('print_period_s', 1.0)
        self.declare_parameter('use_arrival_time', False)

        self.r   = float(self.get_parameter('wheel_radius').value)
        self.sep = float(self.get_parameter('current_wheel_separation').value)
        self.ls  = float(self.get_parameter('left_sign').value)
        self.rs  = float(self.get_parameter('right_sign').value)

        self.min_dt   = float(self.get_parameter('min_joint_dt_s').value)
        self.max_dt   = float(self.get_parameter('max_joint_dt_s').value)
        self.min_imu_n= int(self.get_parameter('min_imu_in_window').value)
        self.min_wz   = float(self.get_parameter('min_abs_imu_wz').value)
        self.min_dv   = float(self.get_parameter('min_abs_diff_vel').value)
        self.sanity_f = float(self.get_parameter('sep_sanity_factor').value)
        self.print_period = float(self.get_parameter('print_period_s').value)
        self.use_arrival  = bool(self.get_parameter('use_arrival_time').value)

        # ---------- buffers ----------
        self.imu_buf   = deque(maxlen=int(self.get_parameter('imu_buffer_len').value))
        self.pairs     = deque(maxlen=int(self.get_parameter('pair_buffer_len').value))
        self.last_joint = None   # (t_sec, diff_vel)

        # ---------- counters ----------
        self.n_joint_msgs = 0
        self.n_imu_msgs   = 0
        self.n_pairs      = 0
        self.n_rejected   = 0
        self.n_intervals  = 0
        self.pos_turns    = 0
        self.neg_turns    = 0
        self.last_print   = self.get_clock().now()

        # ---------- ROS IO ----------
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Imu, '/imu_chassis', self.imu_cb, qos_profile_sensor_data)

        self.get_logger().info(
            "\n==================================================\n"
            " BULLETPROOF IMU/ODOM CALIBRATOR v2 STARTED\n"
            " Rotate the robot BOTH directions (>= 2 full turns each).\n"
            " Live estimate prints ~1 Hz.\n"
            " Press Ctrl+C to print final result.\n"
            "=================================================="
        )

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _msg_time(self, msg) -> float:
        """Use header stamp unless use_arrival_time is set."""
        if (not self.use_arrival) and (msg.header.stamp.sec or msg.header.stamp.nanosec):
            return self._stamp_to_sec(msg.header.stamp)
        return self._now_sec()

    # ------------------------------------------------------------------
    # joint states
    # ------------------------------------------------------------------
    def joint_cb(self, msg: JointState):
        self.n_joint_msgs += 1

        if len(msg.velocity) < 4:
            return
        v = msg.velocity
        v0, v1, v2, v3 = (x if math.isfinite(x) else 0.0 for x in (v[0], v[1], v[2], v[3]))

        w_l = self.ls * (v0 + v2) * 0.5
        w_r = self.rs * (v1 + v3) * 0.5
        v_l = self.r * w_l
        v_r = self.r * w_r
        diff_vel = v_r - v_l  # m/s

        t = self._msg_time(msg)

        # Form an interval with the previous joint sample
        if self.last_joint is not None:
            t0, dv0 = self.last_joint
            dt = t - t0
            if self.min_dt <= dt <= self.max_dt:
                self.n_intervals += 1

                dv_avg = 0.5 * (dv0 + diff_vel)         # trapezoidal
                wz_avg, n_imu_used = self._mean_imu_wz(t0, t)

                if wz_avg is None or n_imu_used < self.min_imu_n:
                    pass  # insufficient IMU coverage, silently skip
                elif abs(wz_avg) < self.min_wz:
                    pass  # not rotating enough, skip (not a rejection)
                elif abs(dv_avg) < self.min_dv:
                    pass  # nearly straight motion, skip
                elif (wz_avg > 0) != (dv_avg > 0):
                    # direction mismatch -> likely slip or wrong sign
                    self.n_rejected += 1
                else:
                    implied_sep = dv_avg / wz_avg  # positive by sign check
                    lo = self.sep / self.sanity_f
                    hi = self.sep * self.sanity_f
                    if lo < implied_sep < hi:
                        self.pairs.append((0.5 * (t0 + t), dv_avg, wz_avg))
                        self.n_pairs += 1
                        if wz_avg > 0:
                            self.pos_turns += 1
                        else:
                            self.neg_turns += 1
                    else:
                        self.n_rejected += 1

        self.last_joint = (t, diff_vel)
        self._maybe_print_live()

    # ------------------------------------------------------------------
    # imu - just buffer
    # ------------------------------------------------------------------
    def imu_cb(self, msg: Imu):
        self.n_imu_msgs += 1
        wz = float(msg.angular_velocity.z)
        if not math.isfinite(wz):
            return
        t = self._msg_time(msg)
        self.imu_buf.append((t, wz))

    # ------------------------------------------------------------------
    # mean IMU yaw rate over [t0, t1]
    # ------------------------------------------------------------------
    def _mean_imu_wz(self, t0, t1):
        """
        Return (mean_wz, n_used) over the interval [t0, t1].
        Uses samples strictly inside the interval if there are enough;
        otherwise falls back to the nearest-sample estimate.
        """
        total = 0.0
        n = 0
        for t, wz in self.imu_buf:
            if t < t0:
                continue
            if t > t1:
                break
            total += wz
            n += 1

        if n >= self.min_imu_n:
            return total / n, n

        # Fallback: closest sample to interval midpoint
        if not self.imu_buf:
            return None, 0
        t_mid = 0.5 * (t0 + t1)
        best_d = float('inf')
        best_wz = None
        for t, wz in self.imu_buf:
            d = abs(t - t_mid)
            if d < best_d:
                best_d = d
                best_wz = wz
        if best_wz is None:
            return None, 0
        return best_wz, 1

    # ------------------------------------------------------------------
    # estimator: robust least squares through origin
    #   model: dv = sep * wz
    #   fit:   sep = sum(dv * wz) / sum(wz^2)
    # ------------------------------------------------------------------
    @staticmethod
    def _robust_origin_fit(samples, n_passes=3, sigma_k=3.0):
        data = list(samples)
        sep = None
        rms = float('nan')
        std = float('nan')

        for _ in range(n_passes):
            if len(data) < 3:
                break
            s_xy = 0.0
            s_xx = 0.0
            for dv, wz in data:
                s_xy += dv * wz
                s_xx += wz * wz
            if s_xx <= 0:
                break
            sep = s_xy / s_xx

            res = [dv - sep * wz for dv, wz in data]
            m = sum(res) / len(res)
            var = sum((r - m) ** 2 for r in res) / max(1, len(res) - 1)
            std = math.sqrt(max(var, 0.0))

            if std <= 1e-12:
                rms = 0.0
                break

            kept = [(dv, wz) for (dv, wz), r in zip(data, res)
                    if abs(r - m) <= sigma_k * std]
            if len(kept) == len(data):
                rms = math.sqrt(sum(r * r for r in res) / len(res))
                break
            data = kept

        if sep is None or not math.isfinite(sep) or sep <= 0:
            return None
        return sep, len(data), rms, std

    # ------------------------------------------------------------------
    # live estimate + print
    # ------------------------------------------------------------------
    def _live_estimate(self):
        if len(self.pairs) < 20:
            return None
        return self._robust_origin_fit([(dv, wz) for _, dv, wz in self.pairs])

    def _maybe_print_live(self):
        now = self.get_clock().now()
        if (now - self.last_print).nanoseconds * 1e-9 < self.print_period:
            return
        self.last_print = now

        est = self._live_estimate()
        if est is None:
            self.get_logger().info(
                f"[live] joint={self.n_joint_msgs} imu={self.n_imu_msgs} "
                f"intervals={self.n_intervals} pairs={self.n_pairs} "
                f"rej={self.n_rejected}  (need more paired samples)"
            )
            return

        sep, n_used, rms, _ = est
        delta_pct = 100.0 * (sep - self.sep) / self.sep if self.sep > 0 else 0.0

        rolling = None
        if len(self.pairs) >= 200:
            tail = [(dv, wz) for _, dv, wz in list(self.pairs)[-200:]]
            rolling = self._robust_origin_fit(tail)

        msg = (
            f"[live] pairs={self.n_pairs} used={n_used} rej={self.n_rejected} "
            f"CW/CCW={self.pos_turns}/{self.neg_turns}  "
            f"sep_est={sep:.4f} m ({delta_pct:+.2f}% vs current {self.sep:.4f})  "
            f"rms_resid={rms:.4f} m/s"
        )
        if rolling is not None:
            rsep, _, rrms, _ = rolling
            msg += f"  | rolling(last200) sep={rsep:.4f} m rms={rrms:.4f}"
        self.get_logger().info(msg)

    # ------------------------------------------------------------------
    # final report
    # ------------------------------------------------------------------
    def compute_results(self):
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("FINAL CALIBRATION")
        self.get_logger().info("=" * 60)

        if len(self.pairs) < 100:
            self.get_logger().warn(
                f"Only {len(self.pairs)} paired samples. Need >=100 for a reliable fit. "
                "Spin the robot more (both directions!) and rerun."
            )
            return

        if self.pos_turns == 0 or self.neg_turns == 0:
            self.get_logger().warn(
                "You only rotated in ONE direction. Gyro bias will not cancel. "
                "Redo with equal CW and CCW turns for best accuracy."
            )
        ratio = self.pos_turns / max(1, self.neg_turns)
        if ratio > 3.0 or ratio < 1.0 / 3.0:
            self.get_logger().warn(
                f"Rotation imbalance CW/CCW = {ratio:.2f}. Prefer closer to 1.0."
            )

        samples = [(dv, wz) for _, dv, wz in self.pairs]
        est = self._robust_origin_fit(samples)
        if est is None:
            self.get_logger().error("Fit failed. No valid estimate produced.")
            return
        sep_new, n_used, rms, std = est

        # cross-check with intercept
        n = len(samples)
        sx  = sum(w for _, w in samples)
        sy  = sum(d for d, _ in samples)
        sxx = sum(w * w for _, w in samples)
        sxy = sum(d * w for d, w in samples)
        denom = n * sxx - sx * sx
        if abs(denom) > 1e-12:
            slope = (n * sxy - sx * sy) / denom
            intercept = (sy - slope * sx) / n
        else:
            slope, intercept = float('nan'), float('nan')

        delta_pct = 100.0 * (sep_new - self.sep) / self.sep if self.sep > 0 else 0.0

        self.get_logger().info(
            f"Paired samples:        {self.n_pairs} (used after robust fit: {n_used})\n"
            f"Rejected (pre-filter): {self.n_rejected}\n"
            f"Total intervals seen:  {self.n_intervals}\n"
            f"CW / CCW samples:      {self.pos_turns} / {self.neg_turns}\n"
            f"Current separation:    {self.sep:.5f} m\n"
            f"★ New separation:      {sep_new:.5f} m   ({delta_pct:+.2f} %)\n"
            f"Residual RMS:          {rms:.5f} m/s  (lower is better)\n"
            f"Cross-check (w/ b):    slope={slope:.5f} m  intercept={intercept:+.5f} m/s\n"
            "------------------------------------------------------------\n"
            "Sanity: intercept should be near 0. Large |intercept| means\n"
            "wheel_radius is off, IMU has bias, or pairing timestamps drift.\n"
            "============================================================"
        )
        self.get_logger().info(
            f"→ Set wheel_separation={sep_new:.5f} in wheel_odometry_node.py"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomImuCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.compute_results()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()