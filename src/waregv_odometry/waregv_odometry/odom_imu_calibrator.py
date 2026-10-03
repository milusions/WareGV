#!/usr/bin/env python3
"""
Bulletproof Odometry & IMU Calibrator for ROS 2
================================================
Subscribes to /joint_states and /imu_chassis.
Pairs wheel-diff-velocity with IMU yaw-rate using ROS timestamps
(interpolated), then performs a robust least-squares fit through the
origin to recover wheel_separation.

wheel_separation = sum(diff_vel * imu_wz) / sum(imu_wz^2)

This is the correct estimator: it naturally weights high-|omega|
samples more (where SNR is best) and is unbiased under zero-mean noise.

Press Ctrl+C to print final result.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState, Imu
from rclpy.qos import qos_profile_sensor_data


class OdomImuCalibrator(Node):
    def __init__(self):
        super().__init__('odom_imu_calibrator')

        # --- Parameters ---
        self.declare_parameter('wheel_radius', 0.036)
        self.declare_parameter('current_wheel_separation', 0.2)
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)

        # Tuning
        self.declare_parameter('pair_window_s', 0.020)     # 20 ms max dt for pairing
        self.declare_parameter('min_abs_imu_wz', 0.10)     # rad/s: ignore slow/noisy samples
        self.declare_parameter('min_abs_diff_vel', 0.005)  # m/s: ignore nearly-straight motion
        self.declare_parameter('buffer_len', 6000)         # ~ a few minutes at 50 Hz
        self.declare_parameter('print_period_s', 1.0)
        self.declare_parameter('sep_sanity_factor', 4.0)   # reject samples >4x or <1/4x current sep

        self.r   = float(self.get_parameter('wheel_radius').value)
        self.sep = float(self.get_parameter('current_wheel_separation').value)
        self.ls  = float(self.get_parameter('left_sign').value)
        self.rs  = float(self.get_parameter('right_sign').value)

        self.pair_window = float(self.get_parameter('pair_window_s').value)
        self.min_wz      = float(self.get_parameter('min_abs_imu_wz').value)
        self.min_dv      = float(self.get_parameter('min_abs_diff_vel').value)
        buf_len          = int(self.get_parameter('buffer_len').value)
        self.print_period= float(self.get_parameter('print_period_s').value)
        self.sanity_f    = float(self.get_parameter('sep_sanity_factor').value)

        # --- Buffers ---
        # Wheel samples: (t_sec, diff_vel_mps)
        self.wheel_buf = deque(maxlen=buf_len)
        # Paired samples: (t_sec, diff_vel_mps, imu_wz_radps)
        self.pairs = deque(maxlen=buf_len)

        # Bookkeeping
        self.n_joint_msgs = 0
        self.n_imu_msgs   = 0
        self.n_pairs      = 0
        self.n_rejected   = 0
        self.pos_turns    = 0
        self.neg_turns    = 0
        self.last_print   = self.get_clock().now()

        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Imu, '/imu_chassis', self.imu_cb, qos_profile_sensor_data)

        self.get_logger().info(
            "\n==================================================\n"
            " BULLETPROOF IMU/ODOM CALIBRATOR STARTED\n"
            " Rotate the robot BOTH directions (>= 2 full turns each).\n"
            " Live estimate prints ~1 Hz.\n"
            " Press Ctrl+C to print final result.\n"
            "=================================================="
        )

    # ---------------- helpers ----------------
    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _now_sec(self) -> float:
        t = self.get_clock().now().nanoseconds
        return t * 1e-9

    # ---------------- callbacks ----------------
    def joint_cb(self, msg: JointState):
        self.n_joint_msgs += 1

        # Need at least 4 wheels; tolerate NaN
        if len(msg.velocity) < 4:
            return
        v = msg.velocity
        # Zero out non-finite values instead of dropping the whole message
        v0, v1, v2, v3 = (x if math.isfinite(x) else 0.0 for x in (v[0], v[1], v[2], v[3]))

        w_l = self.ls * (v0 + v2) * 0.5
        w_r = self.rs * (v1 + v3) * 0.5
        v_l = self.r * w_l
        v_r = self.r * w_r
        diff_vel = v_r - v_l  # m/s

        # Timestamp (prefer message stamp, fallback to node clock)
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            t = self._stamp_to_sec(msg.header.stamp)
        else:
            t = self._now_sec()

        self.wheel_buf.append((t, diff_vel))

    def imu_cb(self, msg: Imu):
        self.n_imu_msgs += 1
        imu_wz = float(msg.angular_velocity.z)
        if not math.isfinite(imu_wz):
            return

        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            t_imu = self._stamp_to_sec(msg.header.stamp)
        else:
            t_imu = self._now_sec()

        # Ignore slow motion (low SNR)
        if abs(imu_wz) < self.min_wz:
            self._maybe_print_live()
            return

        # Find nearest wheel sample(s) and interpolate diff_vel at t_imu
        diff_vel = self._interp_wheel_diff_vel(t_imu)
        if diff_vel is None:
            self._maybe_print_live()
            return

        if abs(diff_vel) < self.min_dv:
            self._maybe_print_live()
            return

        # Sign consistency check: both must turn same way.
        if (imu_wz > 0) != (diff_vel > 0):
            self.n_rejected += 1
            self._maybe_print_live()
            return

        # Loose sanity gate (reject catastrophic skids only)
        implied_sep = diff_vel / imu_wz  # positive by sign check
        if implied_sep <= 0:
            self.n_rejected += 1
            self._maybe_print_live()
            return
        lo = self.sep / self.sanity_f
        hi = self.sep * self.sanity_f
        if not (lo < implied_sep < hi):
            self.n_rejected += 1
            self._maybe_print_live()
            return

        self.pairs.append((t_imu, diff_vel, imu_wz))
        self.n_pairs += 1
        if imu_wz > 0:
            self.pos_turns += 1
        else:
            self.neg_turns += 1

        self._maybe_print_live()

    # ---------------- pairing ----------------
    def _interp_wheel_diff_vel(self, t_target):
        """
        Interpolate diff_vel at t_target using the wheel buffer.
        Returns None if no sample within pair_window.
        """
        if not self.wheel_buf:
            return None

        # Buffer is append-ordered. Walk from the end to find bracket.
        # We keep it small enough this linear scan is fine.
        prev = None
        for t, dv in reversed(self.wheel_buf):
            if t <= t_target:
                if prev is None:
                    # No future sample; fall back to nearest within window
                    if abs(t_target - t) <= self.pair_window:
                        return dv
                    return None
                t_a, dv_a = t, dv
                t_b, dv_b = prev
                dt = t_b - t_a
                if dt <= 1e-9:
                    return dv_a
                alpha = (t_target - t_a) / dt
                return dv_a + alpha * (dv_b - dv_a)
            prev = (t, dv)

        # t_target is older than everything buffered
        t_oldest, dv_oldest = self.wheel_buf[0]
        if abs(t_target - t_oldest) <= self.pair_window:
            return dv_oldest
        return None

    # ---------------- estimator ----------------
    @staticmethod
    def _robust_origin_fit(samples, n_passes=3, sigma_k=3.0):
        """
        samples: list of (dv, wz)
        Model: dv = sep * wz  (through origin)
        Fit:   sep = sum(dv*wz) / sum(wz^2)
        Then drop |residual| > sigma_k * std(residual) and refit.
        Returns (sep, n_used, rms_residual, std_residual)
        """
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

            # residuals
            res = [dv - sep * wz for dv, wz in data]
            m = sum(res) / len(res)
            var = sum((r - m) ** 2 for r in res) / max(1, len(res) - 1)
            std = math.sqrt(max(var, 0.0))

            if std <= 1e-12:
                rms = 0.0
                break

            kept = [(dv, wz) for (dv, wz), r in zip(data, res) if abs(r - m) <= sigma_k * std]
            if len(kept) == len(data):
                rms = math.sqrt(sum(r * r for r in res) / len(res))
                break
            data = kept

        if sep is None or not math.isfinite(sep) or sep <= 0:
            return None
        return sep, len(data), rms, std

    def _live_estimate(self):
        if len(self.pairs) < 20:
            return None
        samples = [(dv, wz) for _, dv, wz in self.pairs]
        return self._robust_origin_fit(samples)

    def _maybe_print_live(self):
        now = self.get_clock().now()
        if (now - self.last_print).nanoseconds * 1e-9 < self.print_period:
            return
        self.last_print = now

        est = self._live_estimate()
        if est is None:
            self.get_logger().info(
                f"[live] joint={self.n_joint_msgs} imu={self.n_imu_msgs} "
                f"pairs={self.n_pairs} rej={self.n_rejected}  (need more paired samples)"
            )
            return

        sep, n_used, rms, std = est
        delta_pct = 100.0 * (sep - self.sep) / self.sep if self.sep > 0 else 0.0
        # also compute a "rolling" estimate using last ~200 pairs for stability view
        rolling = None
        if len(self.pairs) >= 200:
            tail = [(dv, wz) for _, dv, wz in list(self.pairs)[-200:]]
            rolling = self._robust_origin_fit(tail)

        msg = (
            f"[live] pairs={self.n_pairs} used={n_used} rej={self.n_rejected} "
            f"L+/R-={self.pos_turns}/{self.neg_turns}  "
            f"sep_est={sep:.4f} m ({delta_pct:+.2f}% vs current {self.sep:.4f})  "
            f"rms_resid={rms:.4f} m/s"
        )
        if rolling is not None:
            rsep, rn, rrms, _ = rolling
            msg += f"  | rolling(last200) sep={rsep:.4f} m rms={rrms:.4f}"
        self.get_logger().info(msg)

    # ---------------- final ----------------
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

        # Balance warning
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

        # Also give a linear regression with intercept as a cross-check
        # (should have ~0 intercept; nonzero intercept hints at wheel-radius bias
        #  or IMU bias.)
        n = len(samples)
        sx = sum(w for _, w in samples)
        sy = sum(d for d, _ in samples)
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