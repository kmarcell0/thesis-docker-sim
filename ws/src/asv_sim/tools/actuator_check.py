#!/usr/bin/env python3
"""Actuator check: command fixed thruster values and measure the odometry response.

Runs three tests on the simulated boat:
  A. Forward thrust at 10, 20, 40 N — compare measured steady-state speed
     with the drag model prediction (xU·u + xUU·u² = T).
  B. Steering sign — both thrusters at 10 N, pods at −0.4 rad — expect
     positive yaw rate (boat turns LEFT / CCW in ENU).
  C. Differential thrust — left −10 N, right +10 N — expect positive yaw rate.

Usage (from the host, with the sim already running headless):
    docker compose exec -T sim bash -ic \
        "cd /ws/src/asv_sim && python3 tools/actuator_check.py"
"""
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

# ── load drag coefficients from asv.yaml ────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from asvgen import config  # noqa: E402

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))


def predicted_speed(thrust, xu, xuu):
    """Solve xuu·u² + xu·u − T = 0 for steady-state forward speed u > 0."""
    disc = xu ** 2 + 4 * xuu * thrust
    if disc < 0:
        return 0.0
    return (-xu + math.sqrt(disc)) / (2 * xuu)


def yaw_from_quat(q):
    """Extract yaw (rad) from a geometry_msgs Quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y ** 2 + q.z ** 2))


class ActuatorCheck(Node):
    """Publish fixed thruster commands, record odometry, print results."""

    RATE = 10  # Hz — command publish rate

    def __init__(self, xu, xuu):
        # Phases are timed on Gazebo's clock (docs/design.md §3), so they last
        # the same simulated time whatever the real-time factor is.
        super().__init__(
            'actuator_check',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.xu = xu
        self.xuu = xuu
        self.latest_odom = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # publishers
        self.pub_left_thrust = self.create_publisher(Float64, '/asv/thrusters/left/thrust', qos)
        self.pub_right_thrust = self.create_publisher(Float64, '/asv/thrusters/right/thrust', qos)
        self.pub_left_pos = self.create_publisher(Float64, '/asv/thrusters/left/pos', qos)
        self.pub_right_pos = self.create_publisher(Float64, '/asv/thrusters/right/pos', qos)

        # subscriber
        self.create_subscription(Odometry, '/asv/ground_truth/odom', self._odom_cb, qos)

    def _odom_cb(self, msg):
        self.latest_odom = msg

    # ── helpers ──────────────────────────────────────────────────────────

    def publish_commands(self, left_thrust, right_thrust, left_pos, right_pos):
        """Publish a single set of thruster commands."""
        msg = Float64()
        msg.data = left_thrust
        self.pub_left_thrust.publish(msg)
        msg.data = right_thrust
        self.pub_right_thrust.publish(msg)
        msg.data = left_pos
        self.pub_left_pos.publish(msg)
        msg.data = right_pos
        self.pub_right_pos.publish(msg)

    def zero_all(self):
        self.publish_commands(0.0, 0.0, 0.0, 0.0)

    def sim_now(self):
        """Current simulation time in seconds (0 until the first /clock)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_for_odom(self, timeout=10.0):
        """Block until /clock and at least one odom message have arrived.

        The timeout is wall time on purpose: if Gazebo is not running, sim
        time never advances and a sim-time timeout would wait forever.
        """
        t0 = time.monotonic()
        while self.latest_odom is None or self.sim_now() == 0.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > timeout:
                self.get_logger().error('No odometry received — is the sim running?')
                return False
        return True

    def run_phase(self, duration, left_thrust, right_thrust, left_pos, right_pos):
        """Publish commands for *duration* simulated seconds, return list of odom samples.

        Commands go out about every 1/RATE s (the spin_once timeout is wall
        time, so the publish rate is not exact; the phase length is).
        """
        dt = 1.0 / self.RATE
        samples = []
        t0 = self.sim_now()
        while self.sim_now() - t0 < duration:
            self.publish_commands(left_thrust, right_thrust, left_pos, right_pos)
            rclpy.spin_once(self, timeout_sec=dt)
            if self.latest_odom is not None:
                samples.append(self.latest_odom)
        return samples

    def cooldown(self, duration=5.0):
        """Zero thrusters and wait for the boat to slow down."""
        self.run_phase(duration, 0.0, 0.0, 0.0, 0.0)

    # ── test sequences ───────────────────────────────────────────────────

    def test_a_forward(self):
        """Test A: forward thrust at several levels, measure steady-state speed."""
        results = []
        for total_thrust in (10.0, 20.0, 40.0):
            each = total_thrust / 2.0
            # settle
            self.run_phase(8.0, each, each, 0.0, 0.0)
            # measure
            samples = self.run_phase(4.0, each, each, 0.0, 0.0)
            speeds = [s.twist.twist.linear.x for s in samples]
            avg_speed = sum(speeds) / len(speeds) if speeds else 0.0
            pred = predicted_speed(total_thrust, self.xu, self.xuu)
            err_pct = ((avg_speed - pred) / pred * 100) if pred > 0 else 0.0
            results.append((total_thrust, pred, avg_speed, err_pct))
            self.cooldown()
        return results

    def test_b_steering(self):
        """Test B: both thrusters at 10 N, pods at −0.4 rad → expect positive yaw rate."""
        pod_angle = -0.4
        each = 10.0
        self.run_phase(6.0, each, each, pod_angle, pod_angle)
        samples = self.run_phase(3.0, each, each, pod_angle, pod_angle)
        yaw_rates = [s.twist.twist.angular.z for s in samples]
        avg_yaw_rate = sum(yaw_rates) / len(yaw_rates) if yaw_rates else 0.0
        self.cooldown()
        return avg_yaw_rate

    def test_c_differential(self):
        """Test C: left −10 N, right +10 N, pods straight → expect positive yaw rate."""
        self.run_phase(6.0, -10.0, 10.0, 0.0, 0.0)
        samples = self.run_phase(3.0, -10.0, 10.0, 0.0, 0.0)
        yaw_rates = [s.twist.twist.angular.z for s in samples]
        avg_yaw_rate = sum(yaw_rates) / len(yaw_rates) if yaw_rates else 0.0
        self.cooldown()
        return avg_yaw_rate

    # ── main ─────────────────────────────────────────────────────────────

    def run_all(self):
        print('Waiting for odometry...')
        if not self.wait_for_odom():
            return

        print('Starting actuator check (≈80 s)...\n')

        # Test A
        print('  Test A: forward thrust...')
        fwd_results = self.test_a_forward()

        # Test B
        print('  Test B: steering sign...')
        steer_yaw_rate = self.test_b_steering()

        # Test C
        print('  Test C: differential thrust...')
        diff_yaw_rate = self.test_c_differential()

        # zero everything at the end
        self.zero_all()

        # ── print results ────────────────────────────────────────────────
        print('\n=== Actuator Check Results ===\n')

        print('Test A: Forward thrust (drag model)')
        print(f'  {"Thrust [N]":>12}  {"Predicted [m/s]":>15}  {"Measured [m/s]":>14}  {"Error [%]":>10}')
        for thrust, pred, meas, err in fwd_results:
            print(f'  {thrust:12.1f}  {pred:15.2f}  {meas:14.2f}  {err:9.1f}')
        print()

        # steering sign
        direction = 'LEFT' if steer_yaw_rate > 0 else 'RIGHT'
        sign_ok = steer_yaw_rate > 0  # positive = CCW = left in ENU
        mark = '\u2713' if sign_ok else '\u2717 UNEXPECTED'
        print(f'Test B: Steering sign (pods = -0.4 rad, thrust = 20 N)')
        print(f'  Yaw rate: {steer_yaw_rate:+.4f} rad/s \u2192 boat turns {direction} {mark}')
        if not sign_ok:
            print('  WARNING: steering sign convention in CLAUDE.md may need updating')
        print()

        # differential
        direction_d = 'LEFT' if diff_yaw_rate > 0 else 'RIGHT'
        sign_ok_d = diff_yaw_rate > 0
        mark_d = '\u2713' if sign_ok_d else '\u2717 UNEXPECTED'
        print(f'Test C: Differential thrust (left=-10, right=+10 N)')
        print(f'  Yaw rate: {diff_yaw_rate:+.4f} rad/s \u2192 boat turns {direction_d} {mark_d}')
        print()

        return {
            'forward': fwd_results,
            'steer_yaw_rate': steer_yaw_rate,
            'diff_yaw_rate': diff_yaw_rate,
        }


def main():
    cfg, _, _ = config.prepare(os.path.join(PKG, 'config', 'asv.yaml'))
    xu = cfg['hydrodynamics']['xU']
    xuu = cfg['hydrodynamics']['xUU']
    print(f'Drag coefficients from asv.yaml: xU = {xu}, xUU = {xuu}')

    rclpy.init()
    node = ActuatorCheck(xu, xuu)
    try:
        node.run_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
