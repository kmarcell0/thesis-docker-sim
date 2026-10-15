#!/usr/bin/env python3
"""Simulation-time check (M0.2): do ROS-clock timers follow Gazebo's clock?

Runs two 20 Hz timers side by side in a node with use_sim_time = true:
  - a ROS-clock timer (node.create_timer): should fire 20 times per
    SIMULATED second, whatever the real-time factor;
  - a wall-clock timer (clock = SYSTEM_TIME): fires 20 times per REAL second,
    so 20 / RTF times per simulated second. It is only here for comparison.

It also measures the real-time factor:
  RTF = (sim time elapsed) / (wall time elapsed)

Usage (from the host, with the sim already running headless):
    docker compose exec -T sim bash -ic \
        "cd /ws/src/asv_sim && python3 tools/sim_time_check.py --duration 20"
"""
import argparse
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter

RATE = 20.0  # Hz, both timers


class SimTimeCheck(Node):
    """Count ticks of a ROS-clock timer and a wall timer over a sim-time window."""

    def __init__(self):
        super().__init__(
            'sim_time_check',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.ros_ticks = 0
        self.wall_ticks = 0
        self.create_timer(1.0 / RATE, self._on_ros_tick)
        self.create_timer(1.0 / RATE, self._on_wall_tick,
                          clock=Clock(clock_type=ClockType.SYSTEM_TIME))

    def _on_ros_tick(self):
        self.ros_ticks += 1

    def _on_wall_tick(self):
        self.wall_ticks += 1

    def sim_now(self):
        """Current simulation time in seconds (0 until the first /clock)."""
        return self.get_clock().now().nanoseconds * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--duration', type=float, default=20.0,
                        help='simulated seconds to measure (default 20)')
    args = parser.parse_args()

    rclpy.init()
    node = SimTimeCheck()
    try:
        # The ROS clock reads 0 until the first /clock message arrives.
        t_wait = time.monotonic()
        while node.sim_now() == 0.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - t_wait > 10.0:
                print('No /clock received in 10 s - is the sim running?')
                return 1

        # Start the window on a fresh count, so the wait above is not counted.
        node.ros_ticks = node.wall_ticks = 0
        sim0, wall0 = node.sim_now(), time.monotonic()
        while node.sim_now() - sim0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
        sim_dt = node.sim_now() - sim0
        wall_dt = time.monotonic() - wall0

        rtf = sim_dt / wall_dt
        ros_rate = node.ros_ticks / sim_dt
        wall_rate = node.wall_ticks / sim_dt
        print('=== Simulation-time check ===')
        print(f'  simulated time : {sim_dt:8.3f} s')
        print(f'  wall time      : {wall_dt:8.3f} s')
        print(f'  real-time factor (sim/wall): {rtf:.3f}')
        print(f'  ROS-clock timer : {node.ros_ticks:5d} ticks = {ros_rate:6.2f} per sim s '
              f'(expect {RATE:.0f} +- 1)')
        print(f'  wall timer      : {node.wall_ticks:5d} ticks = {wall_rate:6.2f} per sim s '
              f'(expect {RATE / rtf:.2f} = {RATE:.0f} / RTF)')
        ok = abs(ros_rate - RATE) <= 1.0
        print('  RESULT:', 'PASS' if ok else 'FAIL')
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
