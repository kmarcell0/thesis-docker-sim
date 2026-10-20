"""Tests for the pure functions of tools/compare_runs.py (M1.7b).
Run from ws/src/asv_sim (ROS sourced):   python3 -m unittest discover -s tests -v"""
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'tools'))
import compare_runs as cr  # noqa: E402

TOL = 1e-9


class Msg:
    """Stand-in for std_msgs/Float64."""

    def __init__(self, data):
        self.data = data


def circle_poses(t0, t1, dt=0.02, r=5.0, omega=0.2, dx=0.0):
    """Poses (t, x, y, psi) on a left circle; psi wrapped like the odometry."""
    out = []
    n = int(round((t1 - t0) / dt))
    for k in range(n + 1):
        t = t0 + k * dt
        a = omega * t
        out.append((t, r * math.sin(a) + dx, r * (1 - math.cos(a)), cr.wrap(a)))
    return out


class TestAngles(unittest.TestCase):

    def test_wrap(self):
        self.assertAlmostEqual(cr.wrap(3 * math.pi / 2), -math.pi / 2)
        self.assertAlmostEqual(cr.wrap(-math.pi), math.pi)

    def test_unwrap_across_pi(self):
        u = cr.unwrap([3.0, -3.0, -2.9])
        self.assertAlmostEqual(u[1], 3.0 + (2 * math.pi - 6.0))
        self.assertTrue(u[0] < u[1] < u[2])


class TestStartAndTrack(unittest.TestCase):

    def test_start_is_first_running(self):
        status = [(1.0, 'WAITING'), (4.05, 'RUNNING 1/3'), (9.0, 'RUNNING 2/3')]
        self.assertEqual(cr.start_time(status), 4.05)
        self.assertIsNone(cr.start_time([(1.0, 'WAITING')]))

    def test_track_shifts_time(self):
        trk = cr.track([(5.0, 1.0, 2.0, 0.5), (4.0, 0.0, 0.0, 0.0)], 4.0)
        self.assertEqual([s[0] for s in trk], [0.0, 1.0])

    def test_interpolate_midpoint_and_outside(self):
        trk = [(0.0, 0.0, 0.0, 0.0), (1.0, 2.0, 4.0, 1.0)]
        x, y, psi = cr.interpolate(trk, 0.25)
        self.assertAlmostEqual(x, 0.5)
        self.assertAlmostEqual(y, 1.0)
        self.assertAlmostEqual(psi, 0.25)
        self.assertIsNone(cr.interpolate(trk, 1.5))
        self.assertIsNone(cr.interpolate(trk, -0.1))


class TestCompare(unittest.TestCase):

    def test_identical_runs_give_zero(self):
        a = cr.track(circle_poses(3.0, 40.0), 3.0)
        c = cr.compare(a, a, 30.0, 35.0)
        self.assertLess(c['max_e'], TOL)
        self.assertLess(c['max_abs_e_psi'], TOL)
        self.assertAlmostEqual(c['tau_range'][1], 35.0)

    def test_shift_gives_shift(self):
        a = cr.track(circle_poses(3.0, 40.0), 3.0)
        b = cr.track(circle_poses(3.0, 40.0, dx=0.1), 3.0)
        c = cr.compare(a, b, 30.0, 35.0)
        self.assertAlmostEqual(c['max_e'], 0.1)
        self.assertAlmostEqual(c['rms_e'], 0.1)
        self.assertAlmostEqual(c['e_at_end'], 0.1)

    def test_offset_samples_are_interpolated(self):
        # Same path, B sampled 10 ms later and with its own start time: after
        # alignment on tau the difference is only the linear interpolation
        # error of a 5 m circle at 0.02 s steps (< 1e-4 m).
        a = cr.track(circle_poses(3.0, 40.0), 3.0)
        b = cr.track([(t + 1.01, x, y, p) for t, x, y, p in circle_poses(3.0, 40.0)], 4.01)
        c = cr.compare(a, b, 30.0, 35.0)
        self.assertLess(c['max_e'], 1e-4)

    def test_heading_difference_across_pi(self):
        # A and B differ by 0.1 rad of heading while the circle passes +-pi.
        a = cr.track([(t, 0.0, 0.0, cr.wrap(3.1 + 0.1 * t)) for t in range(0, 6)], 0.0)
        b = cr.track([(t, 0.0, 0.0, cr.wrap(3.2 + 0.1 * t)) for t in range(0, 6)], 0.0)
        c = cr.compare(a, b, 4.0, 5.0)
        self.assertAlmostEqual(c['max_abs_e_psi'], 0.1)

    def test_no_overlap_gives_none(self):
        a = cr.track([(0.0, 0, 0, 0), (1.0, 1, 0, 0)], 0.0)
        b = cr.track([(5.0, 0, 0, 0), (6.0, 1, 0, 0)], 0.0)
        self.assertIsNone(cr.compare(a, b, 1.0, 1.0))


class TestCommands(unittest.TestCase):

    def test_changes_and_offset(self):
        a = cr.command_changes([(t, Msg(v)) for t, v in
                                [(3.0, 0.0), (3.05, 0.0), (4.0, 5.0), (4.05, 5.0), (9.0, 0.0)]], 4.0)
        self.assertEqual(a, [(-1.0, 0.0), (0.0, 5.0), (5.0, 0.0)])
        b = cr.command_changes([(t, Msg(v)) for t, v in
                                [(3.1, 0.0), (4.1, 5.0), (9.15, 0.0)]], 4.1)
        self.assertAlmostEqual(cr.command_offset(a, b), 0.05)

    def test_different_commands(self):
        a = [(0.0, 0.0), (1.0, 5.0)]
        b = [(0.0, 0.0), (1.0, 4.0)]
        self.assertIsNone(cr.command_offset(a, b))


if __name__ == '__main__':
    unittest.main()
