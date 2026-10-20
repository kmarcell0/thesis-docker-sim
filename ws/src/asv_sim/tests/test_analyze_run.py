"""Tests for the pure functions of tools/analyze_run.py (M1.5).
The gate and clearance cases are the ones of test/test_gate_monitor.cpp, so
the Python and C++ versions of section 5.6 are checked against the same numbers.
Run from ws/src/asv_sim (ROS sourced):   python3 -m unittest discover -s tests -v"""
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'tools'))
import analyze_run as ar  # noqa: E402

SURMARK = 0.2286   # m, models/rb26_surmark_*/model.sdf
A2 = 0.184         # m, models/rb26_a2_*/model.sdf
L, BW = 1.60, 0.44  # boat length, half width (asv.yaml)
TOL = 1e-9


def task1_course():
    return {
        'buoys': {'gate1_red': (5.0, 1.524, SURMARK), 'gate1_green': (5.0, -1.524, SURMARK),
                  'gate2_red': (13.0, 1.524, SURMARK), 'gate2_green': (13.0, -1.524, SURMARK)},
        'gates': [('gate1', 'gate1_red', 'gate1_green'), ('gate2', 'gate2_red', 'gate2_green')],
        'margin': 0.5, 'capture': 2.0, 'waypoints': [],
    }


def straight(x0, x1, y, dt=0.02):
    """Poses (t, x, y, psi) in steps of 0.05 m from (x0, y) to (x1, y)."""
    psi = 0.0 if x1 > x0 else math.pi
    n = int(round(abs(x1 - x0) / 0.05))
    return [(k * dt, x0 + (x1 - x0) * k / n, y, psi) for k in range(n + 1)]


class Gates(unittest.TestCase):
    def test_outbound_then_inbound_passes(self):
        c = task1_course()
        poses = straight(0.0, 17.0, 0.0) + straight(17.0, -1.0, 0.0)
        ev = ar.gate_events(poses, c, BW)
        self.assertEqual([(e['gate'], e['direction']) for e in ev], ar.EXPECTED_PASSES)
        for e in ev:
            self.assertAlmostEqual(e['offset'], 0.0, delta=TOL)
            self.assertEqual(e['result'], 'PASS')
        self.assertTrue(ar.passes_in_order(ev))

    def test_offset_interpolated(self):
        c = task1_course()
        ev = ar.gate_events([(0.0, 4.0, 0.0, 0.0), (1.0, 6.0, 0.4, 0.0)], c, BW)
        self.assertEqual(len(ev), 1)
        self.assertAlmostEqual(ev[0]['offset'], 0.2, delta=TOL)  # > 0: towards red
        self.assertAlmostEqual(ev[0]['t'], 0.5, delta=TOL)

    def test_band_edge(self):
        # Band 1.524 - 0.2286 - 0.44 - 0.5 = 0.3554 m (section 5.6).
        c = task1_course()
        for y, want in [(0.355, 'PASS'), (-0.355, 'PASS'), (0.356, 'FAIL_CLOSE'),
                        (-0.356, 'FAIL_CLOSE'), (1.5, 'FAIL_CLOSE'), (2.0, 'FAIL_OUTSIDE'),
                        (-3.5, 'FAIL_OUTSIDE')]:
            ev = ar.gate_events(straight(4.0, 6.0, y), c, BW)
            self.assertEqual(len(ev), 1, y)
            self.assertAlmostEqual(ev[0]['offset'], y, delta=TOL)
            self.assertEqual(ev[0]['result'], want, y)
        self.assertEqual(ar.gate_events(straight(4.0, 6.0, 4.0), c, BW), [])  # beyond capture

    def test_landing_on_the_line_counts_once(self):
        c = task1_course()
        poses = [(0.0, 4.9, 0.0, 0.0), (0.1, 5.0, 0.0, 0.0), (0.2, 5.1, 0.0, 0.0)]
        self.assertEqual(len(ar.gate_events(poses, c, BW)), 1)

    def test_mixed_radii(self):
        red, green = (5.0, 1.524, A2), (5.0, -1.524, SURMARK)
        for y, want in [(0.399, 'PASS'), (-0.355, 'PASS'),
                        (0.401, 'FAIL_CLOSE'), (-0.356, 'FAIL_CLOSE')]:
            hit = ar.gate_crossing(red, green, (4.0, y), (6.0, y), BW, 0.5, 2.0)
            self.assertEqual(hit[3], want, y)

    def test_order_check(self):
        ev = [{'gate': g, 'direction': d, 'result': 'PASS'}
              for g, d in [('gate2', 'OUTBOUND'), ('gate1', 'OUTBOUND'),
                           ('gate2', 'INBOUND'), ('gate1', 'INBOUND')]]
        self.assertFalse(ar.passes_in_order(ev))


class Clearance(unittest.TestCase):
    def test_side_bow_corner(self):
        red = (5.0, 1.524, SURMARK)
        self.assertAlmostEqual(ar.clearance(red, (5.0, 0.0), 0.0, L, BW), 0.8554, delta=TOL)
        self.assertAlmostEqual(ar.clearance(red, (5.0, 0.0), math.pi / 2, L, BW), 0.4954,
                               delta=TOL)
        corner = (6.8, 1.44, SURMARK)
        self.assertAlmostEqual(ar.clearance(corner, (5.0, 0.0), 0.0, L, BW),
                               math.hypot(1.0, 1.0) - SURMARK, delta=TOL)

    def test_inside_rectangle(self):
        self.assertAlmostEqual(ar.clearance((5.3, 0.1, SURMARK), (5.0, 0.0), 0.0, L, BW),
                               -SURMARK, delta=TOL)

    def test_min_over_time(self):
        c = task1_course()
        m = ar.min_clearances(straight(0.0, 17.0, 0.0), c['buoys'], L, BW)
        self.assertAlmostEqual(m['gate1_red'][0], 0.8554, delta=TOL)  # abeam at x = 5


class Stats(unittest.TestCase):
    def test_rms(self):
        self.assertAlmostEqual(ar.rms([3.0, -4.0]), math.sqrt(12.5), delta=TOL)
        self.assertTrue(math.isnan(ar.rms([])))

    def test_reverse_share(self):
        self.assertAlmostEqual(ar.reverse_share([1, -1, 2, 0], [1, 1, -2, 0]), 0.5, delta=TOL)
        self.assertTrue(math.isnan(ar.reverse_share([], [])))

    def test_yaw(self):
        h = 0.7
        self.assertAlmostEqual(ar.yaw_from_quaternion(math.cos(h / 2), 0, 0, math.sin(h / 2)),
                               h, delta=TOL)

    def test_parse_status(self):
        st = ar.parse_status([
            (1.0, 'WAITING'), (4.0, 'RUNNING 1/9'), (9.0, 'GATE gate1 OUTBOUND offset -0.002 m PASS'),
            (10.0, 'RUNNING 2/9'), (68.5, 'FINISHED')])
        self.assertEqual(st['start'], 4.0)
        self.assertEqual(st['finish'], 68.5)
        self.assertIsNone(st['stopped'])
        self.assertEqual(st['node_events'][0]['offset'], -0.002)
        self.assertEqual(st['node_events'][0]['direction'], 'OUTBOUND')


if __name__ == '__main__':
    unittest.main()
