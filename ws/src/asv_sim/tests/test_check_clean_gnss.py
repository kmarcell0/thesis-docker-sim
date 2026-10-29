"""Tests for tools/check_clean_gnss.py (M2.2): interpolation and lever-arm maths
on hand-made data (no bag, no ROS).
Run from ws/src/asv_sim:   python3 -m unittest discover -s tests -v"""
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'tools'))
import check_clean_gnss as cg  # noqa: E402
import geodesy as gd  # noqa: E402

REF = (27.367256, -82.451120, 0.0)  # roboboat_course.sdf
ARM = (-0.76, -0.33, 0.185)


def yaw_quat(psi):
    return (math.cos(psi / 2), 0.0, 0.0, math.sin(psi / 2))


class TestMaths(unittest.TestCase):
    def test_rotate_yaw_quarter_turn(self):
        """Heading north (psi = pi/2): body x points north, body y west."""
        v = cg.rotate(yaw_quat(math.pi / 2), (1.0, 2.0, 3.0))
        for a, b in zip(v, (-2.0, 1.0, 3.0)):
            self.assertAlmostEqual(a, b, places=12)

    def test_rotate_roll(self):
        """Roll pi about x turns body z down (the upside-down lidar case)."""
        q = (math.cos(math.pi / 2), math.sin(math.pi / 2), 0.0, 0.0)
        for a, b in zip(cg.rotate(q, (0.0, 1.0, 1.0)), (0.0, -1.0, -1.0)):
            self.assertAlmostEqual(a, b, places=12)

    def test_slerp_halfway_and_shortest_way(self):
        q = cg.slerp(yaw_quat(0.0), yaw_quat(1.0), 0.5)
        for a, b in zip(q, yaw_quat(0.5)):
            self.assertAlmostEqual(a, b, places=12)
        # yaw 3.0 -> -3.0 crosses +-pi: halfway is pi, not 0
        q = cg.slerp(yaw_quat(3.0), yaw_quat(-3.0), 0.5)
        self.assertAlmostEqual(abs(q[3]), 1.0, places=12)

    def test_pose_at_interpolates_and_refuses_outside(self):
        truth = [(0.0, (0.0, 0.0, 0.0), yaw_quat(0.0)), (0.02, (0.04, 0.0, 0.0), yaw_quat(0.2))]
        p, q = cg.pose_at(truth, 0.005)
        self.assertAlmostEqual(p[0], 0.01, places=12)
        for a, b in zip(q, yaw_quat(0.05)):
            self.assertAlmostEqual(a, b, places=12)
        self.assertEqual(cg.pose_at(truth, 0.02)[0], (0.04, 0.0, 0.0))   # exact sample
        self.assertIsNone(cg.pose_at(truth, -0.001))
        self.assertIsNone(cg.pose_at(truth, 0.021))


class TestAntennaErrors(unittest.TestCase):
    def setUp(self):
        self.ltp = gd.LocalTangentPlane(*REF)
        # boat moving east at 2 m/s while turning at 0.5 rad/s, 50 Hz truth
        self.truth = [(k * 0.02, (2.0 * k * 0.02, 1.0, 0.1), yaw_quat(0.5 * k * 0.02))
                      for k in range(101)]

    def fix_at(self, t, shift=(0.0, 0.0, 0.0)):
        """A perfect fix of the antenna at time t (truth evaluated exactly), plus shift."""
        psi = 0.5 * t
        p = (2.0 * t, 1.0, 0.1)
        a = [pi + ri + si for pi, ri, si in zip(p, cg.rotate(yaw_quat(psi), ARM), shift)]
        return (t,) + tuple(self.ltp.to_geo(*a))

    def test_perfect_fixes_give_zero_error(self):
        fixes = [self.fix_at(0.1 * k) for k in range(11)]
        r = cg.summary(cg.antenna_errors(self.truth, fixes, self.ltp, ARM))
        self.assertEqual(r['n'], 11)
        self.assertLess(max(r['h_max'], r['v_max']), 1e-6)

    def test_known_offset_is_measured(self):
        fixes = [self.fix_at(0.1 * k, (0.003, -0.004, 0.002)) for k in range(11)]
        r = cg.summary(cg.antenna_errors(self.truth, fixes, self.ltp, ARM))
        self.assertAlmostEqual(r['h_max'], 0.005, delta=1e-6)
        self.assertAlmostEqual(r['v_rms'], 0.002, delta=1e-6)

    def test_lever_arm_matters(self):
        """Leaving the lever arm out gives an error of about |l| (0.85 m)."""
        fixes = [self.fix_at(0.1 * k) for k in range(11)]
        r = cg.summary(cg.antenna_errors(self.truth, fixes, self.ltp, (0.0, 0.0, 0.0)))
        self.assertAlmostEqual(r['h_max'], math.hypot(ARM[0], ARM[1]), delta=1e-6)

    def test_fixes_outside_truth_are_skipped(self):
        fixes = [self.fix_at(0.5), (5.0, *REF)]
        self.assertEqual(len(cg.antenna_errors(self.truth, fixes, self.ltp, ARM)), 1)

    def test_lever_arms_from_frames_file(self):
        arms = cg.lever_arms(os.path.join(PKG, 'config', 'asv_frames.yaml'))
        self.assertEqual(arms['gnss_primary_link'], [-0.76, -0.33, 0.185])
        self.assertEqual(arms['gnss_secondary_link'], [0.76, 0.33, 0.185])


if __name__ == '__main__':
    unittest.main()
