"""Tests for tools/check_corruptor.py (M2.6d): expected sigmas, pass rule,
residuals, counts and schedule matching on hand-made data (no bag, no ROS).
Run from ws/src/asv_sim:   python3 -m unittest discover -s tests -v"""
import copy
import math
import os
import random
import sys
import unittest
from types import SimpleNamespace as NS

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'tools'))
import check_corruptor as cc  # noqa: E402
import geodesy as gd  # noqa: E402

REF = (27.367256, -82.451120, 0.0)  # roboboat_course.sdf


def load(*parts):
    with open(os.path.join(PKG, 'config', *parts)) as f:
        return yaml.safe_load(f)


def header(t_ns):
    return NS(stamp=NS(sec=t_ns // 1_000_000_000, nanosec=t_ns % 1_000_000_000))


def vec(x, y, z):
    return NS(x=x, y=y, z=z)


def imu(t_ns, g, a):
    return NS(header=header(t_ns), angular_velocity=vec(*g), linear_acceleration=vec(*a))


def yaw_quat(psi):
    return (math.cos(psi / 2), 0.0, 0.0, math.sin(psi / 2))


class ExpectedSigmas(unittest.TestCase):
    def setUp(self):
        self.asv = load('asv.yaml')
        self.nominal = load('noise_profiles', 'nominal.yaml')

    def test_nominal_values(self):
        # Typed from asv.yaml and the M2.6a-c reports (2026-11-02), so a change
        # there shows up here and gets looked at.
        e = cc.expected_sigmas(self.asv, self.nominal)
        self.assertAlmostEqual(e['gyro_density'], 6.632e-5, delta=1e-8)
        self.assertAlmostEqual(e['accel_density'], 1.079e-3, delta=1e-7)
        self.assertEqual(e['modes'], {'rtk_fixed': (0.005, 0.0094),
                                      'rtk_float': (0.05, 0.10),
                                      'standalone': (0.3, 0.5)})
        self.assertAlmostEqual(e['velocity'], 0.03, places=12)
        self.assertAlmostEqual(e['heading'], 1.0532e-3, delta=1e-7)
        self.assertEqual(e['schedule'], [(0.0, 'rtk_fixed')])
        # per 4 ms sample, as in the M2.6a report
        self.assertAlmostEqual(e['gyro_density'] / math.sqrt(0.004), 1.049e-3, delta=1e-6)
        self.assertAlmostEqual(e['accel_density'] / math.sqrt(0.004), 1.706e-2, delta=1e-5)

    def test_multipliers(self):
        p = copy.deepcopy(self.nominal)
        p['imu']['gyro']['white'] = 2.0
        p['gnss']['white'] = 3.0
        p['gnss']['velocity'] = 0.0
        p['gnss']['heading'] = 0.5
        e0, e = cc.expected_sigmas(self.asv, self.nominal), cc.expected_sigmas(self.asv, p)
        self.assertAlmostEqual(e['gyro_density'], 2.0 * e0['gyro_density'], places=15)
        self.assertEqual(e['accel_density'], e0['accel_density'])
        self.assertAlmostEqual(e['modes']['rtk_float'][1], 0.30, places=12)
        self.assertEqual(e['velocity'], 0.0)
        self.assertAlmostEqual(e['heading'], 0.5 * e0['heading'], places=15)

    def test_mode_check_profile(self):
        """mode_check.yaml: nominal sizes, every mode once, outage before the end."""
        p = load('noise_profiles', 'mode_check.yaml')
        e0, e = cc.expected_sigmas(self.asv, self.nominal), cc.expected_sigmas(self.asv, p)
        for k in ('gyro_density', 'accel_density', 'modes', 'velocity', 'heading'):
            self.assertEqual(e[k], e0[k], k)
        self.assertEqual(e['schedule'], [(0.0, 'rtk_fixed'), (20.0, 'rtk_float'),
                                         (40.0, 'standalone'), (60.0, 'outage'),
                                         (70.0, 'rtk_fixed')])

    def test_single_antenna_has_no_heading(self):
        asv = copy.deepcopy(self.asv)
        for s in asv['sensors']:
            if s['type'] == 'gnss_dual':
                s['type'] = 'gnss'
        self.assertIsNone(cc.expected_sigmas(asv, self.nominal)['heading'])


class Stats(unittest.TestCase):
    def draws(self, n, mean, sigma, seed=7):
        rng = random.Random(seed)
        return [rng.gauss(mean, sigma) for _ in range(n)]

    def test_matching_noise_passes(self):
        s = cc.residual_stats(self.draws(2000, 0.0, 0.5), 0.5)
        self.assertTrue(s['mean_ok'] and s['std_ok'], s)
        self.assertEqual(s['n'], 2000)

    def test_too_large_std_fails(self):
        """10 % too large: 3 standard errors of the std at n = 2000 are 4.7 %."""
        s = cc.residual_stats(self.draws(2000, 0.0, 0.55), 0.5)
        self.assertTrue(s['mean_ok'])
        self.assertFalse(s['std_ok'])

    def test_offset_mean_fails(self):
        """Offset 0.2 sigma: 3 standard errors of the mean at n = 2000 are 0.067 sigma."""
        s = cc.residual_stats(self.draws(2000, 0.1, 0.5), 0.5)
        self.assertFalse(s['mean_ok'])
        self.assertTrue(s['std_ok'])

    def test_too_few_samples_fail(self):
        s = cc.residual_stats([0.1], 0.5)
        self.assertFalse(s['mean_ok'] or s['std_ok'])

    def test_wrap(self):
        self.assertAlmostEqual(cc.wrap(2 * math.pi - 0.1), -0.1, places=12)
        self.assertAlmostEqual(cc.wrap(-2 * math.pi + 0.1), 0.1, places=12)


class Residuals(unittest.TestCase):
    def test_imu_only_4ms_steps(self):
        """Residual = corrupted - clean - bias, exactly; the 8 ms step is left out."""
        stamps = [0, 4_000_000, 8_000_000, 16_000_000]   # steps 4, 4, 8 ms
        clean = {t: imu(t, (0.1, 0.2, 0.3), (1.0, 2.0, 9.8)) for t in stamps}
        bias = {t: imu(t, (0.01, 0.02, 0.03), (0.1, 0.2, 0.3)) for t in stamps}
        corrupted = {t: imu(t, (0.111, 0.222, 0.333), (1.15, 2.25, 10.35)) for t in stamps}
        r = cc.imu_residuals(clean, corrupted, bias)
        for j, want in enumerate((0.001, 0.002, 0.003)):
            self.assertEqual(len(r['gyro'][j]), 2)
            for v in r['gyro'][j]:
                self.assertAlmostEqual(v, want, places=12)
        for j, want in enumerate((0.05, 0.05, 0.25)):
            for v in r['accel'][j]:
                self.assertAlmostEqual(v, want, places=12)

    def test_gnss_residual_is_offset_minus_slow_error(self):
        clean_ltp = gd.LocalTangentPlane(*REF)
        lat, lon, alt = clean_ltp.to_geo(100.0, -50.0, 0.2)     # a clean fix
        here = gd.LocalTangentPlane(lat, lon, alt)
        flat, flon, falt = here.to_geo(1.0, 2.0, 3.0)           # + error (1, 2, 3)
        t = 5_000_000_000
        clean = {t: NS(latitude=lat, longitude=lon, altitude=alt)}
        fixes = {t: NS(latitude=flat, longitude=flon, altitude=falt)}
        errors = {t: NS(vector=vec(0.5, 0.5, 0.5))}
        r = cc.gnss_residuals(clean, fixes, errors, {t: 'rtk_float'})
        self.assertEqual(set(r), {'rtk_float'})
        for j, want in enumerate((0.5, 1.5, 2.5)):
            self.assertAlmostEqual(r['rtk_float'][j][0], want, delta=1e-6)

    def truth(self, pose_of_t, t_end=2.0, dt=0.02):
        out = []
        for k in range(int(round(t_end / dt)) + 1):
            t = k * dt
            p, psi = pose_of_t(t)
            out.append((t, p, yaw_quat(psi)))
        return out

    def test_velocity_straight_line(self):
        """Boat going east at 1 m/s, no turn: antenna velocity (1, 0, 0)."""
        truth = self.truth(lambda t: ((t, 0.0, 0.0), 0.0))
        v = {1_000_000_000: NS(twist=NS(twist=NS(linear=vec(1.01, -0.02, 0.0))))}
        r = cc.velocity_residuals(truth, v, (-0.76, -0.33, 0.185))
        for j, want in enumerate((0.01, -0.02, 0.0)):
            self.assertAlmostEqual(r[j][0], want, places=9)

    def test_velocity_includes_rotation_of_the_lever_arm(self):
        """Boat turning on the spot at w = 0.3 rad/s, yaw 0 at t = 1 s: the
        antenna moves with w x l = (-w l_y, w l_x, 0). The central difference
        over +-20 ms is short by a factor sin(w h)/(w h): 6e-6 relative."""
        w, arm = 0.3, (-0.76, -0.33, 0.185)
        truth = self.truth(lambda t: ((0.0, 0.0, 0.0), w * (t - 1.0)))
        v = {1_000_000_000: NS(twist=NS(twist=NS(linear=vec(-w * arm[1], w * arm[0], 0.0))))}
        r = cc.velocity_residuals(truth, v, arm)
        for j in range(3):
            self.assertAlmostEqual(r[j][0], 0.0, delta=1e-5)

    def test_heading_residual_across_the_wrap(self):
        """yaw just past pi/2 means an azimuth just below 2 pi: residual 0, not 2 pi."""
        for psi, err in ((0.3, 0.002), (math.pi / 2 + 0.001, -0.001), (-2.0, 0.0)):
            truth = self.truth(lambda t, psi=psi: ((0.0, 0.0, 0.0), psi))
            alpha = (math.pi / 2 - psi + err) % (2 * math.pi)
            r = cc.heading_residuals(truth, {1_000_000_000: NS(azimuth=alpha)})
            self.assertAlmostEqual(r[0], err, places=9)


class CountsAndSchedule(unittest.TestCase):
    def test_missing_and_extra_inside_the_window(self):
        missing, extra = cc.missing_and_extra({1, 2, 3, 4, 9}, {2, 3, 5, 9}, 2, 5)
        self.assertEqual((missing, extra), ([4], [5]))

    def modes(self, plan, t_start=3.0, t_end=95.0):
        """{stamp_ns: mode} at 10 Hz from plan = [(from_s, mode)], first fix at t_start."""
        out = {}
        for k in range(int(round((t_end - t_start) * 10))):
            t = t_start + k / 10
            mode = [m for f, m in plan if f <= t - t_start + 1e-9][-1]
            out[int(round(t * 1e9))] = mode
        return out

    SCHEDULE = [(0.0, 'rtk_fixed'), (20.0, 'rtk_float'), (40.0, 'standalone'),
                (60.0, 'outage'), (70.0, 'rtk_fixed')]

    def test_switches(self):
        s = cc.mode_switches(self.modes(self.SCHEDULE))
        self.assertEqual([m for _, m in s], [m for _, m in self.SCHEDULE])
        self.assertAlmostEqual(s[1][0], 23.0, places=6)

    def test_full_schedule_matches(self):
        ok, t0 = cc.schedule_matches(cc.mode_switches(self.modes(self.SCHEDULE)),
                                     self.SCHEDULE)
        self.assertTrue(ok)
        self.assertAlmostEqual(t0, 3.0, places=6)

    def test_recording_started_late(self):
        """Recording starts at 30 s: first observed mode is rtk_float."""
        m = {t: v for t, v in self.modes(self.SCHEDULE).items() if t >= 30e9}
        ok, t0 = cc.schedule_matches(cc.mode_switches(m), self.SCHEDULE)
        self.assertTrue(ok)
        self.assertAlmostEqual(t0, 3.0, places=6)

    def test_wrong_spacing_fails(self):
        shifted = [(0.0, 'rtk_fixed'), (20.0, 'rtk_float'), (45.0, 'standalone'),
                   (60.0, 'outage'), (70.0, 'rtk_fixed')]
        ok, _ = cc.schedule_matches(cc.mode_switches(self.modes(shifted)), self.SCHEDULE)
        self.assertFalse(ok)

    def test_wrong_order_fails(self):
        swapped = [(0.0, 'rtk_fixed'), (20.0, 'standalone'), (40.0, 'rtk_float'),
                   (60.0, 'outage'), (70.0, 'rtk_fixed')]
        ok, _ = cc.schedule_matches(cc.mode_switches(self.modes(swapped)), self.SCHEDULE)
        self.assertFalse(ok)

    def test_one_mode_all_run(self):
        nominal = [(0.0, 'rtk_fixed')]
        ok, t0 = cc.schedule_matches(cc.mode_switches(self.modes(nominal)), nominal)
        self.assertTrue(ok)
        self.assertIsNone(t0)


if __name__ == '__main__':
    unittest.main()
