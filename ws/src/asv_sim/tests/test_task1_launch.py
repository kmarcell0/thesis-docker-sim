"""Tests for boat_params() in launch/task1.launch.py.
Run from ws/src/asv_sim (ROS sourced):   python3 -m unittest discover -s tests -v"""
import copy
import importlib.util
import os
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
CONFIG = os.path.join(PKG, 'config', 'asv.yaml')

# The launch file is not a module on the path (its name has a dot), so load it
# by file name.
_spec = importlib.util.spec_from_file_location(
    'task1_launch', os.path.join(PKG, 'launch', 'task1.launch.py'))
task1_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(task1_launch)


class BoatParams(unittest.TestCase):
    def setUp(self):
        with open(CONFIG, 'r') as f:
            self.asv = yaml.safe_load(f)

    def test_values_from_asv_yaml(self):
        # Expected values typed from asv.yaml (2026-10-20), so a change there
        # shows up here and gets looked at.
        p = task1_launch.boat_params(self.asv)
        expected = {
            'drag_linear': 4.0, 'drag_quadratic': 6.0,
            'max_thrust': 50.0, 'min_thrust': -40.0, 'steer_limit': 1.5708,
            'boat_length': 1.60, 'boat_half_width': 0.44,
        }
        self.assertEqual(set(p), set(expected))
        for k, v in expected.items():
            self.assertAlmostEqual(p[k], v, places=12, msg=k)
            self.assertIsInstance(p[k], float, k)

    def test_missing_key_raises(self):
        for section, key in [('hydrodynamics', 'xU'), ('thrusters', 'max_thrust'),
                             ('hull', 'radius')]:
            asv = copy.deepcopy(self.asv)
            del asv[section][key]
            with self.assertRaises(KeyError, msg=key):
                task1_launch.boat_params(asv)


if __name__ == '__main__':
    unittest.main()
