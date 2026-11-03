"""Tests for tools/make_buoy_map.py (M2.7): pose composition, the generated
true buoy map, static buoys in the course models.
Run from ws/src/asv_sim:   python3 -m unittest discover -s tests -v"""
import math
import os
import re
import sys
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'tools'))
import make_buoy_map as mbm  # noqa: E402


def by_name(buoys):
    return {b['name']: b for b in buoys}


class TestCompose(unittest.TestCase):
    def test_no_rotation_is_a_shift(self):
        self.assertEqual(mbm.compose((5.0, 0.0, 0.0), (0.0, 1.524)), (5.0, 1.524))

    def test_quarter_turn(self):
        """Task turned by +pi/2: local +x points north, local +y west."""
        x, y = mbm.compose((22.0, 6.0, math.pi / 2), (2.0, 1.0))
        self.assertAlmostEqual(x, 21.0, places=12)
        self.assertAlmostEqual(y, 8.0, places=12)


class TestMap(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.buoys = mbm.build_map()
        cls.named = by_name(cls.buoys)

    def test_task1_gate1_red(self):
        """Done-when of M2.7: task pose (5, 0) x local pose (0, 1.524)."""
        b = self.named['task1_entry_exit::gate1_red_port']
        self.assertAlmostEqual(b['x'], 5.0, places=9)
        self.assertAlmostEqual(b['y'], 1.524, places=9)
        self.assertEqual((b['class'], b['colour']), ('Sur-Mark', 'red'))
        self.assertAlmostEqual(b['radius'], 0.2286, places=9)

    def test_task2_rotation(self):
        """Task 2 is turned by 1.5708 rad: local (0, 1.219) -> (20.781, 6.000)."""
        b = self.named['task2_debris_clearance::ch_gate1_red']
        self.assertAlmostEqual(b['x'], 20.781, places=4)
        self.assertAlmostEqual(b['y'], 6.0, places=4)

    def test_counts_d19a(self):
        counts = {}
        for b in self.buoys:
            counts[b['class']] = counts.get(b['class'], 0) + 1
        self.assertEqual(counts, {'Sur-Mark': 4, 'A-0': 14, 'A-2': 5,
                                  'indicator': 4, 'other': 5})
        self.assertEqual([b['id'] for b in self.buoys], list(range(32)))

    def test_radii_d20a(self):
        """Largest horizontal radius: spheres and cylinders their radius,
        indicator base 18 in square -> half diagonal, vessel 1.2 x 0.5 m."""
        expected = {'Sur-Mark': 0.2286, 'A-0': 0.1015, 'A-2': 0.184,
                    'indicator': math.hypot(0.4572, 0.4572) / 2,
                    'other': math.hypot(0.6, 0.25)}
        for b in self.buoys:
            self.assertAlmostEqual(b['radius'], expected[b['class']], places=4, msg=b['name'])

    def test_no_dock_parts(self):
        self.assertFalse([b for b in self.buoys if 'dock_N::' in b['name']
                          or 'indicator_red' in b['name'] or 'indicator_green' in b['name']])

    def test_committed_file_is_up_to_date(self):
        with open(mbm.OUT) as f:
            self.assertEqual(f.read(), mbm.to_yaml(self.buoys),
                             'config/buoys_gt.yaml is stale: run tools/make_buoy_map.py')

    def test_task1_route_agrees(self):
        """The follower's hand-typed buoys (task1_route.yaml) are on the map
        within 1 mm, with the same radius."""
        with open(os.path.join(PKG, 'config', 'task1_route.yaml')) as f:
            p = yaml.safe_load(f)['waypoint_follower']['ros__parameters']
        coords, radii = p['buoy_coords'], p['buoy_radii']
        for k, name in enumerate(p['buoy_names']):
            x, y = coords[2 * k], coords[2 * k + 1]
            near = [b for b in self.buoys if math.hypot(b['x'] - x, b['y'] - y) < 1e-3]
            self.assertEqual(len(near), 1, name)
            self.assertAlmostEqual(near[0]['radius'], radii[k], places=9, msg=name)


class TestStatic(unittest.TestCase):
    def test_every_mapped_object_is_static(self):
        """Done-when of M2.7 (D7 a, D19 a): every mapped include in the
        rb26_task* models carries <static>true</static>."""
        for task in sorted(os.listdir(mbm.MODELS)):
            if not task.startswith('rb26_task'):
                continue
            for name, model, _, static in mbm.includes(os.path.join(mbm.MODELS, task, 'model.sdf')):
                if mbm.classify(model) is not None:
                    self.assertTrue(static, f'{task}::{name} is not static')

    def test_static_tag_is_inside_include(self):
        """The generator writes <static> inside <include>, where SDF reads it."""
        path = os.path.join(mbm.MODELS, 'rb26_task1_entry_exit', 'model.sdf')
        with open(path) as f:
            self.assertTrue(re.search(r'<include><static>true</static>', f.read()))


class TestErrors(unittest.TestCase):
    def test_unknown_element_raises(self):
        with self.assertRaises(ValueError):
            mbm.classify('rb26_lighthouse_red')

    def test_unknown_colour_raises(self):
        with self.assertRaises(ValueError):
            mbm.classify('rb26_a2_purple')


if __name__ == '__main__':
    unittest.main()
