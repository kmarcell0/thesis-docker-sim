"""Tests for asvgen.  Run from ws/src/asv_sim:   python3 -m unittest discover -s tests -v"""
import copy
import math
import os
import sys
import unittest
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'tools'))

from asvgen import analysis, boat as boat_mod, config, geometry as geo, hydrostatics as hydro  # noqa: E402
from asvgen import mass, outputs, sdf_model, sensors  # noqa: E402

CONFIG = os.path.join(PKG, 'config', 'asv.yaml')


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


class Geometry(unittest.TestCase):
    def test_rotation_is_orthonormal(self):
        R = geo.rot([0.3, -0.7, 2.1])
        for i in range(3):
            for j in range(3):
                dot = sum(R[i][k] * R[j][k] for k in range(3))
                self.assertTrue(close(dot, 1.0 if i == j else 0.0, 1e-12))

    def test_yaw_and_roll(self):
        x_after_yaw = geo.mat_vec(geo.rot([0, 0, math.pi / 2]), [1, 0, 0])
        self.assertTrue(all(close(a, b, 1e-12) for a, b in zip(x_after_yaw, [0, 1, 0])))
        self.assertTrue(geo.is_upside_down([math.pi, 0, 0]))
        self.assertFalse(geo.is_upside_down([0.2, 0.1, 3.0]))

    def test_to_parent_and_back(self):
        xyz, rpy, p = [0.4, -0.1, 0.6], [0.2, 0.3, -1.0], [1.0, 2.0, 3.0]
        back = geo.to_child(xyz, rpy, geo.to_parent(xyz, rpy, p))
        self.assertTrue(all(close(a, b, 1e-12) for a, b in zip(back, p)))


class Mass(unittest.TestCase):
    def test_two_point_masses(self):
        m = mass.combine([mass.MassItem('a', 1.0, [1, 0, 0]), mass.MassItem('b', 1.0, [-1, 0, 0])])
        self.assertTrue(close(m.mass, 2.0))
        self.assertEqual(m.cog, [0.0, 0.0, 0.0])
        ixx, iyy, izz, ixy, ixz, iyz = m.sdf_inertia
        self.assertTrue(close(ixx, 0.0) and close(iyy, 2.0) and close(izz, 2.0))

    def test_product_of_inertia_sign(self):
        m = mass.combine([mass.MassItem('a', 1.0, [1, 1, 0]), mass.MassItem('b', 1.0, [-1, -1, 0])])
        self.assertTrue(close(m.sdf_inertia[3], -2.0))   # Ixy = -sum(m x y)

    def test_box_inertia(self):
        self.assertEqual(mass.box(12.0, 1.0, 2.0, 3.0), (13.0, 10.0, 5.0))


class Hydrostatics(unittest.TestCase):
    def test_segment_geometry(self):
        r = 0.11
        self.assertTrue(close(hydro.segment_area(r, r), math.pi * r * r / 2, 1e-12))
        self.assertTrue(close(hydro.segment_area(r, 2 * r), math.pi * r * r, 1e-12))
        self.assertTrue(close(hydro.segment_centroid_below_axis(r, r), 4 * r / (3 * math.pi), 1e-12))

    def test_archimedes(self):
        hull = {'radius': 0.11, 'length': 1.6, 'spacing': 0.66}
        h = hydro.solve(hull, 18.0, [0, 0, 0.1], [[1, 0, 0], [0, 2, 0], [0, 0, 2]])
        displaced = 2 * hull['length'] * hydro.segment_area(hull['radius'], h.draft) * hydro.RHO
        self.assertTrue(close(displaced, 18.0, 1e-6))
        self.assertGreater(h.gm_t, 0)

    def test_overloaded_boat_is_refused(self):
        hull = {'radius': 0.05, 'length': 1.0, 'spacing': 0.5}
        with self.assertRaises(hydro.HydrostaticsError):
            hydro.solve(hull, 50.0, [0, 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])


class Config(unittest.TestCase):
    def setUp(self):
        self.base = config.load(CONFIG)

    def errors_of(self, cfg):
        with self.assertRaises(config.ConfigError) as ctx:
            config.validate(cfg)
        return str(ctx.exception)

    def test_shipped_config_is_valid(self):
        self.assertEqual(config.validate(copy.deepcopy(self.base)), [])

    def test_missing_and_wrong_values(self):
        cfg = copy.deepcopy(self.base)
        del cfg['hull']['length']
        cfg['sensors'][0]['xyz'] = [1, 2]
        msg = self.errors_of(cfg)
        self.assertIn('hull.length: missing', msg)
        self.assertIn('xyz: expected vec3', msg)

    def test_unknown_type_duplicate_name_and_missing_spec_key(self):
        cfg = copy.deepcopy(self.base)
        cfg['sensors'].append(dict(cfg['sensors'][0]))            # duplicate name
        cfg['sensors'].append({'name': 'x', 'type': 'sonar', 'spec': 'livox_mid360', 'xyz': [0, 0, 0]})
        del cfg['sensor_specs']['zed2i_4mm']['baseline']
        msg = self.errors_of(cfg)
        self.assertIn('used twice', msg)
        self.assertIn('"sonar" is not one of', msg)
        self.assertIn('zed2i_4mm.baseline: missing', msg)

    def test_unknown_key_is_only_a_warning(self):
        cfg = copy.deepcopy(self.base)
        cfg['battery']['colour'] = 'blue'
        self.assertTrue(any('battery.colour' in w for w in config.validate(cfg)))

    def test_overrides(self):
        cfg = copy.deepcopy(self.base)
        config.apply_override(cfg, 'sensors.lidar.xyz=[0.5, 0, 0.4]')
        config.apply_override(cfg, 'battery.mass=4')
        self.assertEqual([s for s in cfg['sensors'] if s['name'] == 'lidar'][0]['xyz'], [0.5, 0, 0.4])
        self.assertEqual(cfg['battery']['mass'], 4)
        with self.assertRaises(config.ConfigError):
            config.apply_override(cfg, 'sensors.sonar.xyz=[0,0,0]')


class Layout(unittest.TestCase):
    def build(self, *overrides):
        cfg, _, fp = config.prepare(CONFIG, overrides)
        b = boat_mod.build(cfg, fp)
        return b, analysis.analyze(b)[0]

    def test_shipped_layout_has_no_overlaps(self):
        _, m = self.build()
        self.assertTrue(m['layout']['ok'], m['layout'])

    def test_overlap_is_detected(self):
        _, m = self.build('battery.xyz=[-0.2,0,0.225]')          # onto the electronics box
        pairs = {(c['a'], c['b']) for c in m['layout']['conflicts']}
        self.assertIn(('electronics', 'battery'), pairs)

    def test_component_must_sit_on_the_deck(self):
        _, m = self.build('battery.xyz=[0.2,0,0.15]')
        self.assertTrue(any('into the deck' in p for p in m['layout']['placement']))

    def test_antenna_post_stands_on_the_hull(self):
        b, _ = self.build()
        posts = [mt for mt in b.mounts if mt['owner'] == 'gnss']
        self.assertEqual(len(posts), 2)
        for mt in posts:
            x, y, z = mt['base']
            self.assertTrue(close(z, boat_mod.hull_top(b.cfg, x, y), 1e-12))
            self.assertLess(z, b.deck_top)                       # i.e. on the hull, not the deck

    def test_ray_box(self):
        self.assertTrue(close(analysis.ray_box([0, 0, 0], [1, 0, 0], [1, -1, -1], [2, 1, 1], 10), 1.0))
        self.assertIsNone(analysis.ray_box([0, 0, 0], [0, 1, 0], [1, -1, -1], [2, 1, 1], 10))
        self.assertIsNone(analysis.ray_box([0, 0, 0], [1, 0, 0], [1, -1, -1], [2, 1, 1], 0.5))

    def test_near_field_is_sane(self):
        _, m = self.build()
        nf = m['sensors']['lidar']['near_field']
        self.assertTrue(0 < nf['self_hit_share'] < 1)
        for key in ('coverage_front_lane', 'coverage_near_band'):
            self.assertTrue(0 < nf[key] <= 1)
        self.assertLessEqual(nf['gap_ahead_centreline_m'], nf['gap_ahead_m'])

    def test_blind_cone_at_the_bow(self):
        """An upside-down lidar at the bow leaves its blind cone (steeper than the
        lowest beam) on the water ahead: the gap must match the cone radius."""
        b, m = self.build('sensors.lidar.xyz=[0.8,0,0.5]', 'sensors.lidar.mount=[0.48,0.3]')
        h = 0.5 - b.hydro.z_water
        cone = h / math.tan(math.radians(52))
        gap = m['sensors']['lidar']['near_field']['gap_ahead_centreline_m']
        self.assertAlmostEqual(gap, cone, delta=0.1)               # raster is 0.1 m


class Pipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cfg, _, fp = config.prepare(CONFIG)
        cls.boat = boat_mod.build(cfg, fp)
        cls.metrics, cls.report = analysis.analyze(cls.boat)

    def test_sdf_is_complete(self):
        model = ET.fromstring(sdf_model.to_sdf(self.boat).split('\n', 1)[1]).find('model')
        sdf_sensors = model.find('link').findall('sensor')
        expected = sum(len(p.sdf) for p in self.boat.parts)
        self.assertEqual(len(sdf_sensors), expected)
        self.assertEqual(len(model.findall('joint')), 4)
        self.assertEqual(sum(p.get('name') == 'vrx::Surface' for p in model.findall('plugin')), 2)

    def test_vrx_buoyancy_matches_hydrostatics(self):
        """Re-implement vrx::Surface's force law on the generated SDF: at the
        predicted equilibrium height the total buoyancy must equal the weight."""
        model = ET.fromstring(sdf_model.to_sdf(self.boat).split('\n', 1)[1]).find('model')
        z_base = -self.boat.hydro.z_water                 # base_link height with waterline at z = 0
        force = 0.0
        for plugin in (p for p in model.findall('plugin') if p.get('name') == 'vrx::Surface'):
            r = float(plugin.find('hull_radius').text)
            length = float(plugin.find('hull_length').text)
            points = plugin.find('points').findall('point')
            for pt in points:
                z_point = z_base + float(pt.text.split()[2])
                dz = min(max(0.0 - z_point, 0.0), r)      # immersion, capped at the radius
                area = r * r * math.acos((r - dz) / r) - (r - dz) * math.sqrt(2 * r * dz - dz * dz)
                force += area * length / len(points) * hydro.G * hydro.RHO
        self.assertTrue(close(force, self.boat.total.mass * hydro.G, 1e-6 * force))

    def test_mass_bookkeeping(self):
        total = sum(i.mass for i in self.boat.mass_items + self.boat.pod_items)
        self.assertTrue(close(total, self.boat.total.mass, 1e-9))

    def test_topics_and_frames_unique(self):
        topics = [t.topic for t in outputs.bridge_topics(self.boat)]
        self.assertEqual(len(topics), len(set(topics)))
        children = [f.child for p in self.boat.parts for f in p.frames]
        self.assertEqual(len(children), len(set(children)))

    def test_imu_noise_from_density(self):
        spec = {'rate': 100, 'gyro_noise_density': 1e-4, 'accel_noise_density': 1e-3}
        g, a = sensors.imu_white_noise(spec)
        self.assertTrue(close(g, 1e-3) and close(a, 1e-2))

    def test_upside_down_lidar_looks_down(self):
        lid = self.metrics['sensors']['lidar']
        self.assertTrue(lid['upside_down'])
        self.assertTrue(close(lid['elevation_deg'][0], -52.0, 0.1))

    def test_camera_horizon_row(self):
        cam = self.metrics['sensors']['cam_wide_front']
        spec = self.boat.cfg['sensor_specs']['arducam_ar0234']
        f = (spec['width'] / 2) / math.tan(spec['hfov'] / 2)
        pitch = [s for s in self.boat.cfg['sensors'] if s['name'] == 'cam_wide_front'][0]['rpy'][1]
        self.assertEqual(cam['horizon_row'], round(spec['height'] / 2 - f * math.tan(pitch)))


if __name__ == '__main__':
    unittest.main()
