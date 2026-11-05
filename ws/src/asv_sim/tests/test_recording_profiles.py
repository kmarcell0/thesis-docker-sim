"""Tests for the recording profiles (M2.9): config/recording_profiles.yaml,
tools/record.py and tools/check_bag_rates.py.
Run from ws/src/asv_sim:   python3 -m unittest discover -s tests -v"""
import contextlib
import io
import os
import re
import sys
import tempfile
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'tools'))
import check_bag_rates as cbr  # noqa: E402
import record  # noqa: E402


def read_yaml(*parts):
    with open(os.path.join(*parts)) as f:
        return yaml.safe_load(f)


class TestProfiles(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profiles = record.load_profiles()
        cls.nav = record.profile_topics(cls.profiles, 'nav')
        cls.nav_lidar = record.profile_topics(cls.profiles, 'nav_lidar')
        cls.perception = record.profile_topics(cls.profiles, 'perception')

    def test_profiles_nest(self):
        """nav < nav_lidar < perception, each adding the planned topics."""
        self.assertLess(set(self.nav), set(self.nav_lidar))
        self.assertLess(set(self.nav_lidar), set(self.perception))
        self.assertEqual(set(self.nav_lidar) - set(self.nav), {'/asv/lidar/points'})
        self.assertEqual(set(self.perception) - set(self.nav_lidar),
                         {'/asv/zed/left/image_raw', '/asv/zed/left/camera_info'})

    def test_nav_has_what_replay_needs(self):
        """design.md section 8: clock, odom, clean IMU, both clean fixes, thrusters;
        no corrupted topic (slam_replay corrupts offline, D4 a)."""
        for t in ('/clock', '/asv/ground_truth/odom', '/asv/imu/data', '/asv/gnss/fix',
                  '/asv/gnss/secondary/fix', '/asv/thrusters/left/thrust',
                  '/asv/thrusters/left/pos', '/asv/thrusters/right/thrust',
                  '/asv/thrusters/right/pos'):
            self.assertIn(t, self.nav)
        self.assertFalse([t for t in self.perception if t.startswith('/asv/sensors/')])

    def test_every_topic_exists(self):
        """Each topic is bridged (asv_bridge.yaml) or named as a string in one
        of our nodes (src/*.cpp), so a typo can't silently record nothing."""
        bridged = {e['ros_topic_name']
                   for e in read_yaml(PKG, 'config', 'asv_bridge.yaml')}
        sources = ''
        for name in os.listdir(os.path.join(PKG, 'src')):
            with open(os.path.join(PKG, 'src', name)) as f:
                sources += f.read()
        for t in self.perception:
            self.assertTrue(t in bridged or f'"{t}"' in sources, t)

    def test_rates_match_asv_yaml(self):
        """The checked rates are copies of asv.yaml and the generator (the
        camera is not checked: 15.15 Hz on the 4 ms step grid, M2.9)."""
        specs = read_yaml(PKG, 'config', 'asv.yaml')['sensor_specs']
        expected = {
            '/asv/imu/data': specs['pixhawk6x_icm45686']['rate'],
            '/asv/gnss/fix': specs['unicore_um982_rtk']['rate'],
            '/asv/gnss/secondary/fix': specs['unicore_um982_rtk']['rate'],
            '/asv/lidar/points': specs['livox_mid360']['rate'],
        }
        with open(os.path.join(PKG, 'tools', 'asvgen', 'boat.py')) as f:
            odom = re.search(r"'odom_publish_frequency', ([0-9.]+)", f.read())
        expected['/asv/ground_truth/odom'] = float(odom.group(1))
        checked = {t: r for t, r in self.perception.items() if r is not None}
        self.assertEqual(checked, expected)

    def test_extends_cycle_and_duplicates(self):
        with self.assertRaises(ValueError):
            record.profile_topics({'a': {'extends': 'b', 'topics': {}},
                                   'b': {'extends': 'a', 'topics': {}}}, 'a')
        with self.assertRaises(ValueError):
            record.profile_topics({'a': {'topics': {'/x': 1}},
                                   'b': {'extends': 'a', 'topics': {'/x': 1}}}, 'b')
        with self.assertRaises(KeyError):
            record.profile_topics(self.profiles, 'everything')

    def test_record_command(self):
        cmd = record.record_command({'/a': 1, '/b': None}, '/ws/data/bags/x')
        self.assertEqual(cmd, ['ros2', 'bag', 'record', '--use-sim-time', '-o',
                               '/ws/data/bags/x', '--topics', '/a', '/b'])


class TestCheck(unittest.TestCase):
    TOPICS = {'/imu': 200, '/fix': 10, '/status': None}

    def test_counts(self):
        """60 s: IMU 12 000 +- 120 passes, 11 800 fails; a missing topic fails."""
        rows = cbr.check_counts(self.TOPICS, 60.0, {'/imu': 11881, '/fix': 600})
        self.assertEqual([r[4] for r in rows], [True, True, None])
        rows = cbr.check_counts(self.TOPICS, 60.0, {'/imu': 11800, '/fix': 600})
        self.assertFalse(rows[0][4])
        rows = cbr.check_counts(self.TOPICS, 60.0, {'/imu': 12000})
        self.assertEqual(rows[1][2], 0)
        self.assertFalse(rows[1][4])

    def test_metadata_and_exit_code(self):
        """A fake metadata.yaml in the rosbag2 layout gives PASS (0) / FAIL (1)."""
        def write(bag, imu):
            meta = {'rosbag2_bagfile_information': {
                'duration': {'nanoseconds': 10_000_000_000},
                'topics_with_message_count': [
                    {'topic_metadata': {'name': '/asv/imu/data'}, 'message_count': imu},
                    {'topic_metadata': {'name': '/asv/gnss/fix'}, 'message_count': 100},
                ]}}
            with open(os.path.join(bag, 'metadata.yaml'), 'w') as f:
                yaml.safe_dump(meta, f)

        with tempfile.TemporaryDirectory() as bag:
            write(bag, 2000)
            duration, counts = cbr.read_metadata(bag)
            self.assertAlmostEqual(duration, 10.0)
            self.assertEqual(counts, {'/asv/imu/data': 2000, '/asv/gnss/fix': 100})
            profiles = {'p': {'topics': {'/asv/imu/data': 200, '/asv/gnss/fix': 10}}}
            orig = record.load_profiles
            record.load_profiles = lambda path=None: profiles
            out = io.StringIO()
            try:
                with contextlib.redirect_stdout(out):
                    self.assertEqual(cbr.main([bag, '--profile', 'p']), 0)
                    write(bag, 1900)
                    self.assertEqual(cbr.main([bag, '--profile', 'p']), 1)
            finally:
                record.load_profiles = orig


if __name__ == '__main__':
    unittest.main()
