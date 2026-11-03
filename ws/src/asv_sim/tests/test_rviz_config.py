"""Tests for the RViz live-view config rviz/asv.rviz (M2.8): it parses, its fixed
frame is world, and every topic it shows is one the simulation publishes.
Run from ws/src/asv_sim:   python3 -m unittest discover -s tests -v"""
import os
import unittest

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RVIZ = os.path.join(PKG, 'rviz', 'asv.rviz')
BRIDGE = os.path.join(PKG, 'config', 'asv_bridge.yaml')

# Topics from our own nodes (not in the bridge config).
NODE_TOPICS = {'/asv/autonomy/path'}


def load_rviz():
    with open(RVIZ) as f:
        return yaml.safe_load(f)['Visualization Manager']


class TestRvizConfig(unittest.TestCase):
    def test_fixed_frame_is_world(self):
        self.assertEqual(load_rviz()['Global Options']['Fixed Frame'], 'world')

    def test_lidar_cloud_shown(self):
        clouds = [d for d in load_rviz()['Displays']
                  if d['Class'] == 'rviz_default_plugins/PointCloud2']
        self.assertEqual([d['Topic']['Value'] for d in clouds], ['/asv/lidar/points'])

    def test_tf_display_present(self):
        classes = [d['Class'] for d in load_rviz()['Displays']]
        self.assertIn('rviz_default_plugins/TF', classes)

    def test_topics_exist(self):
        """Every display topic is bridged from Gazebo or published by one of our nodes."""
        with open(BRIDGE) as f:
            bridged = {e['ros_topic_name'] for e in yaml.safe_load(f)}
        for d in load_rviz()['Displays']:
            if 'Topic' in d:
                self.assertIn(d['Topic']['Value'], bridged | NODE_TOPICS, d['Name'])

    def test_latched_path_subscribed_transient_local(self):
        """The follower publishes the route once, transient local (waypoint_follower.cpp);
        a volatile subscriber that starts later would never see it."""
        path = [d for d in load_rviz()['Displays'] if d['Class'] == 'rviz_default_plugins/Path']
        self.assertEqual(path[0]['Topic']['Durability Policy'], 'Transient Local')


if __name__ == '__main__':
    unittest.main()
