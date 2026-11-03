"""Tests for tools/measure_buoy_drift.py (M2.7): parsing gz.msgs.Pose_V text
and composing nested-model poses into world positions (no Gazebo).
Run from ws/src/asv_sim:   python3 -m unittest discover -s tests -v"""
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'tools'))
import measure_buoy_drift as mbd  # noqa: E402

# Shape of `gz topic -e` output; fields equal to 0 are left out, as Gazebo does.
SAMPLE = """header {
  stamp {
    sec: 12
    nsec: 500000000
  }
}
pose {
  name: "task_a"
  id: 7
  position {
    x: 10
    z: 0.01
  }
  orientation {
    z: 0.70710678118654757
    w: 0.70710678118654757
  }
}
pose {
  name: "buoy"
  id: 8
  position {
    x: 1
    y: 2
  }
  orientation {
    w: 1
  }
}
pose {
  name: "asv"
  id: 30
  position {
  }
  orientation {
    w: 1
  }
}
pose {
  name: "link"
  id: 9
  position {
  }
  orientation {
    w: 1
  }
}
"""


class TestDrift(unittest.TestCase):
    def test_parse(self):
        stamp, poses = mbd.parse_pose_v(SAMPLE)
        self.assertAlmostEqual(stamp, 12.5)
        self.assertEqual([p[0] for p in poses], ['task_a', 'buoy', 'asv', 'link'])
        self.assertEqual(poses[0][1], (10.0, 0.0, 0.01))
        self.assertEqual(poses[1][1], (1.0, 2.0, 0.0))

    def test_world_position_of_child(self):
        """Task turned by +pi/2 at (10, 0, 0.01): child (1, 2, 0) -> (8, 1, 0.01)."""
        _, poses = mbd.parse_pose_v(SAMPLE)
        out = mbd.world_positions(poses, {'task_a'})
        self.assertEqual(list(out), ['task_a::buoy'])
        for a, b in zip(out['task_a::buoy'], (8.0, 1.0, 0.01)):
            self.assertAlmostEqual(a, b, places=12)

    def test_drift_rows(self):
        buoys = [{'name': 't::b', 'class': 'A-0', 'x': 0.0, 'y': 0.0}]
        rows = mbd.drift({'t::b': (0.0, 0.0, 0.0)}, {'t::b': (3.0, 4.0, -0.1)}, buoys)
        self.assertAlmostEqual(rows[0]['horizontal_m'], 5.0)
        self.assertAlmostEqual(rows[0]['vertical_m'], -0.1)
        self.assertAlmostEqual(rows[0]['from_map_m'], 5.0)

    def test_rotate_identity(self):
        self.assertEqual(mbd.rotate((1, 0, 0, 0), (1.0, 2.0, 3.0)), (1.0, 2.0, 3.0))
        v = mbd.rotate((math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(v[0], 0.0, places=12)
        self.assertAlmostEqual(v[1], 1.0, places=12)


if __name__ == '__main__':
    unittest.main()
