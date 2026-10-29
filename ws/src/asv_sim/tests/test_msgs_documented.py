"""Tests for the asv_msgs message definitions (M2.4): every field is documented
with a unit, the planned types exist, and the generated Python types carry the
planned constant values. Needs a built and sourced workspace (asv_msgs import).
Run from ws/src/asv_sim:   python3 -m unittest discover -s tests -v"""
import os
import re
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
MSG_DIR = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'asv_msgs', 'msg')

PLANNED = {'BuoyColour', 'BuoyDetection', 'BuoyDetectionArray', 'Landmark',
           'LandmarkArray', 'GnssHeading', 'GnssStatus'}

# A field line: "<type> <name>" (no "=", which would make it a constant).
FIELD = re.compile(r'^\s*([A-Za-z_][\w/]*(\[\d*\])?)\s+([a-z_]\w*)\s*(#.*)?$')
UNIT = re.compile(r'#\s*\[[^\]]+\]')  # comment starting with a unit in brackets


def field_lines(path):
    """(line number, text) of every field line, skipping comments, blanks and constants."""
    out = []
    with open(path) as f:
        for n, line in enumerate(f, 1):
            code = line.split('#', 1)[0].strip()
            if not code or '=' in code:
                continue
            out.append((n, line.rstrip('\n')))
    return out


class TestMsgFiles(unittest.TestCase):
    def test_planned_types_exist(self):
        found = {f[:-4] for f in os.listdir(MSG_DIR) if f.endswith('.msg')}
        self.assertEqual(found, PLANNED)

    def test_every_field_has_unit(self):
        """Each field has a trailing comment that starts with [unit] ([-] = no unit)."""
        for name in sorted(PLANNED):
            path = os.path.join(MSG_DIR, name + '.msg')
            lines = field_lines(path)
            self.assertTrue(lines, f'{name}: no fields')
            for n, line in lines:
                with self.subTest(msg=name, line=n):
                    self.assertRegex(line, FIELD)
                    self.assertRegex(line, UNIT)

    def test_spatial_types_name_their_frame(self):
        """Types with a header or a position say which frame they are in."""
        for name in PLANNED - {'BuoyColour', 'GnssStatus'}:
            with open(os.path.join(MSG_DIR, name + '.msg')) as f:
                self.assertRegex(f.read(), r'(?i)frame', name)


class TestGeneratedPython(unittest.TestCase):
    def test_constants(self):
        from asv_msgs.msg import BuoyColour, BuoyDetection, GnssStatus
        self.assertEqual(
            (BuoyColour.UNKNOWN, BuoyColour.RED, BuoyColour.GREEN,
             BuoyColour.YELLOW, BuoyColour.BLACK), (0, 1, 2, 3, 4))
        self.assertEqual((BuoyDetection.CLASS_BUOY, BuoyDetection.CLASS_OTHER), (0, 1))
        self.assertEqual(
            (GnssStatus.MODE_NO_FIX, GnssStatus.MODE_STANDALONE,
             GnssStatus.MODE_RTK_FLOAT, GnssStatus.MODE_RTK_FIXED), (0, 1, 2, 3))

    def test_defaults(self):
        """A fresh message is all zeros: colour UNKNOWN, landmark covariance 2x2."""
        from asv_msgs.msg import BuoyDetection, Landmark, LandmarkArray
        self.assertEqual(BuoyDetection().colour.value, 0)
        self.assertEqual(len(Landmark().covariance), 4)
        self.assertEqual(list(LandmarkArray().landmarks), [])


if __name__ == '__main__':
    unittest.main()
