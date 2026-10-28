"""Tests for tools/geodesy.py (M2.1).
The same cases as asv_core/test/test_geodesy.cpp; both read the shared points in
asv_core/test/data/geodesy_points.csv, so C++ and Python are checked against the
same numbers.
Run from ws/src/asv_sim:   python3 -m unittest discover -s tests -v"""
import csv
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'tools'))
import geodesy as gd  # noqa: E402

WORLD_SDF = os.path.join(PKG, 'worlds', 'roboboat_course.sdf')
POINTS_CSV = os.path.join(os.path.dirname(PKG), 'asv_core', 'test', 'data',
                          'geodesy_points.csv')
REF = (27.367256, -82.451120, 0.0)  # roboboat_course.sdf
TOL_EXACT = 1e-9   # m: C++ = Python, reference -> origin
TOL_ROUND = 1e-3   # m: round trip (Bowring's one-pass inverse is not exact)


def geo_distance(a, b):
    """Rough metres between two (lat_deg, lon_deg, alt): only for small
    differences, as a tolerance check (angle x semi-major axis)."""
    dlat = math.radians(a[0] - b[0]) * gd.A
    dlon = math.radians(a[1] - b[1]) * gd.A * math.cos(math.radians(a[0]))
    return math.sqrt(dlat ** 2 + dlon ** 2 + (a[2] - b[2]) ** 2)


def read_points():
    with open(POINTS_CSV, newline='') as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


class TestLocalTangentPlane(unittest.TestCase):
    def setUp(self):
        self.ltp = gd.LocalTangentPlane(*REF)

    def test_reference_maps_to_origin(self):
        for v in self.ltp.to_enu(*REF):
            self.assertLess(abs(v), TOL_EXACT)
        geo = self.ltp.to_geo(0.0, 0.0, 0.0)
        self.assertLess(geo_distance(geo, REF), TOL_EXACT)

    def test_round_trip_100m_east_100m_north(self):
        geo = self.ltp.to_geo(100.0, 100.0, 0.0)
        e, n, u = self.ltp.to_enu(*geo)
        self.assertLess(math.dist((e, n, u), (100.0, 100.0, 0.0)), TOL_ROUND)
        # North-east of the reference really is north and east.
        self.assertGreater(geo[0], REF[0])
        self.assertGreater(geo[1], REF[1])

    def test_small_step_north_is_meridian_arc(self):
        # Independent formula: meridian radius M = a (1 - e^2) / (1 - e^2 sin^2 phi)^1.5
        phi = math.radians(REF[0])
        m = gd.A * (1 - gd.E2) / (1 - gd.E2 * math.sin(phi) ** 2) ** 1.5
        dlat = 1e-5  # deg, about 1.1 m
        e, n, _ = self.ltp.to_enu(REF[0] + dlat, REF[1], REF[2])
        self.assertLess(abs(n - m * math.radians(dlat)), TOL_ROUND)
        self.assertLess(abs(e), TOL_EXACT)

    def test_small_step_east_is_parallel_arc(self):
        # Independent formula: radius of the parallel = N_G cos(phi)
        phi = math.radians(REF[0])
        dlon = 1e-5  # deg, about 1.0 m
        e, n, _ = self.ltp.to_enu(REF[0], REF[1] + dlon, REF[2])
        expected = gd.prime_vertical_radius(phi) * math.cos(phi) * math.radians(dlon)
        self.assertLess(abs(e - expected), TOL_ROUND)
        self.assertLess(abs(n), TOL_ROUND)

    def test_invalid_reference_rejected(self):
        with self.assertRaises(ValueError):
            gd.LocalTangentPlane(91.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            gd.LocalTangentPlane(float('nan'), 0.0, 0.0)


class TestSharedPoints(unittest.TestCase):
    def test_csv_matches_python(self):
        # The CSV is written by geodesy.py; this catches a stale file.
        rows = read_points()
        fresh = gd.test_point_rows()
        self.assertEqual(len(rows), len(fresh))
        for row, new in zip(rows, fresh):
            self.assertLess(geo_distance((row['lat_deg'], row['lon_deg'], row['alt']),
                                         new[6:9]), TOL_EXACT)

    def test_round_trip_every_point(self):
        for row in read_points():
            ltp = gd.LocalTangentPlane(row['ref_lat_deg'], row['ref_lon_deg'], row['ref_alt'])
            enu = ltp.to_enu(row['lat_deg'], row['lon_deg'], row['alt'])
            self.assertLess(math.dist(enu, (row['e'], row['n'], row['u'])), TOL_ROUND)


class TestReadGeoReference(unittest.TestCase):
    def test_world_reference(self):
        self.assertEqual(gd.read_geo_reference(WORLD_SDF), REF)

    def write_world(self, inner):
        f = tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False)
        f.write(f'<sdf version="1.9"><world name="w"><spherical_coordinates>{inner}'
                '</spherical_coordinates></world></sdf>')
        f.close()
        self.addCleanup(os.remove, f.name)
        return f.name

    def test_rejects_heading(self):
        path = self.write_world('<latitude_deg>1</latitude_deg><heading_deg>90</heading_deg>')
        with self.assertRaises(ValueError):
            gd.read_geo_reference(path)

    def test_rejects_other_surface(self):
        path = self.write_world('<surface_model>MOON_SCS</surface_model>')
        with self.assertRaises(ValueError):
            gd.read_geo_reference(path)

    def test_rejects_other_orientation(self):
        path = self.write_world('<world_frame_orientation>NED</world_frame_orientation>')
        with self.assertRaises(ValueError):
            gd.read_geo_reference(path)


if __name__ == '__main__':
    unittest.main()
