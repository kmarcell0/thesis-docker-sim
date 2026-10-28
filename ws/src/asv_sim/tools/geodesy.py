#!/usr/bin/env python3
"""Geodesy (M2.1): WGS84 latitude/longitude/height <-> local east/north/up.

The same maths as asv_core/geodesy.hpp (C++), both copied from Gazebo's own
conversion: gz-sim 8 sphericalCoordinates() calls gz-math 7.7.0
SphericalCoordinates::PositionTransform(pos, LOCAL2, SPHERICAL) [SRC]. The
world's heading is 0, so Gazebo's heading rotation is the identity;
read_geo_reference() refuses any other heading. Standard library only.

Writing the C++/Python shared test points (asv_core/test/data):
    python3 tools/geodesy.py --write-test-points <file.csv>
"""
import argparse
import csv
import math
import xml.etree.ElementTree as ET

# WGS84 semi-axes as gz-math defines them [SRC]; e^2, e'^2 derived from them.
A = 6378137.0          # m
B = 6356752.314245     # m
E2 = 1.0 - (B * B) / (A * A)
EP2 = (A * A) / (B * B) - 1.0


def prime_vertical_radius(phi):
    """N_G at latitude phi (rad), m."""
    s = math.sin(phi)
    return A / math.sqrt(1.0 - E2 * s * s)


def ecef_from_geo(lat_deg, lon_deg, alt):
    """Geodetic (deg, deg, m above the ellipsoid) -> ECEF (m)."""
    phi = math.radians(lat_deg)
    lam = math.radians(lon_deg)
    n = prime_vertical_radius(phi)
    return ((n + alt) * math.cos(phi) * math.cos(lam),
            (n + alt) * math.cos(phi) * math.sin(lam),
            ((B * B) / (A * A) * n + alt) * math.sin(phi))


def geo_from_ecef(x, y, z):
    """ECEF (m) -> geodetic (deg, deg, m), Bowring's one-pass closed form."""
    rho = math.hypot(x, y)
    theta = math.atan((z * A) / (rho * B))
    st, ct = math.sin(theta), math.cos(theta)
    phi = math.atan((z + EP2 * B * st * st * st) / (rho - E2 * A * ct * ct * ct))
    lam = math.atan2(y, x)
    h = rho / math.cos(phi) - prime_vertical_radius(phi)
    return math.degrees(phi), math.degrees(lam), h


class LocalTangentPlane:
    """Local ENU frame (x east, y north, z up) around a reference point."""

    def __init__(self, lat_deg, lon_deg, alt):
        if not all(math.isfinite(v) for v in (lat_deg, lon_deg, alt)) or abs(lat_deg) > 90.0:
            raise ValueError('LocalTangentPlane: invalid reference point')
        self.reference = (lat_deg, lon_deg, alt)
        self._p0 = ecef_from_geo(lat_deg, lon_deg, alt)
        sp, cp = math.sin(math.radians(lat_deg)), math.cos(math.radians(lat_deg))
        sl, cl = math.sin(math.radians(lon_deg)), math.cos(math.radians(lon_deg))
        # Rows: east, north, up unit vectors of the reference, in ECEF.
        self._r = ((-sl, cl, 0.0),
                   (-cl * sp, -sl * sp, cp),
                   (cl * cp, sl * cp, sp))

    def to_enu(self, lat_deg, lon_deg, alt):
        """Geodetic -> (e, n, u) in m: R (p - p_0)."""
        p = ecef_from_geo(lat_deg, lon_deg, alt)
        d = [p[i] - self._p0[i] for i in range(3)]
        return tuple(sum(row[j] * d[j] for j in range(3)) for row in self._r)

    def to_geo(self, e, n, u):
        """(e, n, u) in m -> geodetic: p = p_0 + R^T [e n u]^T, then Bowring."""
        enu = (e, n, u)
        p = [self._p0[j] + sum(self._r[i][j] * enu[i] for i in range(3)) for j in range(3)]
        return geo_from_ecef(*p)


def read_geo_reference(sdf_path):
    """(lat_deg, lon_deg, elevation) of the world's <spherical_coordinates>.

    Raises ValueError unless the surface is WGS84, the world frame is ENU and
    the heading is 0 (the conversion here leaves the heading rotation out)."""
    root = ET.parse(sdf_path).getroot()
    sc = root.find('./world/spherical_coordinates')
    if sc is None:
        raise ValueError(f'{sdf_path}: no <spherical_coordinates>')

    def text(tag, default):
        el = sc.find(tag)
        return el.text.strip() if el is not None and el.text else default

    # Defaults as in the SDF specification.
    if text('surface_model', 'EARTH_WGS84') != 'EARTH_WGS84':
        raise ValueError(f'{sdf_path}: surface_model is not EARTH_WGS84')
    if text('world_frame_orientation', 'ENU') != 'ENU':
        raise ValueError(f'{sdf_path}: world_frame_orientation is not ENU')
    if float(text('heading_deg', '0')) != 0.0:
        raise ValueError(f'{sdf_path}: heading_deg is not 0')
    return (float(text('latitude_deg', '0')), float(text('longitude_deg', '0')),
            float(text('elevation', '0')))


# Shared test points: reference (deg, deg, m) and ENU offsets (m). The first
# reference is the world's (roboboat_course.sdf); the second is in the
# southern and eastern hemisphere, to catch sign errors.
TEST_REFERENCES = [(27.367256, -82.451120, 0.0), (-33.8568, 151.2153, 25.0)]
TEST_OFFSETS = [(0.0, 0.0, 0.0), (100.0, 100.0, 0.0), (-250.0, 40.0, 1.5),
                (1000.0, -1000.0, 0.0), (0.0, 0.0, 10.0), (-5000.0, 3000.0, -2.0),
                (-0.76, -0.33, 0.185)]
CSV_FIELDS = ['ref_lat_deg', 'ref_lon_deg', 'ref_alt', 'e', 'n', 'u',
              'lat_deg', 'lon_deg', 'alt']


def test_point_rows():
    """Each row: reference, ENU offset, and to_geo() of the offset."""
    rows = []
    for ref in TEST_REFERENCES:
        ltp = LocalTangentPlane(*ref)
        for enu in TEST_OFFSETS:
            rows.append(list(ref) + list(enu) + list(ltp.to_geo(*enu)))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--write-test-points', metavar='CSV', required=True)
    args = ap.parse_args()
    with open(args.write_test_points, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(CSV_FIELDS)
        for row in test_point_rows():
            w.writerow([repr(v) for v in row])  # repr: round-trips exactly


if __name__ == '__main__':
    main()
