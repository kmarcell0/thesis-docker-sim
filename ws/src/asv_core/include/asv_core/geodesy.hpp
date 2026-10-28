// Geodesy: WGS84 latitude/longitude/height <-> local east/north/up (ENU)
// around a reference point. ROS-free (D13 a).
//
// The maths copies Gazebo's own conversion, so a clean NavSat fix converts
// back to exactly the world-frame position Gazebo started from: gz-sim 8
// sphericalCoordinates() (src/Util.cc) calls gz-math 7.7.0
// SphericalCoordinates::PositionTransform(pos, LOCAL2, SPHERICAL) [SRC].
// The world's heading_deg is 0, so Gazebo's heading rotation is the identity
// and is left out here (read_geo_reference() in tools/geodesy.py rejects
// any other heading). The same maths in Python: asv_sim/tools/geodesy.py.
//
// Symbols (glossary, proposed): phi_G, lambda_G, h_G = geodetic latitude,
// longitude, ellipsoidal height; N_G = prime-vertical radius; a, b = WGS84
// semi-axes; e^2, e'^2 = first and second eccentricity squared.
#pragma once

#include <Eigen/Dense>

namespace asv_core
{

// WGS84 semi-axes as gz-math defines them (g_EarthWGS84AxisEquatorial,
// g_EarthWGS84AxisPolar) [SRC]. e^2 = 1 - b^2/a^2 and e'^2 = a^2/b^2 - 1 are
// derived from them, as in gz-math.
constexpr double kWgs84A = 6378137.0;     // m
constexpr double kWgs84B = 6356752.314245;  // m

// A point in geodetic coordinates, in the units of sensor_msgs/NavSatFix.
struct GeoPoint
{
  double lat_deg;  // phi_G, deg, north positive
  double lon_deg;  // lambda_G, deg, east positive
  double alt;      // h_G, m above the WGS84 ellipsoid
};

// Geodetic -> Earth-centred, Earth-fixed (ECEF) position, m.
//   X = (N_G + h_G) cos(phi_G) cos(lambda_G)
//   Y = (N_G + h_G) cos(phi_G) sin(lambda_G)
//   Z = ((b^2/a^2) N_G + h_G) sin(phi_G),   N_G = a / sqrt(1 - e^2 sin^2(phi_G))
Eigen::Vector3d ecef_from_geo(const GeoPoint & geo);

// ECEF -> geodetic with Bowring's one-pass closed form (no iteration), as in
// gz-math:
//   rho = sqrt(X^2 + Y^2),  theta = atan(Z a / (rho b))
//   phi_G = atan((Z + e'^2 b sin^3(theta)) / (rho - e^2 a cos^3(theta)))
//   lambda_G = atan2(Y, X),  h_G = rho / cos(phi_G) - N_G(phi_G)
GeoPoint geo_from_ecef(const Eigen::Vector3d & ecef);

// The local ENU frame whose origin is a reference point (the world's
// <spherical_coordinates>; design.md section 2), x = east, y = north, z = up.
class LocalTangentPlane
{
public:
  // Throws std::invalid_argument if the reference is not finite or its
  // latitude is outside [-90, 90] deg.
  explicit LocalTangentPlane(const GeoPoint & reference);

  // Geodetic -> ENU, m:  [e n u]^T = R (p - p_0), with p the point's and
  // p_0 the reference's ECEF position and R the ECEF -> ENU rotation of the
  // reference (rows: east, north, up unit vectors).
  Eigen::Vector3d to_enu(const GeoPoint & geo) const;

  // ENU (m) -> geodetic:  p = p_0 + R^T [e n u]^T, then geo_from_ecef(p).
  GeoPoint to_geo(const Eigen::Vector3d & enu) const;

  const GeoPoint & reference() const {return reference_;}

private:
  GeoPoint reference_;
  Eigen::Vector3d reference_ecef_;  // p_0
  Eigen::Matrix3d ecef_to_enu_;     // R
};

}  // namespace asv_core
