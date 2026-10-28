// Unit tests for the geodesy helper (M2.1). The same cases as
// asv_sim/tests/test_geodesy.py; both read test/data/geodesy_points.csv
// (written by asv_sim/tools/geodesy.py), so C++ and Python are checked
// against the same numbers.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "asv_core/geodesy.hpp"

namespace
{

using asv_core::GeoPoint;
using asv_core::LocalTangentPlane;

const GeoPoint kRef{27.367256, -82.451120, 0.0};  // roboboat_course.sdf
constexpr double kTolExact = 1e-9;  // m: C++ = Python, reference -> origin
constexpr double kTolRound = 1e-3;  // m: round trip (Bowring is one-pass)

double rad(double deg) {return deg * M_PI / 180.0;}

// Rough metres between two geodetic points, only for small differences, as a
// tolerance check (angle x semi-major axis).
double geo_distance(const GeoPoint & a, const GeoPoint & b)
{
  const double dlat = rad(a.lat_deg - b.lat_deg) * asv_core::kWgs84A;
  const double dlon = rad(a.lon_deg - b.lon_deg) * asv_core::kWgs84A *
    std::cos(rad(a.lat_deg));
  return std::sqrt(dlat * dlat + dlon * dlon + (a.alt - b.alt) * (a.alt - b.alt));
}

struct SharedPoint
{
  GeoPoint ref;
  Eigen::Vector3d enu;
  GeoPoint geo;  // to_geo(enu), from Python
};

std::vector<SharedPoint> read_points()
{
  std::ifstream f(std::string(ASV_CORE_TEST_DATA_DIR) + "/geodesy_points.csv");
  if (!f) {throw std::runtime_error("cannot open geodesy_points.csv");}
  std::string line;
  std::getline(f, line);  // header
  std::vector<SharedPoint> points;
  while (std::getline(f, line)) {
    std::stringstream ss(line);
    std::vector<double> v;
    std::string cell;
    while (std::getline(ss, cell, ',')) {v.push_back(std::stod(cell));}
    if (v.size() != 9) {throw std::runtime_error("bad row: " + line);}
    points.push_back({{v[0], v[1], v[2]}, {v[3], v[4], v[5]}, {v[6], v[7], v[8]}});
  }
  return points;
}

TEST(LocalTangentPlane, ReferenceMapsToOrigin)
{
  const LocalTangentPlane ltp(kRef);
  EXPECT_LT(ltp.to_enu(kRef).norm(), kTolExact);
  EXPECT_LT(geo_distance(ltp.to_geo(Eigen::Vector3d::Zero()), kRef), kTolExact);
}

TEST(LocalTangentPlane, RoundTrip100mEast100mNorth)
{
  const LocalTangentPlane ltp(kRef);
  const Eigen::Vector3d enu(100.0, 100.0, 0.0);
  const GeoPoint geo = ltp.to_geo(enu);
  EXPECT_LT((ltp.to_enu(geo) - enu).norm(), kTolRound);
  // North-east of the reference really is north and east.
  EXPECT_GT(geo.lat_deg, kRef.lat_deg);
  EXPECT_GT(geo.lon_deg, kRef.lon_deg);
}

TEST(LocalTangentPlane, SmallStepNorthIsMeridianArc)
{
  // Independent formula: meridian radius M = a (1 - e^2) / (1 - e^2 sin^2 phi)^1.5
  const double a = asv_core::kWgs84A, b = asv_core::kWgs84B;
  const double e2 = 1.0 - (b * b) / (a * a);
  const double s = std::sin(rad(kRef.lat_deg));
  const double m = a * (1.0 - e2) / std::pow(1.0 - e2 * s * s, 1.5);
  const double dlat = 1e-5;  // deg, about 1.1 m
  const LocalTangentPlane ltp(kRef);
  const Eigen::Vector3d enu = ltp.to_enu({kRef.lat_deg + dlat, kRef.lon_deg, kRef.alt});
  EXPECT_NEAR(enu.y(), m * rad(dlat), kTolRound);
  EXPECT_NEAR(enu.x(), 0.0, kTolExact);
}

TEST(LocalTangentPlane, SmallStepEastIsParallelArc)
{
  // Independent formula: radius of the parallel = N_G cos(phi)
  const double a = asv_core::kWgs84A, b = asv_core::kWgs84B;
  const double e2 = 1.0 - (b * b) / (a * a);
  const double s = std::sin(rad(kRef.lat_deg));
  const double n_g = a / std::sqrt(1.0 - e2 * s * s);
  const double dlon = 1e-5;  // deg, about 1.0 m
  const LocalTangentPlane ltp(kRef);
  const Eigen::Vector3d enu = ltp.to_enu({kRef.lat_deg, kRef.lon_deg + dlon, kRef.alt});
  EXPECT_NEAR(enu.x(), n_g * std::cos(rad(kRef.lat_deg)) * rad(dlon), kTolRound);
  EXPECT_NEAR(enu.y(), 0.0, kTolRound);
}

TEST(LocalTangentPlane, InvalidReferenceRejected)
{
  EXPECT_THROW(LocalTangentPlane({91.0, 0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(LocalTangentPlane({std::nan(""), 0.0, 0.0}), std::invalid_argument);
}

TEST(SharedPoints, CppMatchesPython)
{
  const auto points = read_points();
  ASSERT_EQ(points.size(), 14u);
  for (const auto & p : points) {
    const LocalTangentPlane ltp(p.ref);
    EXPECT_LT(geo_distance(ltp.to_geo(p.enu), p.geo), kTolExact)
      << "enu " << p.enu.transpose();
  }
}

TEST(SharedPoints, RoundTripEveryPoint)
{
  for (const auto & p : read_points()) {
    const LocalTangentPlane ltp(p.ref);
    EXPECT_LT((ltp.to_enu(p.geo) - p.enu).norm(), kTolRound)
      << "enu " << p.enu.transpose();
  }
}

}  // namespace
