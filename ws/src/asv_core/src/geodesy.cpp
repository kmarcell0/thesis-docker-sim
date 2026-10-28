#include "asv_core/geodesy.hpp"

#include <cmath>
#include <stdexcept>

namespace asv_core
{

namespace
{

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

// e^2 and e'^2 from the semi-axes, as gz-math computes them.
const double kE2 = 1.0 - (kWgs84B * kWgs84B) / (kWgs84A * kWgs84A);
const double kEp2 = (kWgs84A * kWgs84A) / (kWgs84B * kWgs84B) - 1.0;

// Prime-vertical radius of curvature N_G at latitude phi (rad), m.
double prime_vertical_radius(double phi)
{
  const double s = std::sin(phi);
  return kWgs84A / std::sqrt(1.0 - kE2 * s * s);
}

}  // namespace

Eigen::Vector3d ecef_from_geo(const GeoPoint & geo)
{
  const double phi = geo.lat_deg * kDegToRad;
  const double lambda = geo.lon_deg * kDegToRad;
  const double n = prime_vertical_radius(phi);
  return {
    (n + geo.alt) * std::cos(phi) * std::cos(lambda),
    (n + geo.alt) * std::cos(phi) * std::sin(lambda),
    ((kWgs84B * kWgs84B) / (kWgs84A * kWgs84A) * n + geo.alt) * std::sin(phi)};
}

GeoPoint geo_from_ecef(const Eigen::Vector3d & ecef)
{
  const double rho = std::hypot(ecef.x(), ecef.y());
  // theta: parametric (reduced) latitude, Bowring's starting guess.
  const double theta = std::atan((ecef.z() * kWgs84A) / (rho * kWgs84B));
  const double st = std::sin(theta);
  const double ct = std::cos(theta);
  const double phi = std::atan(
    (ecef.z() + kEp2 * kWgs84B * st * st * st) /
    (rho - kE2 * kWgs84A * ct * ct * ct));
  const double lambda = std::atan2(ecef.y(), ecef.x());
  const double h = rho / std::cos(phi) - prime_vertical_radius(phi);
  return {phi * kRadToDeg, lambda * kRadToDeg, h};
}

LocalTangentPlane::LocalTangentPlane(const GeoPoint & reference)
: reference_(reference)
{
  if (!std::isfinite(reference.lat_deg) || !std::isfinite(reference.lon_deg) ||
    !std::isfinite(reference.alt) || std::abs(reference.lat_deg) > 90.0)
  {
    throw std::invalid_argument("LocalTangentPlane: invalid reference point");
  }
  reference_ecef_ = ecef_from_geo(reference);

  const double sp = std::sin(reference.lat_deg * kDegToRad);
  const double cp = std::cos(reference.lat_deg * kDegToRad);
  const double sl = std::sin(reference.lon_deg * kDegToRad);
  const double cl = std::cos(reference.lon_deg * kDegToRad);
  // Rows: the east, north and up unit vectors of the reference, in ECEF
  // (gz-math rotECEFToGlobal).
  ecef_to_enu_ <<
    -sl, cl, 0.0,
    -cl * sp, -sl * sp, cp,
    cl * cp, sl * cp, sp;
}

Eigen::Vector3d LocalTangentPlane::to_enu(const GeoPoint & geo) const
{
  return ecef_to_enu_ * (ecef_from_geo(geo) - reference_ecef_);
}

GeoPoint LocalTangentPlane::to_geo(const Eigen::Vector3d & enu) const
{
  return geo_from_ecef(reference_ecef_ + ecef_to_enu_.transpose() * enu);
}

}  // namespace asv_core
