#include "asv_sim/guidance.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "asv_sim/angles.hpp"

namespace asv_sim
{

Route::Route(std::vector<Point2> waypoints, double switch_distance)
: waypoints_(std::move(waypoints)), switch_distance_(switch_distance)
{
  if (waypoints_.size() < 2) {
    throw std::invalid_argument("Route: needs at least two waypoints");
  }
  if (!(switch_distance_ >= 0.0)) {
    throw std::invalid_argument("Route: switch_distance must be >= 0");
  }
  for (std::size_t i = 0; i + 1 < waypoints_.size(); ++i) {
    const double dx = waypoints_[i + 1].x - waypoints_[i].x;
    const double dy = waypoints_[i + 1].y - waypoints_[i].y;
    const double l = std::hypot(dx, dy);
    if (!(l > 0.0)) {
      throw std::invalid_argument("Route: zero-length segment");
    }
    length_.push_back(l);
    tangent_.push_back({dx / l, dy / l});
    direction_.push_back(std::atan2(dy, dx));
  }
}

void Route::update(const Point2 & p)
{
  // s = (p - W_i) . t_i and e = (p - W_i) . n_i with n_i = (-t_y, t_x), the
  // left normal (milestone1_plan.md 5.1).
  const Point2 & w = waypoints_[segment_];
  const Point2 & t = tangent_[segment_];
  const double rx = p.x - w.x, ry = p.y - w.y;
  along_ = rx * t.x + ry * t.y;
  cross_ = -rx * t.y + ry * t.x;

  // "Passed along the path" test: unlike a distance circle it fires even if
  // the waypoint was missed sideways.
  if (along_ >= length_[segment_] - switch_distance_) {
    if (segment_ + 1 < length_.size()) {
      ++segment_;
      // s and e on the new segment are computed on the next call.
    } else {
      finished_ = true;
    }
  }
}

double Route::remaining_distance() const
{
  double d = length_[segment_] - along_;
  for (std::size_t j = segment_ + 1; j < length_.size(); ++j) {
    d += length_[j];
  }
  return d;
}

Point2 Route::point_on_segment(std::size_t i, double s) const
{
  return {waypoints_[i].x + s * tangent_[i].x, waypoints_[i].y + s * tangent_[i].y};
}

LosGuidance::LosGuidance(double lookahead)
: lookahead_(lookahead)
{
  if (!(lookahead_ > 0.0)) {
    throw std::invalid_argument("LosGuidance: lookahead must be > 0");
  }
}

LosOutput LosGuidance::compute(const Route & route, const Point2 & p) const
{
  // Walk Delta forward from the projection; whatever is left of Delta past
  // the end of segment i carries on along segment i+1, and so on.
  std::size_t i = route.segment();
  double d = std::max(0.0, route.along_track() + lookahead_);
  const std::size_t last = route.num_segments() - 1;
  while (i < last && d > route.segment_length(i)) {
    d -= route.segment_length(i);
    ++i;
  }
  d = std::min(d, route.segment_length(i));

  LosOutput out;
  out.target = route.point_on_segment(i, d);
  out.desired_heading = wrap_angle(std::atan2(out.target.y - p.y, out.target.x - p.x));
  return out;
}

}  // namespace asv_sim
