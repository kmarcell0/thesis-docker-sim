// Route following for the Milestone 1 waypoint follower: route progression
// (Route) and line-of-sight guidance (LosGuidance). ROS-free.
// Spec: milestone1_plan.md sections 5.1 and 5.2, decisions 10.6 a and 10.7 a
// (2026-10-18). Symbols as in docs/glossary.md.
#pragma once

#include <cstddef>
#include <vector>

namespace asv_sim
{

// A point in the world frame (ENU, x east, y north), m.
struct Point2
{
  double x;
  double y;
};

// The route W_0 ... W_{n_w} as a polyline; segment i runs from W_i to
// W_{i+1}. W_0 is the spawn point, from the parameter file (10.15 a).
class Route
{
public:
  // waypoints: W_0 ... W_{n_w}, at least two, no zero-length segment.
  // switch_distance: R_switch, m, >= 0.
  // Throws std::invalid_argument otherwise.
  Route(std::vector<Point2> waypoints, double switch_distance);

  // Call once per control step with the boat position p. Computes s and e on
  // the current segment, then switches to the next segment when
  // s >= l_i - R_switch (at most one switch per call); after a switch, s and
  // e are computed again on the new segment, so they always belong to
  // segment() (M1.9). On the last segment the same test sets finished()
  // instead; finished stays set.
  void update(const Point2 & p);

  std::size_t segment() const {return segment_;}               // i
  std::size_t num_segments() const {return length_.size();}    // n_w
  double along_track() const {return along_;}                  // s, m
  double cross_track() const {return cross_;}                  // e, m (> 0: boat left of the path)
  bool finished() const {return finished_;}

  // d_end: distance left along the route, (l_i - s) + sum of l_j for j > i, m
  // (decision 10.7 a).
  double remaining_distance() const;

  // The point at along-track distance s from W_i on segment i: W_i + s t_i.
  Point2 point_on_segment(std::size_t i, double s) const;

  double segment_direction(std::size_t i) const {return direction_[i];}  // gamma_i, rad
  double segment_length(std::size_t i) const {return length_[i];}        // l_i, m

private:
  // Sets s and e of p on the current segment.
  void measure(const Point2 & p);

  std::vector<Point2> waypoints_;
  std::vector<Point2> tangent_;     // unit vector t_i along segment i
  std::vector<double> direction_;   // gamma_i
  std::vector<double> length_;      // l_i
  double switch_distance_;          // R_switch
  std::size_t segment_ = 0;
  double along_ = 0.0;
  double cross_ = 0.0;
  bool finished_ = false;
};

// Output of the line-of-sight guidance.
struct LosOutput
{
  Point2 target;           // p_LOS, the point the boat aims at, m
  double desired_heading;  // psi_d, rad, wrapped to (-pi, pi]
};

// Line-of-sight guidance (Fossen) with the lookahead carried along the route
// (decision 10.6 a).
class LosGuidance
{
public:
  // lookahead: Delta, m, > 0. Throws std::invalid_argument otherwise.
  explicit LosGuidance(double lookahead);

  // Call after route.update(p) in the same control step. The target p_LOS is
  // found by walking Delta along the route from the boat's projection
  // W_i + s t_i (never before W_i, never past W_{n_w});
  // psi_d = atan2(p_LOS - p). On a straight segment this equals
  // psi_d = gamma_i + atan2(-e, Delta) (milestone1_plan.md 5.2).
  LosOutput compute(const Route & route, const Point2 & p) const;

private:
  double lookahead_;
};

}  // namespace asv_sim
