// Gate monitor for the Milestone 1 waypoint follower: detects gate crossings
// (gate, direction, offset, pass/fail) and tracks the smallest clearance
// between the boat and every buoy. ROS-free.
// Spec: milestone1_plan.md section 5.6, decisions 10.8 a, 10.9 a, 10.10 a and
// 10.11 b (2026-10-19). Symbols as in docs/glossary.md.
#pragma once

#include <string>
#include <vector>

#include "asv_sim/guidance.hpp"

namespace asv_sim
{

// A round buoy at its nominal position (world frame).
struct Buoy
{
  std::string name;
  Point2 centre;  // c_j, m
  double radius;  // r_j, m, > 0
};

// A gate: two buoys the boat must pass between.
struct Gate
{
  std::string name;
  Buoy red;
  Buoy green;
};

// The boat as a rectangle centred on base_link (mid hull length, between the
// hulls), x forward, y left.
struct BoatFootprint
{
  double length;      // L, m, > 0 (hull length)
  double half_width;  // b_w, m, > 0 (hull spacing / 2 + hull radius)
};

enum class GateDirection
{
  OUTBOUND,  // (b - a) x (p - p_prev) > 0, a = red, b = green (Task 1: eastward)
  INBOUND
};

enum class GateResult
{
  PASS,          // offset inside the band of section 5.6
  FAIL_CLOSE,    // between the buoys, but closer than the margin allows
  FAIL_OUTSIDE   // the boat's centre crossed the gate line outside a buoy centre
};

struct GateEvent
{
  std::string gate;
  GateDirection direction;
  double offset;  // o, m, from the gate centre, > 0 towards the red buoy
  GateResult result;
};

// Smallest clearance seen so far to one buoy.
struct BuoyClearance
{
  std::string name;
  double min_clearance;  // min over time of c_j, m (< 0: overlap)
};

class GateMonitor
{
public:
  // gates: the gates to check (their buoys are also in the clearance check).
  // other_buoys: further buoys for the clearance check only.
  // margin: d_m, m, >= 0, required gap between the boat's side and a buoy.
  // capture: m, >= 0, how far beyond a buoy centre a crossing of the extended
  //   gate line is still reported (as FAIL_OUTSIDE).
  // Throws std::invalid_argument for a gate whose buoys coincide, a radius
  // <= 0, a footprint size <= 0, or a negative margin or capture.
  GateMonitor(
    std::vector<Gate> gates, const std::vector<Buoy> & other_buoys,
    const BoatFootprint & footprint, double margin, double capture);

  // Call once per control step with the boat position p and heading psi
  // (rad, world frame). Returns the gate crossings between the previous call
  // and this one (none on the first call) and updates the clearances.
  std::vector<GateEvent> update(const Point2 & p, double heading);

  // c_j for one buoy at pose (p, psi): distance from the rectangle to the
  // buoy's surface, m; -r_j if the centre is inside the rectangle.
  double clearance(const Buoy & buoy, const Point2 & p, double heading) const;

  const std::vector<BuoyClearance> & clearances() const {return clearances_;}
  // The buoy with the smallest clearance so far (empty name before the
  // first update or when there are no buoys).
  BuoyClearance closest() const;
  bool collision() const {return collision_;}  // any c_j <= 0 seen so far

private:
  std::vector<Gate> gates_;
  std::vector<Buoy> buoys_;  // gate buoys first, then other_buoys
  BoatFootprint footprint_;
  double margin_;
  double capture_;
  std::vector<BuoyClearance> clearances_;
  bool has_previous_ = false;
  Point2 previous_{0.0, 0.0};  // p_prev
  bool collision_ = false;
};

}  // namespace asv_sim
