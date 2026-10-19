#include "asv_sim/gate_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace asv_sim
{

namespace
{

// 2-D cross product u x v (z component).
double cross(double ux, double uy, double vx, double vy) {return ux * vy - uy * vx;}

// True for x > 0; false for zero, negative values and NaN.
bool positive(double x) {return x > 0.0;}

void check_buoy(const Buoy & b)
{
  if (!positive(b.radius)) {
    throw std::invalid_argument("GateMonitor: buoy radius must be > 0 (" + b.name + ")");
  }
}

// Side of the gate line from a = red to b = green: f(q) = (b - a) x (q - a).
double side_value(const Gate & g, const Point2 & q)
{
  return cross(
    g.green.centre.x - g.red.centre.x, g.green.centre.y - g.red.centre.y,
    q.x - g.red.centre.x, q.y - g.red.centre.y);
}

}  // namespace

GateMonitor::GateMonitor(
  std::vector<Gate> gates, const std::vector<Buoy> & other_buoys,
  const BoatFootprint & footprint, double margin, double capture)
: gates_(std::move(gates)), footprint_(footprint), margin_(margin), capture_(capture)
{
  if (!positive(footprint_.length) || !positive(footprint_.half_width)) {
    throw std::invalid_argument("GateMonitor: footprint length and half width must be > 0");
  }
  if (!(margin_ >= 0.0) || !(capture_ >= 0.0)) {
    throw std::invalid_argument("GateMonitor: margin and capture must be >= 0");
  }
  for (const auto & g : gates_) {
    check_buoy(g.red);
    check_buoy(g.green);
    if (!positive(std::hypot(g.green.centre.x - g.red.centre.x,
      g.green.centre.y - g.red.centre.y)))
    {
      throw std::invalid_argument("GateMonitor: gate buoys coincide (" + g.name + ")");
    }
    buoys_.push_back(g.red);
    buoys_.push_back(g.green);
  }
  for (const auto & b : other_buoys) {
    check_buoy(b);
    buoys_.push_back(b);
  }
  for (const auto & b : buoys_) {
    clearances_.push_back({b.name, INFINITY});
  }
}

std::vector<GateEvent> GateMonitor::update(const Point2 & p, double heading)
{
  std::vector<GateEvent> events;

  // Clearance c_j to every buoy; keep the smallest per buoy.
  for (std::size_t j = 0; j < buoys_.size(); ++j) {
    const double c = clearance(buoys_[j], p, heading);
    clearances_[j].min_clearance = std::min(clearances_[j].min_clearance, c);
    if (c <= 0.0) {
      collision_ = true;
    }
  }

  if (has_previous_) {
    const Point2 & q0 = previous_;
    for (const auto & g : gates_) {
      const double f0 = side_value(g, q0);
      const double f1 = side_value(g, p);
      // Half-open sides (f >= 0 vs. f < 0), so a step that lands exactly on
      // the line and the next step that leaves it count as one crossing.
      if ((f0 >= 0.0) == (f1 >= 0.0)) {
        continue;
      }
      // Crossing point q = p_prev + t (p - p_prev); f is linear along the step.
      const double t = f0 / (f0 - f1);
      const Point2 q{q0.x + t * (p.x - q0.x), q0.y + t * (p.y - q0.y)};

      // Offset o = w_g/2 - (q - a).u, u = (b - a)/w_g: > 0 towards red.
      const double dx = g.green.centre.x - g.red.centre.x;
      const double dy = g.green.centre.y - g.red.centre.y;
      const double width = std::hypot(dx, dy);  // w_g
      const double along = ((q.x - g.red.centre.x) * dx + (q.y - g.red.centre.y) * dy) / width;
      const double offset = 0.5 * width - along;
      if (std::fabs(offset) > 0.5 * width + capture_) {
        continue;  // far beyond the gate: not an attempt at this gate
      }

      GateEvent e;
      e.gate = g.name;
      e.offset = offset;
      e.direction = cross(dx, dy, p.x - q0.x, p.y - q0.y) > 0.0 ?
        GateDirection::OUTBOUND : GateDirection::INBOUND;
      // Band of section 5.6, one side per buoy (radii may differ):
      // -(w_g/2 - r_green - b_w - d_m) <= o <= w_g/2 - r_red - b_w - d_m.
      const double room = 0.5 * width - footprint_.half_width - margin_;
      const double max_red = room - g.red.radius;
      const double max_green = room - g.green.radius;
      if (std::fabs(offset) > 0.5 * width) {
        e.result = GateResult::FAIL_OUTSIDE;
      } else if (offset <= max_red && -offset <= max_green) {
        e.result = GateResult::PASS;
      } else {
        e.result = GateResult::FAIL_CLOSE;
      }
      events.push_back(e);
    }
  }

  previous_ = p;
  has_previous_ = true;
  return events;
}

double GateMonitor::clearance(const Buoy & buoy, const Point2 & p, double heading) const
{
  // Buoy centre in the boat frame: d = R(-psi) (c_j - p).
  const double wx = buoy.centre.x - p.x;
  const double wy = buoy.centre.y - p.y;
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  const double dx = c * wx + s * wy;
  const double dy = -s * wx + c * wy;
  // Nearest point k of the rectangle [-L/2, L/2] x [-b_w, b_w]; if the centre
  // is inside, k = d and the distance is 0, so c_j = -r_j.
  const double kx = std::clamp(dx, -0.5 * footprint_.length, 0.5 * footprint_.length);
  const double ky = std::clamp(dy, -footprint_.half_width, footprint_.half_width);
  return std::hypot(dx - kx, dy - ky) - buoy.radius;
}

BuoyClearance GateMonitor::closest() const
{
  BuoyClearance best{"", INFINITY};
  for (const auto & c : clearances_) {
    if (c.min_clearance < best.min_clearance) {
      best = c;
    }
  }
  return best;
}

}  // namespace asv_sim
