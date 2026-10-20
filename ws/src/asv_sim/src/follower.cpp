#include "asv_sim/follower.hpp"

#include <cstdio>
#include <stdexcept>
#include <utility>

namespace asv_sim
{

namespace
{

const FollowerConfig & checked(const FollowerConfig & c)
{
  if (c.waypoints.size() < 2 || c.speeds.size() != c.waypoints.size() - 1) {
    throw std::invalid_argument("Follower: need one speed per route segment");
  }
  if (!(c.start_delay >= 0.0)) {
    throw std::invalid_argument("Follower: start_delay must be >= 0");
  }
  if (!(c.max_run_time > 0.0) || !(c.odom_timeout > 0.0)) {
    throw std::invalid_argument("Follower: max_run_time and odom_timeout must be > 0");
  }
  return c;
}

}  // namespace

Follower::Follower(const FollowerConfig & config)
: config_(checked(config)),
  route_(config.waypoints, config.switch_distance),
  los_(config.lookahead),
  heading_(config.heading),
  speed_(config.speed),
  monitor_(config.gates, config.other_buoys, config.footprint, config.gate_margin,
    config.gate_capture)
{
}

FollowerOutput Follower::hold(const std::string & status) const
{
  FollowerOutput out;
  out.state = state_;
  out.status = status;
  out.command = {0.0, 0.0, 0.0, 0.0};
  return out;
}

std::string Follower::running_status() const
{
  return "RUNNING " + std::to_string(route_.segment() + 1) + "/" +
         std::to_string(route_.num_segments());
}

FollowerOutput Follower::step(double t, const std::optional<NavState> & nav)
{
  // The monitor first, so it sees every new pose in every state.
  std::vector<GateEvent> events;
  if (nav && (!monitor_stamp_ || nav->stamp != *monitor_stamp_)) {
    events = monitor_.update(nav->p, nav->heading);
    monitor_stamp_ = nav->stamp;
  }
  auto finish = [&events](FollowerOutput out) {
      out.events = std::move(events);
      return out;
    };

  if (state_ == FollowerState::FINISHED) {
    return finish(hold("FINISHED"));
  }
  if (state_ == FollowerState::STOPPED) {
    return finish(hold("STOPPED: " + stop_reason_));
  }
  if (!nav) {
    return finish(hold("WAITING"));
  }
  if (t - nav->stamp > config_.odom_timeout) {
    state_ = FollowerState::STOPPED;
    stop_reason_ = "odom timeout";
    return finish(hold("STOPPED: " + stop_reason_));
  }
  if (!first_stamp_) {
    first_stamp_ = nav->stamp;
  }
  if (t - *first_stamp_ < config_.start_delay) {
    return finish(hold("WAITING"));
  }
  if (!run_start_) {
    run_start_ = t;
    state_ = FollowerState::RUNNING;
  }
  if (t - *run_start_ > config_.max_run_time) {
    state_ = FollowerState::STOPPED;
    stop_reason_ = "time limit";
    return finish(hold("STOPPED: " + stop_reason_));
  }

  route_.update(nav->p);
  if (route_.finished()) {
    state_ = FollowerState::FINISHED;
    return finish(hold("FINISHED"));
  }

  // Control chain (milestone1_plan.md 5.2-5.5), as in test_closed_loop.
  const LosOutput target = los_.compute(route_, nav->p);
  // Course feedback (10.13 a): steer the direction of motion chi.
  const double chi = course_angle(nav->heading, nav->u, nav->v, config_.course_min_speed);
  const HeadingCommand hc = heading_.compute(target.desired_heading, chi, nav->yaw_rate);
  const double u_cmd = speed_.commanded_speed(
    config_.speeds[route_.segment()], hc.heading_error, route_.remaining_distance());
  // dt from the stamps (CLAUDE.md, time rules); 0 skips the integration.
  const double dt = control_stamp_ ? nav->stamp - *control_stamp_ : 0.0;
  control_stamp_ = nav->stamp;
  const double throttle = speed_.update(u_cmd, nav->u, hc.heading_error, dt);

  FollowerOutput out;
  out.state = state_;
  out.status = running_status();
  out.command = mix(throttle, hc.steer, hc.spin, config_.limits);
  out.has_target = true;
  out.target = target.target;
  out.cross_track = route_.cross_track();
  return finish(out);
}

std::string gate_event_text(const GateEvent & event)
{
  const char * result = event.result == GateResult::PASS ? "PASS" :
    event.result == GateResult::FAIL_CLOSE ? "FAIL_CLOSE" : "FAIL_OUTSIDE";
  char buf[160];
  std::snprintf(buf, sizeof(buf), "GATE %s %s offset %+.3f m %s", event.gate.c_str(),
    event.direction == GateDirection::OUTBOUND ? "OUTBOUND" : "INBOUND", event.offset,
    result);
  return buf;
}

}  // namespace asv_sim
