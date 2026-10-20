// Maneuver files on the 3-DOF model (M1.7a): each config/maneuvers/*.yaml is
// played through ManeuverPlayer on the test boat model (boat_model.hpp) from
// the spawn pose, plus a coast with zero thrust. Checks, before any Gazebo
// run, that the open-loop path keeps clear of the Task 1 route's buoys
// (config/task1_route.yaml) and of the other course elements (kZones). The model has no waves: Gazebo will drift from it.
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "asv_sim/gate_monitor.hpp"
#include "asv_sim/maneuver.hpp"
#include "boat_model.hpp"

namespace
{

using asv_sim::Buoy;
using asv_sim::GateMonitor;
using asv_sim::Maneuver;
using asv_sim::ManeuverPlayer;
using asv_sim::PlayerState;

constexpr double kControlDt = 0.05;  // s, the player's 20 Hz loop
constexpr int kSubSteps = 20;        // model steps per control step (2.5 ms)
constexpr double kCoastTime = 10.0;  // s with zero thrust after DONE
// m, required gap to every buoy and zone [EST] (decision Q4 a, 2026-10-20:
// a pass through a Task 1 gate centre gives 0.855 m, so 1.0 m is impossible).
constexpr double kMinClearance = 0.5;

// Keep-out rectangles around the course elements that are not in the
// buoy list (world frame, m) [REPO: task poses in worlds/roboboat_course.sdf
// x element poses and sizes in models/rb26_task*/ and their parts]. There is
// no shoreline in the world; only these elements and the buoys limit a path.
struct Zone
{
  const char * name;
  double x_min, x_max, y_min, y_max;
};
constexpr Zone kZones[] = {
  {"task2", 16.5, 27.5, 6.0, 39.0},     // channel gates, debris, indicators, vessel
  {"task3", 30.0, 50.0, -12.0, -2.5},   // gate, indicator, obstacles, mark, vessel
  {"task5", 17.5, 20.3, -32.6, -17.4},  // two piers (1.0 x 6.6 m), indicators, vessels
};

// Distance from p to the rectangle (0 inside), m.
double zone_distance(const Zone & z, double x, double y)
{
  const double dx = std::max({z.x_min - x, 0.0, x - z.x_max});
  const double dy = std::max({z.y_min - y, 0.0, y - z.y_max});
  return std::hypot(dx, dy);
}

const asv_sim_test::BoatParams & params()
{
  static const auto p = asv_sim_test::load_boat_params(ASV_CONFIG_DIR);
  return p;
}

// All buoys of the follower's parameter file (gates and others).
std::vector<Buoy> task1_buoys()
{
  const YAML::Node p =
    YAML::LoadFile(std::string(ASV_CONFIG_DIR) + "/task1_route.yaml")
    ["waypoint_follower"]["ros__parameters"];
  const auto names = p["buoy_names"].as<std::vector<std::string>>();
  const auto coords = p["buoy_coords"].as<std::vector<double>>();
  const auto radii = p["buoy_radii"].as<std::vector<double>>();
  std::vector<Buoy> buoys;
  for (std::size_t i = 0; i < names.size(); ++i) {
    buoys.push_back({names[i], {coords[2 * i], coords[2 * i + 1]}, radii[i]});
  }
  return buoys;
}

struct ManeuverFile
{
  Maneuver maneuver;
  double start_delay;
};

ManeuverFile load_maneuver(const std::string & name)
{
  const YAML::Node p =
    YAML::LoadFile(std::string(ASV_CONFIG_DIR) + "/maneuvers/" + name + ".yaml")
    ["maneuver_player"]["ros__parameters"];
  return {Maneuver::from_columns(
      p["times"].as<std::vector<double>>(), p["throttle"].as<std::vector<double>>(),
      p["steer"].as<std::vector<double>>(), p["spin"].as<std::vector<double>>()),
    p["start_delay"].as<double>()};
}

class ManeuverFileTest : public ::testing::TestWithParam<std::string> {};

TEST_P(ManeuverFileTest, StaysClearOfBuoysOnTheModel)
{
  const auto file = load_maneuver(GetParam());
  ManeuverPlayer player(file.maneuver, file.start_delay, params().limits);
  asv_sim_test::BoatModel boat(params());
  GateMonitor monitor({}, task1_buoys(), {params().hull_length, params().half_width}, 0.0, 0.0);

  // The clock starts at one control step (0 = no /clock yet).
  const double t_end = kControlDt + file.start_delay + file.maneuver.end_time() + kCoastTime;
  // Boat corner to zone >= centre-to-zone minus half the diagonal (conservative).
  const double half_diagonal = std::hypot(params().hull_length / 2.0, params().half_width);
  double zone_clearance = 1e9;
  std::string zone_name;
  double x_min = 0.0, x_max = 0.0, y_min = 0.0, y_max = 0.0, heading_change = 0.0;
  double last_psi = 0.0;
  bool done = false;
  const double h = kControlDt / kSubSteps;
  for (double t = kControlDt; t < t_end; t += kControlDt) {
    const auto out = player.step(t);
    done = done || out.state == PlayerState::DONE;
    for (int i = 0; i < kSubSteps; ++i) {
      boat.step(out.command, h);
    }
    const auto & s = boat.state();
    monitor.update({s.x, s.y}, s.psi);
    for (const auto & z : kZones) {
      const double c = zone_distance(z, s.x, s.y) - half_diagonal;
      if (c < zone_clearance) {
        zone_clearance = c;
        zone_name = z.name;
      }
    }
    x_min = std::min(x_min, s.x);
    x_max = std::max(x_max, s.x);
    y_min = std::min(y_min, s.y);
    y_max = std::max(y_max, s.y);
    heading_change += std::abs(s.psi - last_psi);
    last_psi = s.psi;
  }
  const auto & s = boat.state();
  const auto closest = monitor.closest();
  std::printf(
    "[ %s ] end (%.2f, %.2f) m, psi %.3f rad; x [%.2f, %.2f], y [%.2f, %.2f] m; "
    "total |heading change| %.2f rad; closest %s %.3f m, zone %s %.2f m\n",
    GetParam().c_str(), s.x, s.y, s.psi, x_min, x_max, y_min, y_max, heading_change,
    closest.name.c_str(), closest.min_clearance, zone_name.c_str(), zone_clearance);

  EXPECT_TRUE(done);
  EXPECT_GE(closest.min_clearance, kMinClearance) << closest.name;
  EXPECT_GE(zone_clearance, kMinClearance) << zone_name;
}

INSTANTIATE_TEST_SUITE_P(
  Files, ManeuverFileTest, ::testing::Values("straight", "circle", "zigzag"),
  [](const ::testing::TestParamInfo<std::string> & info) {return info.param;});

}  // namespace
