// 3-DOF boat model (surge, sway, yaw) for the closed-loop logic test (M1.2e).
// Test-only: it stands in for Gazebo so the whole M1 control chain can be run
// without a simulator. Spec: milestone1_plan.md, M1.2e brief, decision 10.5.
// The numbers are Gazebo's own (asv.yaml, asv_report.json), so this model is
// for testing the controller only, never for the filter (inverse crime).
#ifndef ASV_SIM_TEST_BOAT_MODEL_HPP_
#define ASV_SIM_TEST_BOAT_MODEL_HPP_

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "asv_sim/mix.hpp"

namespace asv_sim_test
{

// Everything the model needs, read from the generator's input and report.
struct BoatParams
{
  double mass;           // m, kg, whole boat incl. pods (report mass.total_kg)
  double inertia_z;      // I_z, kg m^2, about the CoG (report mass.inertia_cog[2][2])
  double drag_u;         // X_u, N per m/s            (hydrodynamics xU)
  double drag_uu;        // X_uu, N per (m/s)^2       (xUU)
  double drag_v;         // Y_v, N per m/s            (yV)
  double drag_vv;        // Y_vv, N per (m/s)^2       (yVV)
  double drag_r;         // N_r, N m per rad/s        (nR)
  double drag_rr;        // N_rr, N m per (rad/s)^2   (nRR)
  double pod_x;          // x_p, m, pod steering axis along x (thrusters.x, < 0: aft)
  double pod_y;          // m, pod distance from the centre line (hull.spacing / 2)
  double steer_rate;     // rad/s, largest pod turning rate (thrusters.steer_rate)
  asv_sim::ThrusterLimits limits;  // max_thrust, min_thrust, steer_limit
  double hull_length;    // L, m (hull.length)
  double half_width;     // b_w, m (hull.spacing / 2 + hull.radius)
};

// config_dir: the package's config folder (asv.yaml, asv_report.json).
// yaml-cpp also reads the JSON report (JSON is valid YAML here).
inline BoatParams load_boat_params(const std::string & config_dir)
{
  const YAML::Node asv = YAML::LoadFile(config_dir + "/asv.yaml");
  const YAML::Node report = YAML::LoadFile(config_dir + "/asv_report.json");
  const YAML::Node hyd = asv["hydrodynamics"];
  const YAML::Node thr = asv["thrusters"];
  const YAML::Node hull = asv["hull"];
  BoatParams p{};
  p.mass = report["mass"]["total_kg"].as<double>();
  p.inertia_z = report["mass"]["inertia_cog"][2][2].as<double>();
  p.drag_u = hyd["xU"].as<double>();
  p.drag_uu = hyd["xUU"].as<double>();
  p.drag_v = hyd["yV"].as<double>();
  p.drag_vv = hyd["yVV"].as<double>();
  p.drag_r = hyd["nR"].as<double>();
  p.drag_rr = hyd["nRR"].as<double>();
  p.pod_x = thr["x"].as<double>();
  p.pod_y = hull["spacing"].as<double>() / 2.0;
  p.steer_rate = thr["steer_rate"].as<double>();
  p.limits = {thr["max_thrust"].as<double>(), thr["min_thrust"].as<double>(),
    thr["steer_limit"].as<double>()};
  p.hull_length = hull["length"].as<double>();
  p.half_width = p.pod_y + hull["radius"].as<double>();
  return p;
}

// Pose in the world frame (ENU), velocities in the body frame (x forward,
// y left), as in /asv/ground_truth/odom.
struct BoatState
{
  double x = 0.0;      // m
  double y = 0.0;      // m
  double psi = 0.0;    // heading psi, rad
  double u = 0.0;      // surge (forward) speed, m/s
  double v = 0.0;      // sway (sideways) speed, m/s, > 0: to the left
  double omega = 0.0;  // yaw rate, rad/s, > 0: counter-clockwise
  double pod = 0.0;    // actual pod angle delta, rad (both pods; > 0 turns RIGHT)
};

class BoatModel
{
public:
  explicit BoatModel(const BoatParams & params, const BoatState & start = {})
  : p_(params), s_(start) {}

  const BoatState & state() const {return s_;}

  // Advance by dt (s) with the command held (zero-order hold), explicit Euler.
  // The pod angle moves towards the commanded one at most steer_rate * dt.
  //   T = T_L + T_R,  X = T cos(delta),  Y = T sin(delta)
  //   N = x_p T sin(delta) + y_p (T_R - T_L) cos(delta)     (plan 10.2)
  //   m (u' - v omega) = X - (X_u + X_uu |u|) u
  //   m (v' + u omega) = Y - (Y_v + Y_vv |v|) v
  //   I_z omega'       = N - (N_r + N_rr |omega|) omega
  //   x' = u cos psi - v sin psi,  y' = u sin psi + v cos psi,  psi' = omega
  // Taken about base_link; the CoG is 1.8 cm aft of it [REPO report], ignored.
  void step(const asv_sim::ThrusterCommand & cmd, double dt)
  {
    const double max_move = p_.steer_rate * dt;
    s_.pod += std::clamp(cmd.left_pos - s_.pod, -max_move, max_move);

    const double total = cmd.left_thrust + cmd.right_thrust;
    const double fx = total * std::cos(s_.pod);
    const double fy = total * std::sin(s_.pod);
    const double nz = p_.pod_x * total * std::sin(s_.pod) +
      p_.pod_y * (cmd.right_thrust - cmd.left_thrust) * std::cos(s_.pod);

    const double du = s_.v * s_.omega +
      (fx - (p_.drag_u + p_.drag_uu * std::abs(s_.u)) * s_.u) / p_.mass;
    const double dv = -s_.u * s_.omega +
      (fy - (p_.drag_v + p_.drag_vv * std::abs(s_.v)) * s_.v) / p_.mass;
    const double dr =
      (nz - (p_.drag_r + p_.drag_rr * std::abs(s_.omega)) * s_.omega) / p_.inertia_z;

    const double c = std::cos(s_.psi);
    const double s = std::sin(s_.psi);
    s_.x += (s_.u * c - s_.v * s) * dt;
    s_.y += (s_.u * s + s_.v * c) * dt;
    s_.psi += s_.omega * dt;
    s_.u += du * dt;
    s_.v += dv * dt;
    s_.omega += dr * dt;
  }

private:
  BoatParams p_;
  BoatState s_;
};

}  // namespace asv_sim_test

#endif  // ASV_SIM_TEST_BOAT_MODEL_HPP_
