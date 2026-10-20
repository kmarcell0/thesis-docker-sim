// teleop: drive the boat from the keyboard (M1.8). Only I/O: puts the
// terminal in raw mode, reads keys, calls asv_sim::apply_key() (teleop.hpp)
// and publishes mix() of the levels on a 20 Hz ROS-clock timer.
// Start it only in your own terminal (a launched node gets no keyboard):
//   ros2 run asv_sim teleop --ros-args -p use_sim_time:=true
// Never run it together with the waypoint follower or the maneuver player
// (all three publish the thruster commands).
//
// The thruster limits come from config/asv.yaml and the key steps from
// config/teleop.yaml, both read here from the package share directory
// (decision T1 a, 2026-10-20): ros2 run gets no parameters from a launch file.
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "yaml-cpp/yaml.h"

#include "asv_sim/mix.hpp"
#include "asv_sim/teleop.hpp"

namespace
{
using Float64 = std_msgs::msg::Float64;

// Set by SIGTERM, SIGHUP or SIGINT; the main loop then publishes zero and exits.
std::atomic<bool> g_signal_stop{false};

void on_signal(int)
{
  g_signal_stop = true;
}

// Raw mode for the terminal on stdin while this object lives: no line
// buffering (keys arrive without Enter), no echo, and no ISIG, so Ctrl-C is
// read as byte 3 instead of killing the node before it publishes zero.
// VMIN = VTIME = 0: read() returns at once, also when no key is waiting.
class RawTerminal
{
public:
  RawTerminal()
  {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0) {
      throw std::runtime_error(
              "teleop needs a terminal: run it yourself in "
              "'docker compose exec sim bash', not from a launch file");
    }
    termios raw = saved_;
    raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO | ISIG);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }
  ~RawTerminal() {tcsetattr(STDIN_FILENO, TCSANOW, &saved_);}
  RawTerminal(const RawTerminal &) = delete;
  RawTerminal & operator=(const RawTerminal &) = delete;

  // One waiting key, or false if none.
  bool read_key(char & key) {return read(STDIN_FILENO, &key, 1) == 1;}

private:
  termios saved_{};
};

}  // namespace

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode()
  : Node("teleop")
  {
    const std::string config =
      ament_index_cpp::get_package_share_directory("asv_sim") + "/config";
    // A missing key throws: no defaults, the node must not run on a guess.
    const YAML::Node thr = YAML::LoadFile(config + "/asv.yaml")["thrusters"];
    limits_ = {thr["max_thrust"].as<double>(), thr["min_thrust"].as<double>(),
      thr["steer_limit"].as<double>()};
    const YAML::Node tel = YAML::LoadFile(config + "/teleop.yaml");
    const YAML::Node st = tel["steps"];
    steps_ = {st["throttle"].as<double>(), st["steer"].as<double>(), st["spin"].as<double>()};
    const double rate = tel["control_rate"].as<double>();
    if (!(rate > 0.0)) {
      throw std::invalid_argument("control_rate must be > 0");
    }

    left_thrust_ = create_publisher<Float64>("/asv/thrusters/left/thrust", 10);
    right_thrust_ = create_publisher<Float64>("/asv/thrusters/right/thrust", 10);
    left_pos_ = create_publisher<Float64>("/asv/thrusters/left/pos", 10);
    right_pos_ = create_publisher<Float64>("/asv/thrusters/right/pos", 10);

    // ROS clock (design.md section 3). The keys are read in the main loop,
    // so quitting also works while no /clock arrives.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
      [this]() {publish(asv_sim::mix(levels_.throttle, levels_.steer, levels_.spin, limits_));});

    std::printf(
      "W/S throttle, A/D steer, Q/E spin, Space all zero, X or Ctrl-C quit.\n"
      "Levels stay set after the key is released.\n");
    print_status();
  }

  // Applies one key; returns true when the key asks to quit.
  bool handle_key(char key)
  {
    const auto u = asv_sim::apply_key(levels_, key, steps_, limits_);
    levels_ = u.levels;
    print_status();
    return u.quit;
  }

  // Zero on all four topics, right now (not on the timer).
  void publish_zero() {publish(asv_sim::mix(0.0, 0.0, 0.0, limits_));}

private:
  void publish(const asv_sim::ThrusterCommand & c)
  {
    send(left_thrust_, c.left_thrust);
    send(right_thrust_, c.right_thrust);
    send(left_pos_, c.left_pos);
    send(right_pos_, c.right_pos);
  }

  static void send(const rclcpp::Publisher<Float64>::SharedPtr & pub, double value)
  {
    Float64 msg;
    msg.data = value;
    pub->publish(msg);
  }

  // One line, overwritten in place (\r), so the terminal does not scroll.
  void print_status() const
  {
    const auto c = asv_sim::mix(levels_.throttle, levels_.steer, levels_.spin, limits_);
    std::printf(
      "\rthrottle %+.2f  steer %+.2f rad  spin %+.2f  ->  L %+6.1f N  R %+6.1f N  pod %+.2f rad ",
      levels_.throttle, levels_.steer, levels_.spin, c.left_thrust, c.right_thrust, c.left_pos);
    std::fflush(stdout);
  }

  asv_sim::ThrusterLimits limits_{};
  asv_sim::TeleopSteps steps_{};
  asv_sim::TeleopLevels levels_;
  rclcpp::Publisher<Float64>::SharedPtr left_thrust_, right_thrust_, left_pos_, right_pos_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  // Our own signal handlers instead of rclcpp's: rclcpp would shut down the
  // context first, and then the zero command could no longer be published.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::signal(SIGHUP, on_signal);
  int code = 0;
  try {
    RawTerminal terminal;
    auto node = std::make_shared<TeleopNode>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    bool quit = false;
    while (rclcpp::ok() && !quit && !g_signal_stop) {
      char key;
      while (!quit && terminal.read_key(key)) {
        quit = node->handle_key(key);
      }
      exec.spin_once(std::chrono::milliseconds(10));
    }
    node->publish_zero();
    // Give DDS a moment to send it before the context goes away. Wall
    // clock on purpose: it must also pass while the simulation is paused.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::printf("\nzero thrust published, teleop stopped\n");
  } catch (const std::exception & e) {
    std::fprintf(stderr, "\nteleop: %s\n", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
