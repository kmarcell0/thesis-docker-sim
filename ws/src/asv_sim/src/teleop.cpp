#include "asv_sim/teleop.hpp"

#include <algorithm>
#include <cctype>

namespace asv_sim
{

namespace
{
constexpr char kCtrlC = 3;  // what Ctrl-C sends in raw mode (ISIG off)
}  // namespace

TeleopUpdate apply_key(
  const TeleopLevels & levels, char key, const TeleopSteps & steps,
  const ThrusterLimits & limits)
{
  TeleopUpdate out{levels, false};
  TeleopLevels & l = out.levels;
  switch (std::tolower(static_cast<unsigned char>(key))) {
    case 'w': l.throttle += steps.throttle; break;
    case 's': l.throttle -= steps.throttle; break;
    case 'a': l.steer += steps.steer; break;   // steer > 0 = left
    case 'd': l.steer -= steps.steer; break;
    case 'q': l.spin += steps.spin; break;     // spin > 0 = left
    case 'e': l.spin -= steps.spin; break;
    case ' ': l = TeleopLevels{}; break;
    case 'x':
    case kCtrlC:
      l = TeleopLevels{};
      out.quit = true;
      break;
    default: break;
  }
  // Reverse throttle stops where one thruster reaches min_thrust on its own.
  l.throttle = std::clamp(l.throttle, limits.min_thrust / limits.max_thrust, 1.0);
  l.steer = std::clamp(l.steer, -limits.steer_limit, limits.steer_limit);
  l.spin = std::clamp(l.spin, -1.0, 1.0);
  return out;
}

}  // namespace asv_sim
