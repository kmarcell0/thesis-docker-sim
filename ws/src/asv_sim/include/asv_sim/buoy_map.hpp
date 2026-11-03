// True buoy map (M2.7, D7 a, D19 a, D20 a): reads config/buoys_gt.yaml, which
// tools/make_buoy_map.py generates from the course files. ROS-free, so it is
// unit tested; the node buoy_map_publisher turns it into an
// asv_msgs/LandmarkArray.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace asv_sim
{

// One object of the course, fixed in place (static, D7 a).
struct TrueBuoy
{
  std::uint32_t id;    // index in buoys_gt.yaml
  std::string name;    // "<task model>::<object>", e.g. task1_entry_exit::gate1_red_port
  std::string cls;     // Sur-Mark, A-0, A-2, indicator, other (= vessel, D19 a)
  std::string colour;  // red, green, yellow, black
  double radius;       // [m] largest horizontal radius (D20 a)
  double x;            // [m] east, in map (= world)
  double y;            // [m] north, in map
};

struct BuoyMap
{
  std::string frame_id;
  std::vector<TrueBuoy> buoys;
};

// Parses and checks the file contents: ids are 0, 1, 2, ... in order; class
// and colour are known; radius > 0; positions finite. Throws
// std::runtime_error naming the bad entry.
BuoyMap parse_buoy_map(const YAML::Node & root);

// asv_msgs/BuoyColour value of a colour name (RED 1, GREEN 2, YELLOW 3,
// BLACK 4); throws std::runtime_error for anything else. The node checks
// these numbers against the message constants at compile time.
std::uint8_t colour_code(const std::string & colour);

}  // namespace asv_sim
