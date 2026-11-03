#include "asv_sim/buoy_map.hpp"

#include <cmath>
#include <set>
#include <stdexcept>

namespace asv_sim
{

std::uint8_t colour_code(const std::string & colour)
{
  if (colour == "red") {return 1;}
  if (colour == "green") {return 2;}
  if (colour == "yellow") {return 3;}
  if (colour == "black") {return 4;}
  throw std::runtime_error("unknown buoy colour '" + colour + "'");
}

BuoyMap parse_buoy_map(const YAML::Node & root)
{
  static const std::set<std::string> classes{"Sur-Mark", "A-0", "A-2", "indicator", "other"};
  BuoyMap map;
  map.frame_id = root["frame_id"].as<std::string>();
  const YAML::Node list = root["buoys"];
  if (!list || !list.IsSequence() || list.size() == 0) {
    throw std::runtime_error("buoy map: 'buoys' is missing or empty");
  }
  for (std::size_t k = 0; k < list.size(); ++k) {
    const YAML::Node & n = list[k];
    TrueBuoy b{
      n["id"].as<std::uint32_t>(), n["name"].as<std::string>(),
      n["class"].as<std::string>(), n["colour"].as<std::string>(),
      n["radius"].as<double>(), n["x"].as<double>(), n["y"].as<double>()};
    const std::string where = "buoy map entry " + std::to_string(k) + " (" + b.name + "): ";
    if (b.id != k) {
      throw std::runtime_error(where + "id must equal its index");
    }
    if (classes.count(b.cls) == 0) {
      throw std::runtime_error(where + "unknown class '" + b.cls + "'");
    }
    colour_code(b.colour);  // throws for an unknown colour
    if (!(b.radius > 0.0) || !std::isfinite(b.x) || !std::isfinite(b.y)) {
      throw std::runtime_error(where + "radius must be > 0 and x, y finite");
    }
    map.buoys.push_back(b);
  }
  return map;
}

}  // namespace asv_sim
