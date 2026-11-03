// Tests of the true buoy map loader (M2.7): the generated config/buoys_gt.yaml
// in the source tree, and the checks on bad entries.
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "asv_sim/buoy_map.hpp"

namespace
{

asv_sim::BuoyMap load_repo_map()
{
  return asv_sim::parse_buoy_map(YAML::LoadFile(std::string(ASV_CONFIG_DIR) + "/buoys_gt.yaml"));
}

// One valid entry; the tests below break one field at a time.
YAML::Node one_entry()
{
  return YAML::Load(
    "frame_id: map\n"
    "buoys:\n"
    "- {id: 0, name: 't::b', class: A-2, colour: yellow, radius: 0.184, x: 1.0, y: 2.0}\n");
}

}  // namespace

TEST(BuoyMap, RepoFileTask1Gate1Red)
{
  const auto map = load_repo_map();
  EXPECT_EQ(map.frame_id, "map");
  const auto & b = map.buoys.at(0);
  EXPECT_EQ(b.name, "task1_entry_exit::gate1_red_port");
  EXPECT_EQ(b.cls, "Sur-Mark");
  EXPECT_EQ(b.colour, "red");
  EXPECT_NEAR(b.x, 5.0, 1e-9);
  EXPECT_NEAR(b.y, 1.524, 1e-9);
  EXPECT_NEAR(b.radius, 0.2286, 1e-9);
}

TEST(BuoyMap, RepoFileCountsD19a)
{
  const auto map = load_repo_map();
  ASSERT_EQ(map.buoys.size(), 32u);
  int other = 0;
  for (const auto & b : map.buoys) {
    other += b.cls == "other";
  }
  EXPECT_EQ(other, 5);  // the vessels
}

TEST(BuoyMap, ValidEntryParses)
{
  const auto map = asv_sim::parse_buoy_map(one_entry());
  ASSERT_EQ(map.buoys.size(), 1u);
  EXPECT_EQ(asv_sim::colour_code(map.buoys[0].colour), 3);
}

TEST(BuoyMap, ColourCodes)
{
  EXPECT_EQ(asv_sim::colour_code("red"), 1);
  EXPECT_EQ(asv_sim::colour_code("green"), 2);
  EXPECT_EQ(asv_sim::colour_code("yellow"), 3);
  EXPECT_EQ(asv_sim::colour_code("black"), 4);
  EXPECT_THROW(asv_sim::colour_code("purple"), std::runtime_error);
}

TEST(BuoyMap, BadEntriesThrow)
{
  auto bad = [](const char * key, const char * value) {
      YAML::Node root = one_entry();
      root["buoys"][0][key] = value;
      return root;
    };
  EXPECT_THROW(asv_sim::parse_buoy_map(bad("id", "1")), std::runtime_error);
  EXPECT_THROW(asv_sim::parse_buoy_map(bad("class", "A-9")), std::runtime_error);
  EXPECT_THROW(asv_sim::parse_buoy_map(bad("colour", "purple")), std::runtime_error);
  EXPECT_THROW(asv_sim::parse_buoy_map(bad("radius", "0")), std::runtime_error);
  EXPECT_THROW(asv_sim::parse_buoy_map(bad("x", ".nan")), std::runtime_error);
}

TEST(BuoyMap, EmptyListThrows)
{
  EXPECT_THROW(
    asv_sim::parse_buoy_map(YAML::Load("frame_id: map\nbuoys: []\n")), std::runtime_error);
}
