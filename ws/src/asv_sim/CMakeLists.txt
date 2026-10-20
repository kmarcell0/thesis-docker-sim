cmake_minimum_required(VERSION 3.8)
project(asv_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Milestone 1 guidance and control logic: plain C++, no ROS (unit tested).
add_library(asv_sim_guidance
  src/angles.cpp
  src/control.cpp
  src/follower.cpp
  src/gate_monitor.cpp
  src/guidance.cpp
  src/mix.cpp
)
target_include_directories(asv_sim_guidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Nodes: ROS I/O only, the logic is in asv_sim_guidance.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
add_executable(waypoint_follower src/waypoint_follower.cpp)
target_link_libraries(waypoint_follower
  asv_sim_guidance
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)
install(TARGETS waypoint_follower DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS asv_sim_guidance
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mix test/test_mix.cpp)
  target_link_libraries(test_mix asv_sim_guidance)
  ament_add_gtest(test_angles test/test_angles.cpp)
  target_link_libraries(test_angles asv_sim_guidance)
  ament_add_gtest(test_guidance test/test_guidance.cpp)
  target_link_libraries(test_guidance asv_sim_guidance)
  ament_add_gtest(test_control test/test_control.cpp)
  target_link_libraries(test_control asv_sim_guidance)
  ament_add_gtest(test_gate_monitor test/test_gate_monitor.cpp)
  target_link_libraries(test_gate_monitor asv_sim_guidance)
  ament_add_gtest(test_follower test/test_follower.cpp)
  target_link_libraries(test_follower asv_sim_guidance)

  # Closed-loop logic test (M1.2e): reads the boat values from config/asv.yaml
  # and config/asv_report.json in the source tree.
  find_package(yaml_cpp_vendor REQUIRED)
  find_package(yaml-cpp REQUIRED)
  ament_add_gtest(test_closed_loop test/test_closed_loop.cpp)
  target_link_libraries(test_closed_loop asv_sim_guidance yaml-cpp::yaml-cpp)
  target_compile_definitions(test_closed_loop PRIVATE
    ASV_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
endif()

ament_package()
