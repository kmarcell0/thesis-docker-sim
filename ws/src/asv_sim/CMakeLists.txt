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
find_package(asv_core REQUIRED)

# Milestone 1 guidance and control logic: plain C++, no ROS (unit tested).
add_library(asv_sim_guidance
  src/control.cpp
  src/follower.cpp
  src/gate_monitor.cpp
  src/guidance.cpp
  src/maneuver.cpp
  src/mix.cpp
  src/teleop.cpp
)
target_include_directories(asv_sim_guidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# wrap_angle() and yaw_from_quaternion() come from asv_core (M2.1).
target_link_libraries(asv_sim_guidance PUBLIC asv_core::asv_core)

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
add_executable(maneuver_player src/maneuver_player.cpp)
target_link_libraries(maneuver_player
  asv_sim_guidance
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)
# Keyboard teleop (M1.8): reads asv.yaml and teleop.yaml itself (T1 a).
find_package(ament_index_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
add_executable(teleop src/teleop_node.cpp)
target_link_libraries(teleop
  asv_sim_guidance
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  yaml-cpp::yaml-cpp
  ${std_msgs_TARGETS}
)
# Corruptor (M2.6a, IMU): the message glue is its own library (ROS messages,
# no node), so it is unit tested; the maths is in asv_core.
find_package(sensor_msgs REQUIRED)
add_library(asv_sim_corruptor src/imu_corruption.cpp)
target_include_directories(asv_sim_corruptor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(asv_sim_corruptor PUBLIC
  asv_core::asv_core
  yaml-cpp::yaml-cpp
  ${sensor_msgs_TARGETS}
)
add_executable(corruptor src/corruptor_node.cpp)
target_link_libraries(corruptor
  asv_sim_corruptor
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
)
install(TARGETS waypoint_follower maneuver_player teleop corruptor DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS asv_sim_guidance asv_sim_corruptor
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
  ament_add_gtest(test_guidance test/test_guidance.cpp)
  target_link_libraries(test_guidance asv_sim_guidance)
  ament_add_gtest(test_control test/test_control.cpp)
  target_link_libraries(test_control asv_sim_guidance)
  ament_add_gtest(test_gate_monitor test/test_gate_monitor.cpp)
  target_link_libraries(test_gate_monitor asv_sim_guidance)
  ament_add_gtest(test_follower test/test_follower.cpp)
  target_link_libraries(test_follower asv_sim_guidance)
  ament_add_gtest(test_maneuver test/test_maneuver.cpp)
  target_link_libraries(test_maneuver asv_sim_guidance)
  ament_add_gtest(test_teleop test/test_teleop.cpp)
  target_link_libraries(test_teleop asv_sim_guidance)

  # Closed-loop logic test (M1.2e): reads the boat values from config/asv.yaml
  # and config/asv_report.json in the source tree.
  ament_add_gtest(test_closed_loop test/test_closed_loop.cpp)
  target_link_libraries(test_closed_loop asv_sim_guidance yaml-cpp::yaml-cpp)
  target_compile_definitions(test_closed_loop PRIVATE
    ASV_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  # Maneuver files on the 3-DOF model (M1.7a): config/maneuvers/*.yaml, and
  # the Task 1 buoys from config/task1_route.yaml.
  ament_add_gtest(test_maneuver_files test/test_maneuver_files.cpp)
  target_link_libraries(test_maneuver_files asv_sim_guidance yaml-cpp::yaml-cpp)
  target_compile_definitions(test_maneuver_files PRIVATE
    ASV_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  # IMU corruption glue (M2.6a): reads config/asv.yaml and noise_profiles/.
  ament_add_gtest(test_imu_corruption test/test_imu_corruption.cpp)
  target_link_libraries(test_imu_corruption asv_sim_corruptor)
  target_compile_definitions(test_imu_corruption PRIVATE
    ASV_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
endif()

ament_package()
