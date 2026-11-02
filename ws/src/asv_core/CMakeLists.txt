cmake_minimum_required(VERSION 3.8)
project(asv_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Plain C++, no ROS (D13 a): used by the nodes in asv_sim and later asv_slam.
add_library(asv_core
  src/angles.cpp
  src/geodesy.cpp
  src/gnss_corruptor.cpp
  src/imu_corruptor.cpp
  src/noise.cpp
)
target_include_directories(asv_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(asv_core PUBLIC Eigen3::Eigen)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS asv_core
  EXPORT export_asv_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Other packages link with asv_core::asv_core.
ament_export_targets(export_asv_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_angles test/test_angles.cpp)
  target_link_libraries(test_angles asv_core)

  # Geodesy (M2.1): also reads the C++/Python shared points in test/data.
  ament_add_gtest(test_geodesy test/test_geodesy.cpp)
  target_link_libraries(test_geodesy asv_core)
  target_compile_definitions(test_geodesy PRIVATE
    ASV_CORE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

  # Noise library (M2.5): statistics on 10-40 h records, so a longer timeout.
  ament_add_gtest(test_noise test/test_noise.cpp TIMEOUT 300)
  target_link_libraries(test_noise asv_core)

  # IMU corruptor (M2.6a): 1 h record and 2000-seed spreads.
  ament_add_gtest(test_imu_corruptor test/test_imu_corruptor.cpp TIMEOUT 120)
  target_link_libraries(test_imu_corruptor asv_core)

  # GNSS position corruptor (M2.6b): 2000-seed spreads.
  ament_add_gtest(test_gnss_corruptor test/test_gnss_corruptor.cpp TIMEOUT 120)
  target_link_libraries(test_gnss_corruptor asv_core)
endif()

ament_package()
