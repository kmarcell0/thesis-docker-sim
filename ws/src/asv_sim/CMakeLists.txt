cmake_minimum_required(VERSION 3.8)
project(asv_sim)

find_package(ament_cmake REQUIRED)

# No C++ targets yet — this package only installs data files and launch.
# C++ nodes will be added in a later step when guidance logic is needed.

install(DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
