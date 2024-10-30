cmake_minimum_required(VERSION 3.22)
project(carma_nav2_emergency_stop)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(carma_nav2_emergency_stop
  src/emergency_stop.cpp
)

ament_auto_add_executable(carma_nav2_emergency_stop_node
  src/emergency_stop_node.cpp
)

ament_auto_package()
