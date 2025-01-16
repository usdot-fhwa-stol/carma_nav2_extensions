cmake_minimum_required(VERSION 3.22)
project(nav2_port_drayage_demo)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include/
)

ament_auto_add_library(nav2_port_drayage_demo
  src/port_drayage_demo.cpp
)

ament_auto_add_executable(nav2_port_drayage_demo_node
  src/port_drayage_demo_node.cpp
)

target_link_libraries(nav2_port_drayage_demo_node
  nav2_port_drayage_demo
)

# Testing
if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable#

  ament_add_gtest(test_port_drayage_demo test/test_port_drayage_demo.cpp)

  target_link_libraries(test_port_drayage_demo nav2_port_drayage_demo)

endif()

ament_auto_package()
