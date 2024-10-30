cmake_minimum_required(VERSION 3.22)
project(carma_nav2_behavior_tree)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  nav2_util
  nav2_behavior_tree
)

add_library(carma_nav2_compute_and_track_route_action_bt_node SHARED plugins/action/compute_and_track_route_action.cpp)
list(APPEND plugin_libs carma_nav2_compute_and_track_route_action_bt_node)

add_library(carma_nav2_compute_route_action_bt_node SHARED plugins/action/compute_route_action.cpp)
list(APPEND plugin_libs carma_nav2_compute_route_action_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable#

  ament_add_gtest(test_compute_route_action
  test/plugins/action/test_compute_route_action.cpp
  )
  ament_target_dependencies(test_compute_route_action ${dependencies})
  target_link_libraries(test_compute_route_action ${plugin_libs})

  ament_add_gtest(test_compute_and_track_route_action
    test/plugins/action/test_compute_and_track_route_action.cpp
  )
  ament_target_dependencies(test_compute_and_track_route_action ${dependencies})
  target_link_libraries(test_compute_and_track_route_action ${plugin_libs})

endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
