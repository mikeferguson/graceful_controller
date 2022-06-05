cmake_minimum_required(VERSION 3.5)
project(graceful_controller_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(graceful_controller REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_library(graceful_controller_ros SHARED
  src/graceful_controller_ros.cpp
  src/orientation_tools.cpp
  src/visualization.cpp
)
ament_target_dependencies(graceful_controller_ros
  angles
  geometry_msgs
  graceful_controller
  nav_2d_utils
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

if(CATKIN_ENABLE_TESTING)
  #find_package(ament_cmake_gtest REQUIRED)
  #find_package(rostest)

  #add_executable(graceful_controller_tests
  #  test/graceful_controller_tests.cpp
  #)
  #target_link_libraries(graceful_controller_tests
  #  ${catkin_LIBRARIES}
  #  ${GTEST_LIBRARIES}
  #)
  #add_dependencies(tests graceful_controller_tests)
  #add_rostest(test/graceful_controller.test)

  #catkin_add_gtest(orientation_filter_tests
  #  src/orientation_tools.cpp
  #  test/orientation_tools_tests.cpp
  #)
  #target_link_libraries(orientation_filter_tests
  #  ${catkin_LIBRARIES}
  #)
endif()

install(TARGETS graceful_controller_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/graceful_controller_ros
)

ament_export_include_directories(include)
ament_export_libraries(graceful_controller_ros)
ament_export_dependencies(
  geometry_msgs
  graceful_controller
  nav_2d_utils
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)
ament_package()
