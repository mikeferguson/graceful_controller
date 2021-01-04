cmake_minimum_required(VERSION 3.0.2)
project(graceful_controller_ros)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    base_local_planner
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    graceful_controller
    nav_msgs
    pluginlib
    roscpp
    tf2_geometry_msgs
    tf2_ros
)

generate_dynamic_reconfigure_options(
    cfg/GracefulController.cfg
)

catkin_package(
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    graceful_controller
    nav_msgs
    pluginlib
    roscpp
    tf2_geometry_msgs
    tf2_ros
  LIBRARIES
    graceful_controller_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(graceful_controller_ros
  src/graceful_controller_ros.cpp
)
target_link_libraries(graceful_controller_ros
  ${catkin_LIBRARIES}
)
add_dependencies(graceful_controller_ros
  ${catkin_EXPORTED_TARGETS}
)

if (CATKIN_ENABLE_TESTING)
  # TODO
endif()

install(TARGETS graceful_controller_ros
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
