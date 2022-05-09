cmake_minimum_required(VERSION 3.0.2)
project(graceful_controller_ros)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    angles
    base_local_planner
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    graceful_controller
    nav_msgs
    pluginlib
    roscpp
    std_msgs
    tf2_geometry_msgs
    tf2_ros
)

generate_dynamic_reconfigure_options(
    cfg/GracefulController.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    graceful_controller
    nav_msgs
    pluginlib
    roscpp
    std_msgs
    tf2_geometry_msgs
    tf2_ros
  LIBRARIES
    graceful_controller_ros
)

if (CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(graceful_controller_ros
  src/graceful_controller_ros.cpp
  src/orientation_tools.cpp
  src/visualization.cpp
)
target_link_libraries(graceful_controller_ros
  ${catkin_LIBRARIES}
)
add_dependencies(graceful_controller_ros
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)

  add_executable(graceful_controller_tests
    test/graceful_controller_tests.cpp
  )
  target_link_libraries(graceful_controller_tests
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
  )
  add_dependencies(tests graceful_controller_tests)
  add_rostest(test/graceful_controller.test)

  catkin_add_gtest(orientation_filter_tests
    src/orientation_tools.cpp
    test/orientation_tools_tests.cpp
  )
  target_link_libraries(orientation_filter_tests
    ${catkin_LIBRARIES}
  )

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDS tests
    )
  endif()
endif()

install(TARGETS graceful_controller_ros
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
