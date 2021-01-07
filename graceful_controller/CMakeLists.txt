cmake_minimum_required(VERSION 3.0.2)
project(graceful_controller)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    angles
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    graceful_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(graceful_controller
  src/graceful_controller.cpp
)
target_link_libraries(graceful_controller
  ${catkin_LIBRARIES}
)
add_dependencies(graceful_controller
  ${catkin_EXPORTED_TARGETS}
)

if (CATKIN_ENABLE_TESTING)
  # TODO
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS graceful_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
