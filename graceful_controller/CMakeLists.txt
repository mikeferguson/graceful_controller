cmake_minimum_required(VERSION 3.5)
project(graceful_controller)

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

include_directories(
  include
)

add_library(graceful_controller SHARED
  src/graceful_controller.cpp
)
target_link_libraries(graceful_controller
  ${catkin_LIBRARIES}
)
ament_target_dependencies(graceful_controller
  angles
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS graceful_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/graceful_controller
)

ament_export_include_directories(include)
ament_export_libraries(graceful_controller)
ament_package()
