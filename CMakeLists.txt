cmake_minimum_required(VERSION 3.5)
project(aruco_tracker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Library
add_subdirectory(lib/KFilter)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)

add_executable(aruco_tracker
  src/aruco_tracker.cpp
  src/main.cpp
)

target_include_directories(aruco_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  lib
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(aruco_tracker
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  aruco_msgs
)

target_link_libraries(aruco_tracker KFilter)

install(TARGETS aruco_tracker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
