cmake_minimum_required(VERSION 3.5)
project(iotbot_motion_control)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(iotbot_interface REQUIRED)

set(NODE_NAME_OMNI iotbot_motion_omnidirectional_node)
set(NODE_NAME_DIFF iotbot_motion_differential_node)

add_executable(${NODE_NAME_OMNI}
  src/iotbot_motion_control.cpp
  src/iotbot_motion_omnidirectional_node.cpp)

add_executable(${NODE_NAME_DIFF}
  src/iotbot_motion_control.cpp
  src/iotbot_motion_differential_node.cpp)

ament_target_dependencies(${NODE_NAME_OMNI}
  rclcpp
  std_msgs
  geometry_msgs
  iotbot_interface)

ament_target_dependencies(${NODE_NAME_DIFF}
  rclcpp
  std_msgs
  geometry_msgs
  iotbot_interface)

target_include_directories(${NODE_NAME_OMNI} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)

target_include_directories(${NODE_NAME_DIFF} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${NODE_NAME_OMNI}
  ${NODE_NAME_DIFF}
  
DESTINATION lib/${PROJECT_NAME})

ament_package()
