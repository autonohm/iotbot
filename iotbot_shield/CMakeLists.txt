cmake_minimum_required(VERSION 3.5)
project(iotbot_shield)

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
# find_package(iotbot_msgs)

# add the mraa lib directory to the workspace to build -> has to be cloned first from https://github.com/eclipse/mraa
#add_subdirectory(mraa-lib)
#include_directories(mraa-lib/api)

add_executable(iotbot_shield_node src/SerialPort.cpp) # src/iotbot_shield_node.cpp)

ament_target_dependencies(iotbot_shield_node rclcpp std_msgs geometry_msgs) # iotbot_msgs)

#target_link_libraries(iotbot_shield_node mraa) # additionaly link with mraa library

target_include_directories(iotbot_shield_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  iotbot_shield_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
