cmake_minimum_required(VERSION 3.8)
project(dobot_bringup_v4)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dobot_msgs_v4 REQUIRED)

add_executable(dobot_bringup_v4_node
  src/main.cpp
  src/cr_robot_ros2.cpp
  src/command.cpp
  src/tcp_socket.cpp
  src/parseTool.cpp
)

target_include_directories(dobot_bringup_v4_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dobot_bringup_v4_node
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  dobot_msgs_v4
)

target_link_libraries(dobot_bringup_v4_node pthread)

install(TARGETS dobot_bringup_v4_node
  DESTINATION lib/dobot_bringup_v4)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
