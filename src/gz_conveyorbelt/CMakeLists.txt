cmake_minimum_required(VERSION 3.16)
project(gz_conveyorbelt)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find Gazebo packages via ROS2 vendor packages
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-transport13 REQUIRED)

# Conveyor Belt Plugin for Gazebo Sim 8 (Harmonic)
add_library(gz_conveyorbelt SHARED
  src/ConveyorBeltPlugin.cpp
)

target_include_directories(gz_conveyorbelt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against Gazebo libraries
target_link_libraries(gz_conveyorbelt
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
)

ament_target_dependencies(gz_conveyorbelt
  rclcpp
  std_msgs
)

# Install library to Gazebo plugin path
install(TARGETS gz_conveyorbelt
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install plugin description
install(FILES plugins/ConveyorBeltPlugin.sdf
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python scripts
install(PROGRAMS
  scripts/conveyor_controller.py
  scripts/run_conveyor_demo.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
