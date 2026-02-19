cmake_minimum_required(VERSION 3.22)
project(cr20_moveit)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install scripts
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

# Make scripts executable
install(PROGRAMS 
  scripts/cr20_isaac_bridge_node.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Install setup assistant config
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
