# Install script for directory: /home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/iudc/dobot_ws/install/dobot_msgs_v4")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rosidl_interfaces" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/dobot_msgs_v4")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/msg" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/msg/RobotStatus.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/msg" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/msg/ToolVectorActual.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/EnableRobot.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/DisableRobot.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/ClearError.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SpeedFactor.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/User.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/Tool.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/RobotMode.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetPayload.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/DO.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/DOInstant.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/ToolDO.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/ToolDOInstant.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/AO.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/AOInstant.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/AccJ.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/AccL.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/VelJ.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/VelL.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/CP.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/PowerOn.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/RunScript.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/Stop.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/Pause.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/Continue.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/PositiveKin.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/InverseKin.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetCollisionLevel.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetAngle.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetPose.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/EmergencyStop.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/ModbusRTUCreate.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/ModbusCreate.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/ModbusClose.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetInBits.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetInRegs.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetCoils.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetCoils.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetHoldRegs.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetHoldRegs.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetSafeSkin.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/MovJ.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/MovL.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/RelJointMovJ.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/MoveJog.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/StopMoveJog.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/DOGroup.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/BrakeControl.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/StartDrag.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/EnableSafeSkin.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetStartPose.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/StartPath.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/InverseSolution.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetErrorID.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/DI.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/ToolDI.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/AI.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/ToolAI.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/DIGroup.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/StopDrag.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/DragSensivity.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetDO.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetAO.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetDOGroup.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetTool485.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetSafeWallEnable.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetToolPower.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetToolMode.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetBackDistance.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetPostCollisionMode.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetUser.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetTool.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/CalcUser.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/CalcTool.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetInputBool.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetInputInt.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetInputFloat.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetOutputBool.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetOutputInt.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetOutputFloat.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetOutputBool.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetOutputInt.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetOutputFloat.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/MovLIO.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/MovJIO.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/Arc.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/Circle.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/RelMovJTool.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/RelMovLTool.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/RelMovJUser.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/RelMovLUser.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetCurrentCommandId.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/ServoJ.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/ServoP.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/TcpDashboard.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/EnableFTSensor.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SixForceHome.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetForce.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/ForceDriveMode.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/ForceDriveSpeed.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/FCForceMode.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/FCSetDeviation.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/FCSetForceLimit.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/FCSetMass.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/FCSetStiffness.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/FCSetDamping.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/FCOff.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/FCSetForceSpeedLimit.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/FCSetForce.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetFCCollision.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/FCCollisionSwitch.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/SetWorkZoneEnable.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetToolDO.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/ResetRobot.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/RunTo.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/StartRTOffset.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/EndRTOffset.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetError.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/DOGroupDEC.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/GetDOGroupDEC.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/DIGroupDEC.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_type_description/dobot_msgs_v4/srv/RequestControl.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dobot_msgs_v4/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_c/dobot_msgs_v4/" REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/opt/ros/jazzy/lib/python3.12/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_generator_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dobot_msgs_v4/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_typesupport_fastrtps_c/dobot_msgs_v4/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/home/iudc/dobot_ws/build/dobot_msgs_v4:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dobot_msgs_v4/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_cpp/dobot_msgs_v4/" REGEX "/[^/]*\\.hpp$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dobot_msgs_v4/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_typesupport_fastrtps_cpp/dobot_msgs_v4/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so"
         OLD_RPATH "/opt/ros/jazzy/lib:/home/iudc/dobot_ws/build/dobot_msgs_v4:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_fastrtps_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dobot_msgs_v4/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_typesupport_introspection_c/dobot_msgs_v4/" REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/iudc/dobot_ws/build/dobot_msgs_v4:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so"
         OLD_RPATH "/home/iudc/dobot_ws/build/dobot_msgs_v4:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dobot_msgs_v4/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_typesupport_introspection_cpp/dobot_msgs_v4/" REGEX "/[^/]*\\.hpp$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so"
         OLD_RPATH "/home/iudc/dobot_ws/build/dobot_msgs_v4:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_introspection_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_typesupport_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so"
         OLD_RPATH "/home/iudc/dobot_ws/build/dobot_msgs_v4:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_typesupport_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4-0.0.0-py3.12.egg-info" TYPE DIRECTORY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_python/dobot_msgs_v4/dobot_msgs_v4.egg-info/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4" TYPE DIRECTORY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_py/dobot_msgs_v4/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/iudc/dobot_ws/install/dobot_msgs_v4/lib/python3.12/site-packages/dobot_msgs_v4"
      )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/iudc/dobot_ws/build/dobot_msgs_v4/dobot_msgs_v4__py/cmake_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4" TYPE MODULE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_py/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/home/iudc/dobot_ws/build/dobot_msgs_v4:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/dobot_msgs_v4_s__rosidl_typesupport_fastrtps_c.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4" TYPE MODULE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_py/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/iudc/dobot_ws/build/dobot_msgs_v4:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/dobot_msgs_v4_s__rosidl_typesupport_introspection_c.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4" TYPE MODULE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_generator_py/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.so"
         OLD_RPATH "/home/iudc/dobot_ws/build/dobot_msgs_v4:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/dobot_msgs_v4/dobot_msgs_v4_s__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/dobot_msgs_v4_s__rosidl_typesupport_c.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/libdobot_msgs_v4__rosidl_generator_py.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so"
         OLD_RPATH "/home/iudc/dobot_ws/build/dobot_msgs_v4:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdobot_msgs_v4__rosidl_generator_py.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/msg" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/msg/RobotStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/msg" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/msg/ToolVectorActual.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/EnableRobot.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DisableRobot.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ClearError.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SpeedFactor.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/User.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/Tool.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RobotMode.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetPayload.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DO.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DOInstant.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ToolDO.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ToolDOInstant.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/AO.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/AOInstant.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/AccJ.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/AccL.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/VelJ.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/VelL.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/CP.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/PowerOn.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RunScript.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/Stop.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/Pause.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/Continue.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/PositiveKin.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/InverseKin.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetCollisionLevel.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetAngle.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetPose.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/EmergencyStop.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ModbusRTUCreate.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ModbusCreate.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ModbusClose.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetInBits.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetInRegs.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetCoils.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetCoils.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetHoldRegs.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetHoldRegs.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetSafeSkin.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/MovJ.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/MovL.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RelJointMovJ.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/MoveJog.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/StopMoveJog.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DOGroup.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/BrakeControl.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/StartDrag.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/EnableSafeSkin.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetStartPose.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/StartPath.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/InverseSolution.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetErrorID.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DI.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ToolDI.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/AI.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ToolAI.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DIGroup.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/StopDrag.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DragSensivity.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetDO.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetAO.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetDOGroup.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetTool485.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetSafeWallEnable.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetToolPower.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetToolMode.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetBackDistance.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetPostCollisionMode.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetUser.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetTool.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/CalcUser.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/CalcTool.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetInputBool.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetInputInt.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetInputFloat.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetOutputBool.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetOutputInt.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetOutputFloat.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetOutputBool.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetOutputInt.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetOutputFloat.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/MovLIO.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/MovJIO.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/Arc.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/Circle.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RelMovJTool.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RelMovLTool.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RelMovJUser.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RelMovLUser.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetCurrentCommandId.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ServoJ.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ServoP.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/TcpDashboard.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/EnableFTSensor.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SixForceHome.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetForce.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ForceDriveMode.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ForceDriveSpeed.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/FCForceMode.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/FCSetDeviation.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/FCSetForceLimit.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/FCSetMass.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/FCSetStiffness.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/FCSetDamping.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/FCOff.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/FCSetForceSpeedLimit.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/FCSetForce.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetFCCollision.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/FCCollisionSwitch.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/SetWorkZoneEnable.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetToolDO.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/ResetRobot.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RunTo.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/StartRTOffset.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/EndRTOffset.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetError.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DOGroupDEC.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/GetDOGroupDEC.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/DIGroupDEC.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_adapter/dobot_msgs_v4/srv/RequestControl.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/msg" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/msg/RobotStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/msg" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/msg/ToolVectorActual.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/EnableRobot.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DisableRobot.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ClearError.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SpeedFactor.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/User.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/Tool.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RobotMode.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetPayload.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DO.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DOInstant.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ToolDO.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ToolDOInstant.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/AO.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/AOInstant.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/AccJ.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/AccL.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/VelJ.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/VelL.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/CP.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/PowerOn.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RunScript.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/Stop.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/Pause.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/Continue.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/PositiveKin.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/InverseKin.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetCollisionLevel.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetAngle.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetPose.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/EmergencyStop.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ModbusRTUCreate.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ModbusCreate.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ModbusClose.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetInBits.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetInRegs.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetCoils.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetCoils.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetHoldRegs.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetHoldRegs.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetSafeSkin.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/MovJ.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/MovL.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RelJointMovJ.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/MoveJog.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/StopMoveJog.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DOGroup.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/BrakeControl.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/StartDrag.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/EnableSafeSkin.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetStartPose.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/StartPath.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/InverseSolution.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetErrorID.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DI.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ToolDI.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/AI.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ToolAI.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DIGroup.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/StopDrag.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DragSensivity.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetDO.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetAO.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetDOGroup.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetTool485.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetSafeWallEnable.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetToolPower.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetToolMode.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetBackDistance.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetPostCollisionMode.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetUser.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetTool.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/CalcUser.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/CalcTool.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetInputBool.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetInputInt.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetInputFloat.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetOutputBool.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetOutputInt.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetOutputFloat.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetOutputBool.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetOutputInt.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetOutputFloat.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/MovLIO.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/MovJIO.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/Arc.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/Circle.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RelMovJTool.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RelMovLTool.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RelMovJUser.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RelMovLUser.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetCurrentCommandId.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ServoJ.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ServoP.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/TcpDashboard.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/EnableFTSensor.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SixForceHome.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetForce.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ForceDriveMode.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ForceDriveSpeed.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/FCForceMode.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/FCSetDeviation.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/FCSetForceLimit.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/FCSetMass.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/FCSetStiffness.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/FCSetDamping.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/FCOff.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/FCSetForceSpeedLimit.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/FCSetForce.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetFCCollision.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/FCCollisionSwitch.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/SetWorkZoneEnable.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetToolDO.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/ResetRobot.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RunTo.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/StartRTOffset.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/EndRTOffset.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetError.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DOGroupDEC.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/GetDOGroupDEC.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/DIGroupDEC.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/srv" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/srv/RequestControl.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/dobot_msgs_v4")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/dobot_msgs_v4")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/environment" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_index/share/ament_index/resource_index/packages/dobot_msgs_v4")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cExport.cmake"
         "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_generator_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_generator_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_generator_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport.cmake"
         "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cppExport.cmake"
         "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_generator_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_generator_cppExport.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport.cmake"
         "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_typesupport_fastrtps_cppExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cExport.cmake"
         "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/dobot_msgs_v4__rosidl_typesupport_introspection_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/dobot_msgs_v4__rosidl_typesupport_introspection_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/dobot_msgs_v4__rosidl_typesupport_introspection_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cExport.cmake"
         "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/dobot_msgs_v4__rosidl_typesupport_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/dobot_msgs_v4__rosidl_typesupport_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/dobot_msgs_v4__rosidl_typesupport_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport.cmake"
         "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/dobot_msgs_v4__rosidl_typesupport_introspection_cppExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cppExport.cmake"
         "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/dobot_msgs_v4__rosidl_typesupport_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/dobot_msgs_v4__rosidl_typesupport_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/dobot_msgs_v4__rosidl_typesupport_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/dobot_msgs_v4__rosidl_typesupport_cppExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_pyExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_pyExport.cmake"
         "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_generator_pyExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_pyExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake/export_dobot_msgs_v4__rosidl_generator_pyExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_generator_pyExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/CMakeFiles/Export/e91720f1affd72b2fa9846ba57dd2b2e/export_dobot_msgs_v4__rosidl_generator_pyExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/rosidl_cmake-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES "/home/iudc/dobot_ws/build/dobot_msgs_v4/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4/cmake" TYPE FILE FILES
    "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_core/dobot_msgs_v4Config.cmake"
    "/home/iudc/dobot_ws/build/dobot_msgs_v4/ament_cmake_core/dobot_msgs_v4Config-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_msgs_v4" TYPE FILE FILES "/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/iudc/dobot_ws/build/dobot_msgs_v4/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
