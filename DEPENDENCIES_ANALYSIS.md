# Análisis de Dependencias - Repositorio dobot_ws

## Resumen Ejecutivo

| Métrica | Valor |
|---------|-------|
| Total de paquetes | 11 |
| Paquetes con formato ROS2 | 0 |
| Paquetes mentioning Jazzy | 4 |
| Paquetes mencionando Humble | 0 |

---

## 1. Resumen de Paquetes por Categoría

### Bringup/Control: 1 paquete(s)
- [dobot_bringup_v4](/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/package.xml) (v0.0.0)

### Demo: 1 paquete(s)
- [dobot_demo](/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_demo/package.xml) (v0.0.0)

### Gazebo: 3 paquete(s)
- [dobot_gazebo](/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_gazebo/package.xml) (v0.0.0)
- [gz_conveyorbelt](/home/iudc/dobot_ws/src/gz_conveyorbelt/package.xml) (v1.0.0)
- [pruebas_de_vision](/home/iudc/dobot_ws/src/pruebas_de_vision/package.xml) (v0.0.1)

### Messages/Services: 1 paquete(s)
- [dobot_msgs_v4](/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/package.xml) (v0.0.0)

### MoveIt: 2 paquete(s)
- [cr20_moveit](/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cr20_moveit/package.xml) (v0.3.0)
- [dobot_moveit](/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_moveit/package.xml) (v0.0.0)

### RViz: 1 paquete(s)
- [dobot_rviz](/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_rviz/package.xml) (v0.0.0)

### Robot Description: 1 paquete(s)
- [cra_description](/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cra_description/package.xml) (v0.0.0)

### Servo: 1 paquete(s)
- [servo_action](/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/servo_action/package.xml) (v0.0.0)


## 2. Detalle de Paquetes: Bringup/Control

| Paquete | Versión | Formato | Build Type | ROS Version |
|---------|---------|---------|------------|-------------|
| dobot_bringup_v4 | 0.0.0 | 2 | ament_cmake | Not specified |

### Descripciones

**dobot_bringup_v4**: TODO: Package description


### Dependencias

**dobot_bringup_v4**: ament_cmake, rclcpp, rclcpp_action, std_msgs, dobot_msgs_v4, ament_lint_auto, ament_lint_common


## 2. Detalle de Paquetes: Demo

| Paquete | Versión | Formato | Build Type | ROS Version |
|---------|---------|---------|------------|-------------|
| dobot_demo | 0.0.0 | 2 | ament_cmake | Not specified |

### Descripciones

**dobot_demo**: TODO: Package description


### Dependencias

**dobot_demo**: ament_copyright, ament_flake8, ament_pep257, python3-pytest


## 2. Detalle de Paquetes: Gazebo

| Paquete | Versión | Formato | Build Type | ROS Version |
|---------|---------|---------|------------|-------------|
| dobot_gazebo | 0.0.0 | 2 | ament_cmake | Jazzy (inferred) |
| gz_conveyorbelt | 1.0.0 | 2 | ament_cmake | Jazzy (inferred) |
| pruebas_de_vision | 0.0.1 | 2 | ament_cmake | Jazzy (inferred) |

### Descripciones

**dobot_gazebo**: Launch files and worlds for Dobot simulation in Gz Harmonic

**gz_conveyorbelt**: Conveyor Belt Controller for Gazebo Sim 8 (Harmonic) - Scripted Pose Control

**pruebas_de_vision**: Paquete aislado para pruebas de visión con cámara en Gazebo y detección YOLOv8-OBB


### Dependencias

**dobot_gazebo**: ament_cmake, ros_gz_sim, ros_gz_bridge, gz_ros2_control, robot_state_publisher, cra_description, xacro, controller_manager, joint_state_broadcaster, joint_trajectory_controller

**gz_conveyorbelt**: ament_cmake_python, gz-sim8, gz-transport13, rclcpp, rclpy, std_msgs, std_srvs, geometry_msgs, example_interfaces, ros_gz_bridge, ament_lint_auto, ament_lint_common, ament_copyright, ament_flake8, ament_pep257, python3-pytest

**pruebas_de_vision**: ament_python, ros_gz_sim, ros_gz_bridge, gz_sim_vendor, python3-opencv, rclpy, sensor_msgs, cv_bridge, geometry_msgs, vision_msgs, tf_transformations


## 2. Detalle de Paquetes: Messages/Services

| Paquete | Versión | Formato | Build Type | ROS Version |
|---------|---------|---------|------------|-------------|
| dobot_msgs_v4 | 0.0.0 | 2 | ament_cmake | Not specified |

### Descripciones

**dobot_msgs_v4**: TODO: Package description


### Dependencias

**dobot_msgs_v4**: ament_cmake, rosidl_default_runtime, rclcpp, std_msgs, rosidl_default_generators, ament_lint_auto, ament_lint_common


## 2. Detalle de Paquetes: MoveIt

| Paquete | Versión | Formato | Build Type | ROS Version |
|---------|---------|---------|------------|-------------|
| cr20_moveit | 0.3.0 | 2 | ament_cmake | Jazzy (inferred) |
| dobot_moveit | 0.0.0 | 2 | ament_cmake | Not specified |

### Descripciones

**cr20_moveit**: An automatically generated package with all the configuration and launch files for using the cr20_ro

**dobot_moveit**: TODO: Package description


### Dependencias

**cr20_moveit**: ament_cmake, moveit_ros_move_group, moveit_kinematics, moveit_planners, moveit_simple_controller_manager, joint_state_publisher, joint_state_publisher_gui, tf2_ros, xacro, controller_manager, dobot_rviz, moveit_configs_utils, moveit_ros_move_group, moveit_ros_visualization, moveit_ros_warehouse, moveit_setup_assistant, robot_state_publisher, rviz2, rviz_common, rviz_default_plugins, tf2_ros, xacro

**dobot_moveit**: ament_copyright, ament_flake8, ament_pep257, python3-pytest


## 2. Detalle de Paquetes: RViz

| Paquete | Versión | Formato | Build Type | ROS Version |
|---------|---------|---------|------------|-------------|
| dobot_rviz | 0.0.0 | 2 | ament_cmake | Not specified |

### Descripciones

**dobot_rviz**: TODO: Package description


### Dependencias

**dobot_rviz**: ament_cmake, rviz2, xacro, robot_state_publisher, joint_state_publisher, ros2launch, ament_lint_auto, ament_lint_common


## 2. Detalle de Paquetes: Robot Description

| Paquete | Versión | Formato | Build Type | ROS Version |
|---------|---------|---------|------------|-------------|
| cra_description | 0.0.0 | 2 | ament_cmake | Not specified |

### Descripciones

**cra_description**: The cra_description package


### Dependencias

**cra_description**: ament_cmake


## 2. Detalle de Paquetes: Servo

| Paquete | Versión | Formato | Build Type | ROS Version |
|---------|---------|---------|------------|-------------|
| servo_action | 0.0.0 | 2 | ament_cmake | Not specified |

### Descripciones

**servo_action**: TODO: Package description


### Dependencias

**servo_action**: ament_copyright, ament_flake8, ament_pep257, python3-pytest


## 3. Análisis de Incompatibilidades

| Paquete | Problemas Potenciales |
|---------|----------------------|
| cr20_moveit | ✓ Usa MoveIt2 |
| cra_description | ✓ Sin problemas detectados |
| dobot_bringup_v4 | ✓ Sin problemas detectados |
| dobot_demo | ✓ Sin problemas detectados |
| dobot_gazebo | ✓ Usa Gazebo Sim (Harmonic)<br>✓ Usa ros2_control |
| dobot_moveit | ✓ Sin problemas detectados |
| dobot_msgs_v4 | ✓ Sin problemas detectados |
| dobot_rviz | ✓ Sin problemas detectados |
| gz_conveyorbelt | ✓ Usa Gazebo Sim (Harmonic) |
| pruebas_de_vision | ✓ Sin problemas detectados |
| servo_action | ✓ Sin problemas detectados |

---

## 4. Sugerencias para Isaac Sim

### Dependencias a reemplazar para Isaac Sim

| Paquete Original | Alternativa Isaac Sim |
|-----------------|----------------------|
| `ros_gz_sim` | `ros_ign_bridge` o APIs nativas de Isaac Sim |
| `gz-sim8` | `ignition-gazebo` o `gz-sim` |
| `gazebo_ros` | Eliminar (usar bridge ROS-Isaac) |
| `ros2_controllers` | Usar Isaac Actuation API |
| `robot_state_publisher` | Usar Isaac OM |

### Cambios recomendados

1. **Bridge ROS2-Isaac**: Crear bridges personalizados para topic translation
2. **URDF/USD**: Convertir meshes STL a USD para mejor rendimiento
3. **Controladores**: Reemplazar `joint_trajectory_controller` con Isaac Codelets
4. **MoveIt**: Mantener para planificación, usar Isaac execution cuando sea posible
5. **Simulación**: El archivo `Modelo/CR20V_isaac.usd` ya existe y puede ser usado directamente

### Paquetes compatibles con Isaac Sim

Los siguientes paquetes NO requieren cambios:
- cr20_moveit
- cra_description
- dobot_bringup_v4
- dobot_demo
- dobot_gazebo
- dobot_moveit
- dobot_msgs_v4
- dobot_rviz
- gz_conveyorbelt
- pruebas_de_vision
- servo_action


### Paquetes que requieren modificación


---

## 5. Dependencias Completas por Paquete

### Bringup/Control

#### dobot_bringup_v4 (`0.0.0`)

**Descripción**: TODO: Package description

**Ubicación**: `/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/package.xml`

**Build Type**: `ament_cmake`

- **buildtool_depend**: ament_cmake
- **depend**: rclcpp, rclcpp_action, std_msgs, dobot_msgs_v4
- **test_depend**: ament_lint_auto, ament_lint_common

### Demo

#### dobot_demo (`0.0.0`)

**Descripción**: TODO: Package description

**Ubicación**: `/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_demo/package.xml`

**Build Type**: `ament_cmake`

- **test_depend**: ament_copyright, ament_flake8, ament_pep257, python3-pytest

### Gazebo

#### dobot_gazebo (`0.0.0`)

**Descripción**: Launch files and worlds for Dobot simulation in Gz Harmonic

**Ubicación**: `/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_gazebo/package.xml`

**Build Type**: `ament_cmake`

- **buildtool_depend**: ament_cmake
- **exec_depend**: ros_gz_sim, ros_gz_bridge, gz_ros2_control, robot_state_publisher, cra_description, xacro, controller_manager, joint_state_broadcaster, joint_trajectory_controller

#### gz_conveyorbelt (`1.0.0`)

**Descripción**: Conveyor Belt Controller for Gazebo Sim 8 (Harmonic) - Scripted Pose Control

**Ubicación**: `/home/iudc/dobot_ws/src/gz_conveyorbelt/package.xml`

**Build Type**: `ament_cmake`

- **buildtool_depend**: ament_cmake_python
- **exec_depend**: gz-sim8, gz-transport13
- **depend**: rclcpp, rclpy, std_msgs, std_srvs, geometry_msgs, example_interfaces, ros_gz_bridge
- **test_depend**: ament_lint_auto, ament_lint_common, ament_copyright, ament_flake8, ament_pep257, python3-pytest

#### pruebas_de_vision (`0.0.1`)

**Descripción**: Paquete aislado para pruebas de visión con cámara en Gazebo y detección YOLOv8-OBB

**Ubicación**: `/home/iudc/dobot_ws/src/pruebas_de_vision/package.xml`

**Build Type**: `ament_cmake`

- **buildtool_depend**: ament_python
- **exec_depend**: ros_gz_sim, ros_gz_bridge, gz_sim_vendor, python3-opencv
- **depend**: rclpy, sensor_msgs, cv_bridge, geometry_msgs, vision_msgs, tf_transformations

### Messages/Services

#### dobot_msgs_v4 (`0.0.0`)

**Descripción**: TODO: Package description

**Ubicación**: `/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/package.xml`

**Build Type**: `ament_cmake`

- **buildtool_depend**: ament_cmake
- **exec_depend**: rosidl_default_runtime
- **depend**: rclcpp, std_msgs, rosidl_default_generators
- **test_depend**: ament_lint_auto, ament_lint_common

### MoveIt

#### cr20_moveit (`0.3.0`)

**Descripción**: An automatically generated package with all the configuration and launch files for using the cr20_ro

**Ubicación**: `/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cr20_moveit/package.xml`

**Build Type**: `ament_cmake`

- **buildtool_depend**: ament_cmake
- **exec_depend**: moveit_ros_move_group, moveit_kinematics, moveit_planners, moveit_simple_controller_manager, joint_state_publisher, joint_state_publisher_gui, tf2_ros, xacro, controller_manager, dobot_rviz, moveit_configs_utils, moveit_ros_move_group, moveit_ros_visualization, moveit_ros_warehouse, moveit_setup_assistant, robot_state_publisher, rviz2, rviz_common, rviz_default_plugins, tf2_ros, xacro

#### dobot_moveit (`0.0.0`)

**Descripción**: TODO: Package description

**Ubicación**: `/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_moveit/package.xml`

**Build Type**: `ament_cmake`

- **test_depend**: ament_copyright, ament_flake8, ament_pep257, python3-pytest

### RViz

#### dobot_rviz (`0.0.0`)

**Descripción**: TODO: Package description

**Ubicación**: `/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_rviz/package.xml`

**Build Type**: `ament_cmake`

- **buildtool_depend**: ament_cmake
- **exec_depend**: rviz2, xacro, robot_state_publisher, joint_state_publisher, ros2launch
- **test_depend**: ament_lint_auto, ament_lint_common

### Robot Description

#### cra_description (`0.0.0`)

**Descripción**: The cra_description package

**Ubicación**: `/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cra_description/package.xml`

**Build Type**: `ament_cmake`

- **buildtool_depend**: ament_cmake

### Servo

#### servo_action (`0.0.0`)

**Descripción**: TODO: Package description

**Ubicación**: `/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/servo_action/package.xml`

**Build Type**: `ament_cmake`

- **test_depend**: ament_copyright, ament_flake8, ament_pep257, python3-pytest


---

*Reporte generado automáticamente por analyze_packages.py*
