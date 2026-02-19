# Plan de Migración: Gazebo → Isaac Sim

## RESUMEN EJECUTIVO

Este documento presenta el plan detallado de cambios necesarios para migrar la simulación del robot CR20 desde Gazebo (Harmonic) hacia NVIDIA Isaac Sim.

### Lista de Cambios Principales

| # | Cambio | Dificultad | Prioridad |
|---|--------|-------------|-----------|
| 1 | Crear bridges ROS-Isaac personalizados | Media | Alta |
| 2 | Modificar launch files para Isaac Sim | Media | Alta |
| 3 | Convertir meshes STL a USD | Fácil | Media |
| 4 | Adaptar controladores ros2_control | Difícil | Alta |
| 5 | Crear paquete isaac_bringup | Media | Media |
| 6 | Mantener MoveIt (compatible) | Fácil | Baja |
| 7 | Migrar Worlds SDF a formato Isaac | Media | Media |
| 8 | Adaptar controlador de cinta transportadora | Media | Baja |

### Cronograma Estimado

| Fase | Tareas | Tiempo Estimado |
|------|--------|-----------------|
| **Fase 1** | Configuración básica Isaac + Bridge | 2-3 horas |
| **Fase 2** | Migración de robot (meshes + física) | 3-4 horas |
| **Fase 3** | Integración MoveIt | 2 horas |
| **Fase 4** | Componentes adicionales (cinta, visión) | 3-4 horas |
| **Fase 5** | Pruebas y validación | 2-3 horas |
| | **TOTAL** | **12-16 horas** |

---

## 1. ¿Qué cambios ya hizo ALIASRICO para Jazzy?

### 1.1 Cambios Identificados en el Repositorio

El repositorio ya contiene varias adaptaciones para ROS2 Jazzy:

| Paquete | Cambio Realizado | Archivo |
|---------|-----------------|---------|
| dobot_gazebo | Dependencias para Gazebo Sim 8 (Harmonic) | package.xml |
| gz_conveyorbelt | Compatible con gz-sim8, gz-transport13 | package.xml |
| cr20_moveit | MoveIt2 compatible | package.xml |
| pruebas_de_vision | Integración con Gazebo y visión | package.xml |

### 1.2 Dependencias Jazzy Ya Presentes

```yaml
# Paquetes que funcionan con Jazzy:
- ros_gz_sim (Gazebo Sim)
- ros_gz_bridge
- gz_ros2_control
- moveit_ros_move_group
- moveit_kinematics
- moveit_planners
```

### 1.3 Lo que NO se ha cambiado para Jazzy

- ❌ Los launch files todavía usan configuración de Gazebo
- ❌ No hay bridges para Isaac Sim
- ❌ El robot usa el mismo URDF (correcto, pero我们需要 convertir meshes)

---

## 2. ¿Qué cambios adicionales necesitamos para Isaac Sim?

### 2.1 Cambios de Infraestructura

| Cambio | Descripción | Impacto |
|--------|-------------|---------|
| **Reemplazar ros_gz_sim** | Isaac Sim usa su propio motor de simulación | Alto |
| **Reemplazar ros_gz_bridge** | Usar ros_ign_bridge o custom bridge | Alto |
| **Adaptar robot_state_publisher** | Isaac usa Omniverse Graph | Medio |
| **Adaptar joint_trajectory_controller** | Isaac usa Isaac Actuation API | Alto |

### 2.2 Cambios de Simulación

| Cambio | Descripción | Impacto |
|--------|-------------|---------|
| **Convertir worlds SDF** | Isaac usa USD/nodes | Medio |
| **Convertir meshes STL→USD** | Isaac渲染更好 | Bajo |
| **Adaptar physics parameters** | Isaac usa PhysX | Bajo |

### 2.3 Cambios de Integración

| Cambio | Descripción | Impacto |
|--------|-------------|---------|
| **Crear Isaac Omnigraph** | Graph de simulación Isaac | Alto |
| **Adaptar controladores** | Isaac Actuation | Alto |
| **Mantenerse_moveit** | Compatible con use_sim_time | Bajo |

---

## 3. Archivos a CREAR nuevos

### 3.1 Lista de Archivos Nuevos

| # | Archivo | Ubicación | Propósito |
|---|--------|-----------|-----------|
| 1 | `isaac_bringup/package.xml` | `src/isaac_bringup/` | Meta-paquete para Isaac |
| 2 | `isaac_bringup/__init__.py` | `src/isaac_bringup/` | Inicialización Python |
| 3 | `isaac_robot_description/package.xml` | `src/isaac_robot_description/` | Descripción robot Isaac |
| 4 | `isaac_robot_description/config/robot.usd` | `src/isaac_robot_description/config/` | Modelo USD del robot |
| 5 | `isaac_bridges/launch/isaac_bridge.launch.py` | `src/isaac_bridges/launch/` | Bridges ROS-Isaac |
| 6 | `isaac_bridges/config/topics.yaml` | `src/isaac_bridges/config/` | Configuración de tópicos |
| 7 | `cr20_moveit/launch/isaac_moveit.launch.py` | `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/launch/` | MoveIt para Isaac |
| 8 | ` Modelo/CR20_isaac_updated.usd` | `Modelo/` | Modelo actualizado |

### 3.2 Detalle de Archivos Nuevos

#### 3.2.1 isaac_bringup/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>isaac_bringup</name>
  <version>1.0.0</version>
  <description>Launch files for Isaac Sim robot simulation</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>ros_ign_bridge</exec_depend>
  <exec_depend>ros_ign_image</exec_depend>
  <exec_depend>ros_ign_point_cloud</exec_depend>
</package>
```

#### 3.2.2 isaac_bridge.launch.py

```python
#!/usr/bin/env python3
"""
Bridge entre ROS2 y Isaac Sim
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Clock bridge
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            ],
        ),
        # Joint states bridge
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='joint_states_bridge',
            arguments=[
                '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            ],
        ),
        # TF bridge
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='tf_bridge',
            arguments=[
                '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
        ),
    ])
```

---

## 4. Archivos a MODIFICAR

### 4.1 Tabla de Modificaciones

| # | Archivo | Acción | Cambio Específico | Razón | Dificultad |
|---|---------|--------|-------------------|-------|------------|
| 1 | `dobot_gazebo/package.xml` | MODIFICAR | Agregar deps Isaac | Soporte Isaac Sim | Fácil |
| 2 | `dobot_gazebo/launch/palletizing_sim.launch.py` | CREAR NUEVO | isaac_palletizing.launch.py | Reemplazar Gazebo | Media |
| 3 | `cr20_moveit/config/joint_limits.yaml` | MANTENER | - | Compatible | - |
| 4 | `cr20_moveit/config/ros2_controllers.yaml` | MODIFICAR | Cambiar a Isaac controller | Isaac usa diferentes interfaces | Difícil |
| 5 | `cra_description/urdf/cr20_robot.urdf` | MANTENER | - | Compatible | - |
| 6 | `cra_description/meshes/*.STL` | CREAR COPIA USD | Convertir a USD | Mejor rendimiento Isaac | Fácil |
| 7 | `gz_conveyorbelt/package.xml` | MANTENER | - | Compatible con cambios menores | - |
| 8 | `gz_conveyorbelt/launch/*.launch.py` | MODIFICAR | Adaptar a Isaac | Isaac usa diferentes topics | Media |
| 9 | `dobot_msgs_v4/package.xml` | MANTENER | - | Compatible | - |
| 10 | `pruebas_de_vision/package.xml` | MODIFICAR | Agregar Isaac vision | Integrar con Isaac sensors | Media |

### 4.2 Detalle de Modificaciones Críticas

#### 4.2.1 ros2_controllers.yaml → Isaac Adaptation

**Archivo original** (`ros2_controllers.yaml`):
```yaml
cr20_group_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

**Cambio necesario**: El JointTrajectoryController estándar funciona, pero necesita:
- Bridge hacia Isaac Sim para ejecutar comandos
- Configuración de sim_time=true

#### 4.2.2 Launch File Principal

**Archivo a crear**: `isaac_palletizing.launch.py`

```python
#!/usr/bin/env python3
"""
Launch para Isaac Sim + CR20 Robot
Uso:
    ros2 launch isaac_bringup isaac_palletizing.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    robot_type = os.getenv("DOBOT_TYPE", "cr20")
    
    # Paths
    isaac_bringup_path = get_package_share_directory('isaac_bringup')
    cr20_moveit_path = get_package_share_directory('cr20_moveit')
    
    return LaunchDescription([
        # Isaac Bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(isaac_bringup_path, 'launch', 'isaac_bridge.launch.py')
            ),
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),
        
        # MoveIt (compatible with Isaac)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cr20_moveit_path, 'launch', 'move_group.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true'
            }.items(),
        ),
    ])
```

---

## 5. Archivos a MANTENER igual

### 5.1 Lista de Archivos Sin Cambios

| # | Archivo | Razón |
|---|---------|-------|
| 1 | `cr20_moveit/config/kinematics.yaml` | KDL es compatible |
| 2 | `cr20_moveit/config/cr20_robot.srdf` | SRDF es independiente del simulador |
| 3 | `cr20_moveit/config/moveit_controllers.yaml` | MoveIt es compatible |
| 4 | `cr20_moveit/config/initial_positions.yaml` | Formato estándar ROS2 |
| 5 | `cr20_moveit/config/pilz_cartesian_limits.yaml` | Planificador independiente |
| 6 | `dobot_msgs_v4/msg/*.msg` | Mensajes ROS estándar |
| 7 | `dobot_msgs_v4/srv/*.srv` | Servicios ROS estándar |
| 8 | `dobot_bringup_v4/**/*` | Para robot físico real |
| 9 | `cra_description/urdf/cr20_robot.urdf` | Formato universal |
| 10 | `dobot_rviz/**/*` | RViz funciona con cualquier simulador |

### 5.2 Explicación de Compatibilidad

```
┌─────────────────────────────────────────────────────────────┐
│                    COMPONENTES COMPATIBLES                  │
├─────────────────────────────────────────────────────────────┤
│  MoveIt2     → Compatible con cualquier simulador          │
│  ros2_control → Funciona con bridges a Isaac               │
│  URDF/XACRO  → Formato universal                           │
│  Messages    → ROS2 estándar                               │
│  RViz       → Funciona independientemente                   │
└─────────────────────────────────────────────────────────────┘
```

---

## 6. DIAGRAMA DE FLUJO: Gazebo → Isaac Sim

### 6.1 Arquitectura Actual (Gazebo)

```
                    ┌─────────────────────────────────────┐
                    │         GAZEBO HARMONIC              │
                    │  ┌─────────────────────────────┐    │
                    │  │ gz-sim (servidor)          │    │
                    │  │ - Física del robot         │    │
                    │  │ - Rendering                │    │
                    │  │ - Sensores                │    │
                    │  └─────────────────────────────┘    │
                    └──────────────┬──────────────────────┘
                                   │
                    ┌──────────────▼──────────────────────┐
                    │         ROS_GZ_BRIDGE              │
                    │  ┌─────────────────────────────┐   │
                    │  │ /clock                     │   │
                    │  │ /joint_states              │   │
                    │  │ /tf                       │   │
                    │  │ /camera/*                 │   │
                    │  └─────────────────────────────┘   │
                    └──────────────┬──────────────────────┘
                                   │
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
        ▼                          ▼                          ▼
┌───────────────┐        ┌─────────────────┐        ┌────────────────┐
│ robot_state   │        │   MoveIt        │        │  Controllers   │
│ publisher     │        │   MoveGroup     │        │  ros2_control  │
│               │        │   Planning      │        │  Trajectory    │
└───────────────┘        └─────────────────┘        └────────────────┘
```

### 6.2 Arquitectura Target (Isaac Sim)

```
                    ┌─────────────────────────────────────┐
                    │          ISAAC SIM                  │
                    │  ┌─────────────────────────────┐    │
                    │  │ Isaac Omnigraph             │    │
                    │  │ - Physics Scene             │    │
                    │  │ - Robot USD                 │    │
                    │  │ - Sensors                   │    │
                    │  └─────────────────────────────┘    │
                    └──────────────┬──────────────────────┘
                                   │
                    ┌──────────────▼──────────────────────┐
                    │       ROS_IGN_BRIDGE               │
                    │  ┌─────────────────────────────┐   │
                    │  │ /clock                     │   │
                    │  │ /joint_states              │   │
                    │  │ /tf                       │   │
                    │  │ /camera/*                 │   │
                    │  └─────────────────────────────┘   │
                    └──────────────┬──────────────────────┘
                                   │
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
        ▼                          ▼                          ▼
┌───────────────┐        ┌─────────────────┐        ┌────────────────┐
│ robot_state   │        │   MoveIt        │        │  Isaac         │
│ publisher     │        │   MoveGroup     │        │  Actuation     │
│ (use_sim_time)│        │   (compatible) │        │  API           │
└───────────────┘        └─────────────────┘        └────────────────┘
```

### 6.3 Comparativa de Cambios

| Componente | Gazebo (Antes) | Isaac Sim (Después) | Cambio |
|------------|----------------|---------------------|--------|
| Simulador | gz-sim8 | Isaac Sim | REEMPLAZAR |
| Bridge | ros_gz_bridge | ros_ign_bridge | MODIFICAR |
| Física | ODE/SimPhys | PhysX | AUTOMÁTICO |
| Meshes | STL | USD (mejora) | CONVERTIR |
| Controlador | ros2_control | Isaac Actuation | ADAPTAR |
| MoveIt | move_group | move_group | MANTENER |
| Visualización | RViz | RViz/Isaac Viewer | MANTENER |

---

## 7. PASOS DE IMPLEMENTACIÓN

### Fase 1: Configuración Básica Isaac + Bridge (2-3 horas)

| Paso | Tarea | Dependencia | Archivo Resultado |
|------|-------|-------------|-------------------|
| 1.1 | Crear paquete isaac_bringup | - | `src/isaac_bringup/package.xml` |
| 1.2 | Crear launch de bridge básico | 1.1 | `isaac_bridge.launch.py` |
| 1.3 | Probar bridge /clock | 1.2 | - |
| 1.4 | Probar bridge /joint_states | 1.2 | - |

### Fase 2: Migración Robot (3-4 horas)

| Paso | Tarea | Dependencia | Archivo Resultado |
|------|-------|-------------|-------------------|
| 2.1 | Identificar meshes STL | - | `meshes/cr20/*.STL` |
| 2.2 | Convertir meshes a USD | 1.1 | `Modelo/CR20_isaac.usd` (actualizar) |
| 2.3 | Verificar física en Isaac | 2.2 | - |
| 2.4 | Probar spawn del robot | 2.3 | - |

### Fase 3: Integración MoveIt (2 horas)

| Paso | Tarea | Dependencia | Archivo Resultado |
|------|-------|-------------|-------------------|
| 3.1 | Verificar MoveIt con use_sim_time | 1.2 | - |
| 3.2 | Probar planificación | 3.1 | - |
| 3.3 | Probar ejecución | 3.2 | - |

### Fase 4: Componentes Adicionales (3-4 horas)

| Paso | Tarea | Dependencia | Archivo Resultado |
|------|-------|-------------|-------------------|
| 4.1 | Migrar cinta transportadora | 2.2 | `gz_conveyorbelt/isaac_*.py` |
| 4.2 | Migrar cámara/visión | 1.2 | `pruebas_de_vision/isaac_*.py` |
| 4.3 | Migrar worlds | 2.2 | `worlds/*.usd` |

### Fase 5: Pruebas y Validación (2-3 horas)

| Paso | Tarea | Dependencia | Archivo Resultado |
|------|-------|-------------|-------------------|
| 5.1 | Prueba de navegación | 3.3 | - |
| 5.2 | Prueba de picking/place | 4.1 | - |
| 5.3 | Integración completa | 4.2 | - |

---

## 8. Resumen de Cambios por Archivo

### 8.1 ARCHIVOS NUEVOS (8 archivos)

| Archivo | Propósito |
|---------|-----------|
| `src/isaac_bringup/package.xml` | Meta-paquete Isaac |
| `src/isaac_bringup/__init__.py` | Inicialización |
| `src/isaac_bringup/launch/isaac_bridge.launch.py` | Bridges ROS-Isaac |
| `src/isaac_bringup/config/topics.yaml` | Configuración topics |
| `src/isaac_robot_description/package.xml` | Descripción robot |
| `src/isaac_robot_description/config/robot.usd` | Modelo USD |
| `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/launch/isaac_moveit.launch.py` | MoveIt Isaac |
| `Modelo/CR20V_isaac.usd` (actualizar) | Modelo actualizado |

### 8.2 ARCHIVOS A MODIFICAR (5 archivos)

| Archivo | Cambio |
|---------|--------|
| `src/dobot_gazebo/package.xml` | Agregar deps Isaac |
| `src/DOBOT_6Axis_ROS2_V4/dobot_gazebo/launch/isaac_palletizing.launch.py` | Nuevo launch Isaac |
| `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/config/ros2_controllers.yaml` | Adaptar controllers |
| `src/gz_conveyorbelt/package.xml` | Dependencias menores |
| `src/pruebas_de_vision/package.xml` | Agregar Isaac vision |

### 8.3 ARCHIVOS A MANTENER (10+ archivos)

| Archivo | Razón |
|---------|-------|
| `cr20_moveit/config/kinematics.yaml` | Compatible |
| `cr20_moveit/config/cr20_robot.srdf` | Compatible |
| `cr20_moveit/config/moveit_controllers.yaml` | Compatible |
| `cr20_moveit/config/initial_positions.yaml` | Compatible |
| `dobot_msgs_v4/**/*` | Estándar ROS2 |
| `dobot_bringup_v4/**/*` | Robot físico |
| `cra_description/urdf/cr20_robot.urdf` | Formato universal |
| `dobot_rviz/**/*` | Independiente |

---

## 9. Notas Finales

### 9.1 Estrategia de Migración Recomendada

1. **No eliminar Gazebo** - Mantener ambos sistemas durante la transición
2. **Usar variables de entorno** - Distinguir entre simulación Gazebo e Isaac
3. **Testing incremental** - Probar cada fase antes de continuar
4. **Documentar cambios** - Mantener registro de modificaciones

### 9.2 Comandos de Referencia

```bash
# Gazebo (actual)
export DOBOT_TYPE=cr20
ros2 launch dobot_gazebo palletizing_sim.launch.py world:=palletizing.sdf

# Isaac Sim (futuro)
export DOBOT_TYPE=cr20
export SIMULATOR=isaac
ros2 launch isaac_bringup isaac_palletizing.launch.py
```

---

*Documento generado automáticamente para planificación de migración Gazebo → Isaac Sim*
