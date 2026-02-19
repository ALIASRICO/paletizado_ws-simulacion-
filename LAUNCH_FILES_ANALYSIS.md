# Análisis de Launch Files del Robot CR20

Este documento analiza todos los archivos de lanzamiento (launch files) disponibles para el robot Dobot CR20, explicando qué hacen, qué parámetros aceptan y cómo se relacionan entre sí.

---

## 1. Resumen de Launch Files

### 1.1 Total de archivos encontrados: 19

| Paquete | Launch Files |
|---------|--------------|
| cr20_moveit | 10 |
| dobot_gazebo | 4 |
| dobot_moveit | 4 |
| dobot_bringup_v4 | 1 |

---

## 2. Diagramas de Dependencias

### 2.1 Estructura General

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ARQUITECTURA DE LANZAMIENTO                        │
└─────────────────────────────────────────────────────────────────────────────┘

                        ┌──────────────────────────────┐
                        │     dobot_bringup_ros2       │
                        │   (Robot físico real)        │
                        └──────────────────────────────┘
                                      │
                    (requiere: IP del robot)
                                      │
                                      ▼
        ┌─────────────────────────────────────────────────────────┐
        │                    GAZEBO SIMULATION                    │
        │  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐  │
        │  │palletizing_ │  │ dobot_      │  │ gazebo_     │  │
        │  │sim          │  │ gazebo      │  │ moveit      │  │
        │  └─────────────┘  └─────────────┘  └──────────────┘  │
        │        │                │                 │           │
        └────────┼────────────────┼─────────────────┼───────────┘
                 │                │                 │
                 ▼                ▼                 ▼
        ┌─────────────────────────────────────────────────────────┐
        │                   MOVEIT PLANNING                       │
        │  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐  │
        │  │ demo        │  │ dobot_      │  │ moveit_     │  │
        │  │             │  │ moveit      │  │ gazebo      │  │
        │  └─────────────┘  └─────────────┘  └──────────────┘  │
        │        │                │                 │           │
        │        └────────────────┼─────────────────┘           │
        │                         │                              │
        │                         ▼                              │
        │              ┌──────────────────────┐                  │
        │              │   move_group        │                  │
        │              │  (Planificador)     │                  │
        │              └──────────────────────┘                  │
        │                         │                              │
        │                         ▼                              │
        │              ┌──────────────────────┐                  │
        │              │ rviz2 / Visualizador│                  │
        │              └──────────────────────┘                  │
        └─────────────────────────────────────────────────────────┘
```

### 2.2 Flujo de Datos entre Componentes

```
┌──────────────┐     /joint_states      ┌────────────────┐
│   Gazebo    │ ──────────────────────▶│ joint_state    │
│   Sim       │                         │ broadcaster    │
└──────────────┘                         └───────┬────────┘
                                                  │
                                          /joint_states
                                                  │
                                                  ▼
                                        ┌────────────────┐
                                        │   move_group   │
                                        │ (planificador) │
                                        └───────┬────────┘
                                                │
                                   /plan /execute_traj
                                                │
                                                ▼
                                        ┌────────────────┐
                                        │   controller    │
                                        │ (ejecutor)     │
                                        └───────┬────────┘
                                                │
                                        /joint_commands
                                                │
                                                ▼
                                        ┌────────────────┐
                                        │ Gazebo/Real   │
                                        │ Robot         │
                                        └───────────────┘
```

---

## 3. Launch Files de cr20_moveit

### 3.1 demo.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/launch/demo.launch.py`

**Descripción**: Launch de demostración que inicia MoveIt con RViz en modo visualización.

**Uso**:
```bash
ros2 launch cr20_moveit demo.launch.py
```

**Qué hace**:
- Inicia Robot State Publisher
- Inicia MoveIt Move Group
- Inicia RViz2 con configuración de MoveIt

**Parámetros**: Ninguno (usa valores por defecto)

**Nodos lanzados**:
- `robot_state_publisher`
- `move_group`
- `rviz2`

---

### 3.2 dobot_moveit.launch.py ⭐ (PRINCIPAL)

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/launch/dobot_moveit.launch.py`

**Descripción**: Launch principal que integra robot state publisher, move group y RViz.

**Uso**:
```bash
ros2 launch cr20_moveit dobot_moveit.launch.py
```

**Qué hace**:
1. Lanza Robot State Publisher (rsp)
2. Lanza MoveIt Move Group (planificador)
3. Lanza RViz2 (visualización)

**Parámetros**:
| Parámetro | Tipo | Default | Descripción |
|-----------|------|---------|-------------|
| `publish_frequency` | float | 15.0 | Frecuencia de publicación de tf |
| `debug` | bool | false | Habilitar modo debug |
| `allow_trajectory_execution` | bool | true | Permitir ejecución de trayectorias |
| `publish_monitored_planning_scene` | bool | true | Publicar escena de planificación |
| `capabilities` | string | (default) | Capabilities de MoveGroup |
| `disable_capabilities` | string | (default) | Capabilities a deshabilitar |
| `rviz_config` | string | moveit.rviz | Archivo de configuración RViz |

**Nodos lanzados**:
| Nodo | Paquete | Descripción |
|------|---------|-------------|
| robot_state_publisher | robot_state_publisher | Publica transforms del robot |
| move_group | moveit_ros_move_group | Planificador de movimientos |
| rviz2 | rviz2 | Visualizador 3D |

**Tópicos**:
- `/joint_states` - Estados de las articulaciones
- `/robot_description` - Descripción del robot (URDF)
- `/planning_scene` - Escena de planificación

---

### 3.3 move_group.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/launch/move_group.launch.py`

**Descripción**: Lanza solo el nodo MoveIt Move Group.

**Uso**:
```bash
ros2 launch cr20_moveit move_group.launch.py
```

**Nodos**: `move_group`

---

### 3.4 rsp.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/launch/rsp.launch.py`

**Descripción**: Robot State Publisher - publica la descripción cinemática del robot.

**Uso**:
```bash
ros2 launch cr20_moveit rsp.launch.py
```

**Nodos**: `robot_state_publisher`

---

### 3.5 moveit_rviz.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/launch/moveit_rviz.launch.py`

**Descripción**: Inicia solo RViz con la configuración de MoveIt.

**Uso**:
```bash
ros2 launch cr20_moveit moveit_rviz.launch.py
```

**Nodos**: `rviz2`

---

### 3.6 moveit_gazebo.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/launch/moveit_gazebo.launch.py`

**Descripción**: MoveIt con Gazebo - integra MoveIt con simulación Gazebo.

**Uso**:
```bash
ros2 launch cr20_moveit moveit_gazebo.launch.py
```

**Parámetros**:
- `use_sim_time: true` - Usa tiempo de simulación
- `debug` - Modo debug

**Nodos**:
- `move_group` (con use_sim_time=true)
- `rviz2` (con use_sim_time=true)

---

### 3.7 spawn_controllers.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/launch/spawn_controllers.launch.py`

**Descripción**: Lanza los controladores ROS2.

**Uso**:
```bash
ros2 launch cr20_moveit spawn_controllers.launch.py
```

**Qué hace**:
- Carga `joint_state_broadcaster`
- Carga `cr20_group_controller`

---

### 3.8 static_virtual_joint_tfs.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/launch/static_virtual_joint_tfs.launch.py`

**Descripción**: Publica transforms estáticos para joints virtuales.

---

### 3.9 warehouse_db.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/launch/warehouse_db.launch.py`

**Descripción**: Base de datos de MoveIt para guardar estados.

---

### 3.10 setup_assistant.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/launch/setup_assistant.launch.py`

**Descripción**: Asistente de configuración MoveIt.

---

## 4. Launch Files de dobot_bringup_v4

### 4.1 dobot_bringup_ros2.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/launch/dobot_bringup_ros2.launch.py`

**Descripción**: Lanza el nodo de comunicación con el robot físico real via TCP/IP.

**Uso**:
```bash
# Con variables de entorno
export IP_address=192.168.1.6
export DOBOT_TYPE=cr20
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

**Parámetros (vía entorno)**:
| Variable | Descripción | Ejemplo |
|----------|-------------|---------|
| `IP_address` | IP del robot | 192.168.1.6 |
| `DOBOT_TYPE` | Tipo de robot | cr20 |

**Parámetros (del archivo param.json)**:
- `robot_number`
- `trajectory_duration`
- `robot_node_name`

**Nodos**:
| Nodo | Paquete | Descripción |
|------|---------|-------------|
| cr_robot_ros2 | dobot_bringup_v4 | Nodo de comunicación TCP/IP |

**Servicios**:
- Control del robot

---

## 5. Launch Files de dobot_gazebo

### 5.1 palletizing_sim.launch.py ⭐ (SIMULACIÓN COMPLETA)

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/dobot_gazebo/models/launch/palletizing_sim.launch.py`

**Descripción**: Launch completo de simulación con Gazebo, robot, controlador de cinta transportadora y MoveIt.

**Uso**:
```bash
# Basic
ros2 launch dobot_gazebo palletizing_sim.launch.py

# Con mundo específico
ros2 launch dobot_gazebo palletizing_sim.launch.py world:=palletizing.sdf

# Modo headless (sin GUI)
ros2 launch dobot_gazebo palletizing_sim.launch.py headless:=true
```

**Parámetros**:
| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `world` | minimal_calibration.sdf | Archivo SDF del mundo |
| `headless` | false | Ejecutar sin GUI |

**Variables de entorno**:
| Variable | Default | Descripción |
|----------|---------|-------------|
| `DOBOT_TYPE` | cr20 | Tipo de robot |

**Nodos lanzados**:
| Nodo | Paquete | Descripción |
|------|---------|-------------|
| gz_sim | ros_gz_sim | Servidor de simulación Gazebo |
| robot_state_publisher | robot_state_publisher | Publica descripción del robot |
| create | ros_gz_sim | Spawn del robot en Gazebo |
| clock_bridge | ros_gz_bridge | Bridge de reloj |
| conveyor_bridge | ros_gz_bridge | Bridge de cámara |
| conveyor_logic_controller | gz_conveyorbelt | Controlador de cinta |
| joint_state_broadcaster | ros2_control | Publica estados |
| cr20_group_controller | ros2_control | Controlador de trayectorias |

**Tópicos**:
- `/clock` - Reloj de simulación
- `/camera/color/image_raw` - Imagen de cámara
- `/joint_states` - Estados articulares
- `/cr20_robot/joint_trajectory/command` - Comandos de trayectoria

**Secuencia de ejecución**:
1. Inicia Gazebo Sim
2. Publica robot_description
3. Spawn robot en Gazebo (z=0.72m)
4. Carga joint_state_broadcaster
5. Carga joint_trajectory_controller
6. Inicia controlador de cinta transportadora (5s delay)

---

### 5.2 dobot_gazebo.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/dobot_gazebo/models/launch/dobot_gazebo.launch.py`

**Descripción**: Simulación básica de Gazebo con el robot.

**Uso**:
```bash
ros2 launch dobot_gazebo dobot_gazebo.launch.py
```

**Nodos**:
- gz_sim
- robot_state_publisher
- create (spawn robot)
- gz_bridge (clock)

---

### 5.3 gazebo_moveit.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/dobot_gazebo/models/launch/gazebo_moveit.launch.py`

**Descripción**: Gazebo con MoveIt y controladores.

**Uso**:
```bash
ros2 launch dobot_gazebo gazebo_moveit.launch.py
```

**Nodos**:
- gz_sim
- robot_state_publisher
- create (spawn)
- gz_bridge
- joint_state_broadcaster (cargado)
- cr20_group_controller (cargado)

---

### 5.4 gazebo_with_camera.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/dobot_gazebo/models/launch/gazebo_with_camera.launch.py`

**Descripción**: Gazebo con cámara RealSense D435i.

---

## 6. Launch Files de dobot_moveit

### 6.1 dobot_moveit.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/dobot_moveit/launch/dobot_moveit.launch.py`

**Descripción**: Wrapper que incluye dobot_joint y el launch específico.

**Uso**:
```bash
export DOBOT_TYPE=cr20
ros2 launch dobot_moveit dobot_moveit.launch.py
```

---

### 6.2 dobot_joint.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/dobot_moveit/launch/dobot_joint.launch.py`

**Descripción**: Nodos de acción y estados articulares para MoveIt.

**Nodos**:
| Nodo | Descripción |
|------|-------------|
| action_move_server | Servidor de acciones de movimiento |
| joint_states | Publicador de estados articulares |

---

### 6.3 moveit_demo.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/dobot_moveit/launch/moveit_demo.launch.py`

**Descripción**: Demo de MoveIt.

---

### 6.4 moveit_gazebo.launch.py

**Ruta**: `src/DOBOT_6Axis_ROS2_V4/dobot_moveit/launch/moveit_gazebo.launch.py`

**Descripción**: MoveIt con Gazebo.

---

## 7. Launch Principal Recomendado

### Para Simulación (Gazebo + MoveIt)

```bash
# Configurar tipo de robot
export DOBOT_TYPE=cr20

# Lanzar simulación completa
ros2 launch dobot_gazebo palletizing_sim.launch.py world:=palletizing.sdf
```

Este launch incluye:
- ✅ Gazebo Sim
- ✅ Robot CR20 spawnneado
- ✅ Controladores ROS2 (joint_state_broadcaster + joint_trajectory_controller)
- ✅ Bridge ROS-Gazebo
- ✅ Controlador de cinta transportadora

### Para Robot Físico Real

```bash
# Configurar IP del robot
export IP_address=192.168.1.6
export DOBOT_TYPE=cr20

# Lanzar nodo de comunicación
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

---

## 8. Adaptación para Isaac Sim

### 8.1 Cambios Necesarios

| Componente Original | Alternativa Isaac Sim |
|-------------------|---------------------|
| Gazebo Sim | Isaac Sim (gz-sim) |
| ros_gz_sim | Isaac USD Composer |
| ros_gz_bridge | Isaac ROS Bridge |
| robot_state_publisher | Isaac Omniverse Graph |
| joint_trajectory_controller | Isaac Actuation |

### 8.2 Nodos a Reemplazar

```
ORIGINAL (Gazebo):
├── gz_sim (servidor simulación)
├── robot_state_publisher
├── ros_gz_bridge (topics)
├── joint_state_broadcaster
└── cr20_group_controller

PARA ISAAC SIM:
├── Isaac Sim (simulación)
├── Isaac Omniverse Graph (state pub)
├── ros_ign_bridge o custom bridge
├── Isaac Joint State Reader
└── Isaac Joint Trajectory Controller
```

### 8.3 Launch Simplificado para Isaac Sim

```python
# Propuesta de launch para Isaac Sim
def generate_launch_description():
    return LaunchDescription([
        # 1. Isaac Sim debe estar ejecutándose primero
        # 2. Bridge ROS2-Isaac
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            ],
        ),
        
        # 3. Robot State Publisher (opcional, puede usar Isaac)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),
        
        # 4. MoveIt (se mantiene igual)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('cr20_moveit'), 
                           'launch', 'move_group.launch.py')
            ),
            launch_arguments={'use_sim_time': True}.items(),
        ),
    ])
```

---

## 9. Resumen de Tópicos y Servicios

### 9.1 Tópicos Publicados

| Tópico | Tipo | Fuente |
|--------|------|--------|
| /joint_states | sensor_msgs/JointState | robot_state_publisher / jsb |
| /robot_description | std_msgs/String | xacro |
| /planning_scene | moveit_msgs/PlanningScene | move_group |
| /clock | rosgraph_msgs/Clock | Gazebo/Isaac |
| /cr20_robot/joint_trajectory/command | trajectory_msgs/JointTrajectory | controller |

### 9.2 Tópicos Suscritos

| Tópico | Tipo | Destino |
|--------|------|---------|
| /cr20_robot/joint_trajectory/command | trajectory_msgs/JointTrajectory | controller |
| /planning_scene | moveit_msgs/PlanningScene | move_group |

---

*Documento generado automáticamente para análisis de launch files del CR20*
