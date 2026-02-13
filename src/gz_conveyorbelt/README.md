# gz_conveyorbelt

Plugin de Cinta Transportadora para Gazebo Sim 8 (Harmonic)

## Descripción

Este paquete proporciona un plugin de Gazebo Sim 8 para simular una cinta transportadora. El plugin controla un joint prismático que representa la superficie de la cinta, moviéndola continuamente y arrastrando objetos por fricción.

## Instalación

```bash
cd ~/dobot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select gz_conveyorbelt --symlink-install
source install/setup.bash
```

## Uso

### Opción 1: Script de demostración

```bash
source install/setup.bash
./install/gz_conveyorbelt/lib/gz_conveyorbelt/run_conveyor_demo.sh
```

### Opción 2: Launch file ROS2

```bash
source install/setup.bash
ros2 launch gz_conveyorbelt conveyor_demo.launch.py
```

### Opción 3: Manual

```bash
source install/setup.bash
export GZ_PLUGIN_PATH=/home/iudc/dobot_ws/install/gz_conveyorbelt/lib:$GZ_PLUGIN_PATH
gz sim -r install/gz_conveyorbelt/share/gz_conveyorbelt/worlds/conveyor_demo.sdf
```

## Control de la Cinta

La cinta se controla mediante topics de Gazebo Transport:

```bash
# Iniciar cinta al 50% de velocidad
gz topic -t /conveyor/power -m gz.msgs.Double -p 'data: 50.0'

# Iniciar cinta al 80% de velocidad
gz topic -t /conveyor/power -m gz.msgs.Double -p 'data: 80.0'

# Detener cinta
gz topic -t /conveyor/power -m gz.msgs.Double -p 'data: 0.0'
```

## Parámetros del Plugin

En el archivo SDF del modelo:

```xml
<plugin filename="gz_conveyorbelt" name="gz_conveyorbelt::ConveyorBeltPlugin">
  <joint_name>belt_joint</joint_name>      <!-- Nombre del joint prismático -->
  <max_velocity>0.2</max_velocity>          <!-- Velocidad máxima en m/s -->
  <power>50</power>                         <!-- Potencia inicial (0-100) -->
  <topic>/conveyor/power</topic>            <!-- Topic de control -->
</plugin>
```

## Estructura del Modelo

El modelo de cinta transportadora debe tener:

1. **Base estática**: Link fijo que soporta la cinta
2. **Superficie móvil**: Link que se mueve sobre un joint prismático
3. **Joint prismático**: Conecta la base con la superficie, permite movimiento en un eje

### Ejemplo de modelo SDF

```xml
<model name="conveyor_belt">
  <!-- Base fija -->
  <link name="base_link">
    <static>true</static>
    <visual name="base_visual">
      <geometry><box><size>0.8 3.0 0.1</size></box></geometry>
    </visual>
  </link>
  
  <!-- Superficie móvil -->
  <link name="belt_link">
    <inertial>
      <mass>5.0</mass>
    </inertial>
    <visual name="belt_visual">
      <geometry><box><size>0.6 2.9 0.02</size></box></geometry>
    </visual>
    <collision name="belt_collision">
      <geometry><box><size>0.6 2.9 0.02</size></box></geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>  <!-- Fricción alta para mover objetos -->
          </ode>
        </friction>
      </surface>
    </collision>
  </link>
  
  <!-- Joint prismático -->
  <joint name="belt_joint" type="prismatic">
    <parent>base_link</parent>
    <child>belt_link</child>
    <axis>
      <xyz>0 1 0</xyz>  <!-- Dirección de movimiento -->
      <limit>
        <lower>0</lower>
        <upper>1.0</upper>  <!-- Límite para reset -->
      </limit>
    </axis>
  </joint>
  
  <!-- Plugin -->
  <plugin filename="gz_conveyorbelt" name="gz_conveyorbelt::ConveyorBeltPlugin">
    <joint_name>belt_joint</joint_name>
    <max_velocity>0.2</max_velocity>
  </plugin>
</model>
```

## Cómo Funciona

1. El plugin controla la velocidad del joint prismático
2. Cuando el joint alcanza su límite superior, se resetea a 0
3. Los objetos sobre la cinta se mueven por fricción física
4. La velocidad se calcula como: `velocidad = max_velocity * (power / 100)`

## Adaptación desde IFRA_ConveyorBelt

Este plugin está adaptado del proyecto [IFRA_ConveyorBelt](https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt) de Cranfield University, originalmente diseñado para Gazebo Classic.

Cambios principales:
- API de Gazebo Sim 8 (gz-sim) en lugar de Gazebo Classic
- Uso de `gz::sim::components` para acceso a componentes
- Mensajes `gz.msgs.Double` para control simplificado
- Compatible con ROS2 Jazzy

## Licencia

Apache-2.0 (basado en IFRA_ConveyorBelt)

## Créditos

- IFRA Group - Cranfield University (plugin original)
- Adaptación para Gazebo Sim 8
