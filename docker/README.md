# Docker - Sistema Completo DOBOT CR20V + YOLO + ROS2 Jazzy

Este directorio contiene la configuración Docker para el **sistema completo de paletizado** con el robot DOBOT CR20V.

## 🤖 Sistema Incluido

### Robot DOBOT CR20V
- **Payload**: 20 kg
- **DOF**: 6 ejes
- **Aplicación**: Paletizado automatizado

### Paquetes ROS2 Incluidos
| Paquete | Descripción |
|---------|-------------|
| `dobot_bringup_v4` | Driver TCP/IP para controlador Dobot |
| `dobot_msgs_v4` | Mensajes y servicios del SDK V4 |
| `cr20_moveit` | Configuración MoveIt2 para CR20V |
| `cra_description` | Modelos URDF y mallas 3D |
| `dobot_gazebo` | Simulación Gazebo Classic |
| `dobot_moveit` | Integración MoveIt adicional |
| `dobot_vision` | Visión con RealSense |
| `dobot_demo` | Scripts de demostración |
| `dobot_rviz` | Configuraciones RViz |
| `servo_action` | Acciones de servo |
| `gz_conveyorbelt` | Plugin banda transportadora (Gazebo Sim 8) |
| `pruebas_de_vision` | Detección YOLOv8-OBB |

## 📁 Estructura de Archivos

```
docker/
├── Dockerfile           # Multi-stage build completo
├── docker-compose.yml   # Orquestación de 5 servicios
├── entrypoint.sh        # Script de inicio
├── requirements.txt     # Dependencias Python YOLO
└── README.md            # Esta documentación
```

## 🚀 Inicio Rápido

### Prerrequisitos

1. **Docker** >= 24.0
2. **Docker Compose** >= 2.20
3. **X11** (para GUI de Gazebo/RViz/OpenCV)
4. **Recursos**: Mínimo 16GB RAM, 8 CPU cores

```bash
# Verificar versiones
docker --version
docker compose version

# Permitir X11 forwarding
xhost +local:docker
```

### Construir Imagen

```bash
# Desde la raíz del workspace
cd ~/dobot_ws

# Construir imagen (puede tomar 20-30 min)
docker compose -f docker/docker-compose.yml build

# O con output detallado
docker compose -f docker/docker-compose.yml build --progress=plain
```

### Ejecutar Sistema Completo

```bash
# Iniciar todos los servicios
docker compose -f docker/docker-compose.yml up -d

# Ver logs de todos los servicios
docker compose -f docker/docker-compose.yml logs -f

# Ver logs de un servicio específico
docker compose -f docker/docker-compose.yml logs -f gz_sim
docker compose -f docker/docker-compose.yml logs -f yolo_detector
docker compose -f docker/docker-compose.yml logs -f moveit
```

### Detener Sistema

```bash
# Detener servicios
docker compose -f docker/docker-compose.yml down

# Detener y eliminar volúmenes
docker compose -f docker/docker-compose.yml down -v
```

## 📦 Servicios Definidos

| Servicio | Descripción | Puerto | Recursos |
|----------|-------------|--------|----------|
| `gz_sim` | Simulación Gazebo Sim 8 | 11345 | 6 CPU, 12GB RAM |
| `yolo_detector` | Detección YOLOv8-OBB | - | 4 CPU, 8GB RAM |
| `ros2_bridge` | Bridge ROS2 ↔ Gazebo | - | 1 CPU, 1GB RAM |
| `moveit` | Planificación MoveIt2 | - | 4 CPU, 8GB RAM |
| `robot_state_publisher` | Estado del robot | - | 1 CPU, 1GB RAM |

## 🔧 Uso Individual de Servicios

### Solo Gazebo (Simulación)

```bash
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  dobot-cr20v-complete:latest \
  gz sim -r /ros2_ws/install/gz_conveyorbelt/share/gz_conveyorbelt/worlds/conveyor_demo.sdf
```

### Solo MoveIt2 (Demo)

```bash
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  dobot-cr20v-complete:latest \
  ros2 launch cr20_moveit demo.launch.py
```

### Solo YOLO Detector

```bash
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/dobot_ws/runs:/ros2_ws/runs:ro \
  dobot-cr20v-complete:latest \
  python3 /ros2_ws/src/pruebas_de_vision/pruebas_de_vision/yolo_detector.py \
    --model /ros2_ws/runs/obb/kartonger_yolov8l/weights/best.pt \
    --confidence 0.5
```

### Shell Interactivo

```bash
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  dobot-cr20v-complete:latest \
  bash
```

## 🔑 Variables de Entorno

### Globales
| Variable | Default | Descripción |
|----------|---------|-------------|
| `ROS_DOMAIN_ID` | 0 | Dominio ROS2 DDS |
| `RMW_IMPLEMENTATION` | rmw_cyclonedds_cpp | Middleware DDS |
| `DISPLAY` | :0 | Display X11 |
| `GZ_SIM_RESOURCE_PATH` | ... | Paths a mundos Gazebo |
| `GZ_PLUGIN_PATH` | ... | Paths a plugins |

### YOLO Detector
| Variable | Default | Descripción |
|----------|---------|-------------|
| `MODEL_PATH` | runs/.../best.pt | Ruta al modelo entrenado |
| `CONFIDENCE_THRESHOLD` | 0.5 | Umbral de confianza |
| `IMAGE_TOPIC` | /camera/color/image_raw | Topic de imagen |
| `SHOW_WINDOW` | false | Mostrar ventana OpenCV |

## 📂 Volúmenes y Mounts

| Host Path | Container Path | Propósito |
|-----------|----------------|-----------|
| `./runs` | `/ros2_ws/runs` | Modelos entrenados |
| `./kartonger-1` | `/ros2_ws/data/kartonger` | Dataset |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 forwarding |

## 🛠️ Troubleshooting

### Error: "cannot connect to X server"

```bash
# Permitir conexiones locales
xhost +local:docker

# Verificar DISPLAY
echo $DISPLAY
```

### Error: "ROS2 topic list fails"

```bash
# Verificar que el daemon está corriendo
docker exec -it gz_sim bash
ros2 daemon start
ros2 topic list
```

### Error: "Model not found"

```bash
# Verificar que el modelo existe
ls -la ~/dobot_ws/runs/obb/kartonger_yolov8l/weights/best.pt

# Si no existe, entrenar primero
docker run -it --rm \
  -v ~/dobot_ws/runs:/ros2_ws/runs \
  -v ~/dobot_ws/kartonger-1:/ros2_ws/data/kartonger \
  dobot-cr20v-complete:latest \
  python3 /ros2_ws/src/pruebas_de_vision/pruebas_de_vision/train_yolo.py
```

### Error: "MoveIt configuration not found"

```bash
# Verificar que cr20_moveit está compilado
docker exec -it moveit ls -la /ros2_ws/install/cr20_moveit/
```

### Build lento o fallido

```bash
# Limpiar cache de Docker
docker builder prune -a

# Rebuild sin cache
docker compose -f docker/docker-compose.yml build --no-cache
```

### Error: "Out of memory"

```bash
# Aumentar recursos en docker-compose.yml
# O limitar servicios activos
docker compose -f docker/docker-compose.yml up -d gz_sim yolo_detector
```

## 📊 Monitoreo

### Ver recursos

```bash
docker stats gz_sim yolo_detector moveit ros2_bridge
```

### Ver topics ROS2

```bash
docker exec -it gz_sim ros2 topic list
docker exec -it gz_sim ros2 topic hz /vision/detections/poses
docker exec -it gz_sim ros2 topic echo /joint_states
```

### Ver servicios disponibles

```bash
docker exec -it gz_sim ros2 service list | grep dobot
```

## 🔄 Desarrollo

### Rebuild con cambios de código

```bash
# Si cambiaste código fuente
docker compose -f docker/docker-compose.yml build --no-cache

# Si solo cambiaste launch files o configs (symlink-install)
docker compose -f docker/docker-compose.yml restart
```

### Debugging

```bash
# Entrar al contenedor con entorno completo
docker exec -it gz_sim bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
source /opt/yolo_venv/bin/activate

# Verificar paquetes
ros2 pkg list | grep dobot
ros2 pkg list | grep cr20
```

## 🚀 Comandos de Despliegue Completo

```bash
# 1. Permitir X11
xhost +local:docker

# 2. Construir imagen
cd ~/dobot_ws
docker compose -f docker/docker-compose.yml build

# 3. Iniciar servicios
docker compose -f docker/docker-compose.yml up -d

# 4. Verificar estado
docker compose -f docker/docker-compose.yml ps
docker compose -f docker/docker-compose.yml logs -f

# 5. Detener
docker compose -f docker/docker-compose.yml down
```

## 📝 Notas Importantes

1. **Network Mode**: Se usa `host` para permitir DDS discovery entre contenedores
2. **X11**: Requiere `xhost +local:docker` para GUI
3. **Modelos**: Deben estar en `runs/` antes de iniciar
4. **Dataset**: Debe estar en `kartonger-1/` para entrenamiento
5. **Recursos**: El sistema completo requiere ~30GB RAM y 16 CPU cores para óptimo rendimiento
6. **Build Time**: La primera construcción puede tomar 20-30 minutos

## 🔐 Seguridad

- Usuario no-root (`rosuser`) por defecto
- Sin privilegios excesivos
- Solo los mounts necesarios

## 📚 Documentación Adicional

- [AGENTS.md](../AGENTS.md) - Reglas del proyecto
- [README.md](../README.md) - Documentación general
- [plans/docker_plan.md](../plans/docker_plan.md) - Plan de contenedorización
