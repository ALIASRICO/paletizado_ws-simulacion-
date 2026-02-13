# 🤖 Sistema de Paletizado - DOBOT CR20V

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange)](https://ubuntu.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)

Sistema de paletizado automatizado para el cobot industrial **DOBOT CR20V** utilizando ROS 2 Jazzy.

---

## 📋 Índice

- [Descripción](#-descripción)
- [Características](#-características)
- [Requisitos](#-requisitos)
- [Instalación](#-instalación)
- [Estructura del Proyecto](#-estructura-del-proyecto)
- [Uso](#-uso)
- [Documentación](#-documentación)
- [Contribución](#-contribución)
- [Licencia](#-licencia)

---

## 📖 Descripción

Este proyecto implementa un sistema de paletizado profesional para el robot DOBOT CR20V (6 DOF, 20kg payload) utilizando:

- **ROS 2 Jazzy** - Framework de robótica
- **MoveIt 2** - Planificación de movimientos
- **Gazebo** - Simulación
- **OpenCV + AprilTag** - Visión por computadora
- **Dobot SDK V4** - Control del robot via TCP/IP

---

## ✨ Características

- ✅ Arquitectura de tres capas desacoplada (Percepción → Coordinator → Driver)
- ✅ Planificación de movimientos segura con validación de límites
- ✅ Detección de objetos mediante visión (RealSense D435i + AprilTag)
- ✅ Simulación completa en Gazebo
- ✅ Configuración modular y parametizable
- ✅ Safety checks críticos antes de operaciones

---

## 📦 Requisitos

| Componente | Versión/Requisito |
|------------|-------------------|
| OS         | Ubuntu 24.04     |
| ROS 2      | Jazzy            |
| Python     | 3.12+            |
| C++        | C++17 mínimo     |
| RAM        | 8GB mínimo       |
| GPU        | Recomendado para Gazebo |

### Dependencias

```bash
# Instalar dependencias principales
sudo apt install ros-jazzy-desktop-full
sudo apt install ros-jazzy-moveit
sudo apt install ros-jazzy-realsense2-camera
sudo apt install ros-jazzy-apriltag-ros

# Dependencias Python
pip install apriltag numpy opencv-python
```

---

## 🐳 Ejecución con Docker (Recomendado)

La forma más sencilla de ejecutar el proyecto completo es usando Docker:

```bash
# 1. Permitir X11 (para GUI de Gazebo/RViz)
xhost +local:docker

# 2. Construir y ejecutar todo el sistema
docker compose -f docker/docker-compose.yml up --build
```

Esto levanta automáticamente:
- **Gazebo Sim 8** con banda transportadora y detección de cajas
- **YOLOv8-OBB** para detección de cajas de cartón
- **MoveIt2** para planificación de movimientos
- **Bridge ROS2 ↔ Gazebo** para comunicación
- **Robot State Publisher** para el modelo URDF

> 📖 Ver [docker/README.md](docker/README.md) para documentación completa.

---

## 🚀 Instalación (Desarrollo Local)

```bash
# 1. Clonar el repositorio
cd ~/dobot_ws/src
git clone https://github.com/ALIASRICO/paletizado_ws-simulacion-.git

# 2. Instalar dependencias
cd ~/dobot_ws
rosdep install --from-paths src --ignore-src -r

# 3. Compilar
colcon build --symlink-install

# 4. Fuente el entorno
source install/setup.bash
```

---

## 📁 Estructura del Proyecto

```
dobot_ws/
├── .github/              # Configuración GitHub Actions
├── .gitignore            # Exclusiones de Git
├── README.md             # Este archivo
├── src/
│   └── DOBOT_6Axis_ROS2_V4/
│       ├── cr20_moveit/      # Configuración MoveIt2 para CR20V
│       ├── dobot_bringup_v4/ # Driver TCP/IP para robot real
│       ├── dobot_msgs_v4/    # Mensajes y servicios ROS2
│       ├── dobot_vision/    # Nodos de visión (AprilTag, RealSense)
│       ├── dobot_moveit/    # Integración MoveIt
│       ├── dobot_gazebo/    # Simulación Gazebo
│       ├── cra_description/  # Modelos URDF/XACRO
│       ├── servo_action/    # Cliente de acciones
│       └── dobot_demo/      # Demostraciones
├── build/                # Archivos de compilación (NO commit)
├── install/              # Archivos de instalación (NO commit)
└── log/                  # Logs (NO commit)
```

---

## 🎮 Uso

### Simulación con RViz

```bash
ros2 launch cr20_moveit demo.launch.py
```

### Simulación con Gazebo

```bash
ros2 launch dobot_gazebo palletizing_sim.launch.py
```

### Visión (Cámara Real)

```bash
ros2 launch dobot_vision vision_real.launch.py
```

### Robot Real

```bash
# Conectar al robot
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py

# Verificar conexión
ros2 service call /dobot_bringup_v4/srv/GetPose dobot_msgs_v4/srv/GetPose
```

---

## 📚 Documentación

- **[AGENTS.md](AGENTS.md)** - Reglas y guías del proyecto
- **[docs/ejecutar_procesos.txt](docs/ejecutar_procesos.txt)** - Procesos de ejecución
- **[V4新增指令/](V4新增指令/)** - Documentación SDK Dobot V4

---

## 🤝 Contribución

1. Fork el repositorio
2. Crear una rama feature (`git checkout -b feature/AmazingFeature`)
3. Commit los cambios (`git commit -m 'Add: AmazingFeature'`)
4. Push a la rama (`git push origin feature/AmazingFeature`)
5. Abrir un **Pull Request**

---

## ⚠️ Safety First

> **IMPORTANTE**: Este proyecto controla un robot industrial de 20kg.
> - SIEMPRE validar límites de movimiento antes de ejecutar
> - NUNCA operar sin supervisión en el robot real
> - Usar SpeedFactor conservador (10-20%) inicialmente
> - Mantener parada de emergencia accesible

---

## 📄 Licencia

Este proyecto está bajo la Licencia MIT. Ver [LICENSE](LICENSE) para más detalles.

---

## 👨‍💻 Autor

**IUDC** - [@ALIASRICO](https://github.com/ALIASRICO)

---

⌨️ Hecho con ❤️ para la comunidad de robótica
