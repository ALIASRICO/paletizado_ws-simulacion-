#!/bin/bash
# =============================================================================
# Entrypoint para contenedor ROS2 + YOLO + Gazebo
# =============================================================================
# Este script configura el entorno antes de ejecutar el comando principal.

set -e

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}  ROS2 Jazzy + YOLO + Gazebo Sim 8${NC}"
echo -e "${BLUE}=============================================${NC}"

# Source ROS2
echo -e "${YELLOW}[INFO] Sourcing ROS2 Jazzy...${NC}"
source /opt/ros/jazzy/setup.bash

# Source workspace
if [ -f "/ros2_ws/install/setup.bash" ]; then
    echo -e "${YELLOW}[INFO] Sourcing workspace...${NC}"
    source /ros2_ws/install/setup.bash
fi

# Activar entorno virtual de YOLO
if [ -d "/opt/yolo_venv" ]; then
    echo -e "${YELLOW}[INFO] Activating YOLO virtual environment...${NC}"
    source /opt/yolo_venv/bin/activate
fi

# Configurar variables de entorno
export GZ_SIM_RESOURCE_PATH="/ros2_ws/src/gz_conveyorbelt/worlds:/ros2_ws/worlds:${GZ_SIM_RESOURCE_PATH}"
export GZ_PLUGIN_PATH="/ros2_ws/install/gz_conveyorbelt/lib:${GZ_PLUGIN_PATH}"

# Verificar que ROS2 está funcionando
echo -e "${YELLOW}[INFO] Checking ROS2...${NC}"
if ros2 topic list > /dev/null 2>&1; then
    echo -e "${GREEN}[OK] ROS2 is ready${NC}"
else
    echo -e "${YELLOW}[WARN] ROS2 daemon not running (this is normal for new containers)${NC}"
fi

# Verificar entorno virtual
if command -v yolo &> /dev/null; then
    echo -e "${GREEN}[OK] YOLOv8 available: $(yolo version 2>/dev/null || echo 'installed')${NC}"
fi

# Mostrar información del sistema
echo ""
echo -e "${BLUE}Environment:${NC}"
echo -e "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo -e "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
echo -e "  GZ_SIM_RESOURCE_PATH: ${GZ_SIM_RESOURCE_PATH}"
echo ""
echo -e "${BLUE}Available commands:${NC}"
echo -e "  ${GREEN}gz sim -r <world_file>${NC}     - Launch Gazebo simulation"
echo -e "  ${GREEN}ros2 launch <package> <file>${NC} - Launch ROS2 nodes"
echo -e "  ${GREEN}yolo_detector${NC}               - Run YOLO detection node"
echo -e "  ${GREEN}train_yolo${NC}                  - Train YOLO model"
echo ""

# Manejar señales para cleanup
trap 'echo -e "${YELLOW}[INFO] Received shutdown signal...${NC}"; exit 0' SIGTERM SIGINT

# Ejecutar comando pasado al contenedor
if [ $# -eq 0 ]; then
    echo -e "${GREEN}[INFO] Starting interactive shell...${NC}"
    exec bash
else
    echo -e "${GREEN}[INFO] Executing: $@${NC}"
    exec "$@"
fi
