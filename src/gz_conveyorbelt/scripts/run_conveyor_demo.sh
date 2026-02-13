#!/bin/bash
# Script para ejecutar la simulación de cinta transportadora con Gazebo Sim 8

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Conveyor Belt Demo - Gazebo Sim 8 ===${NC}"

# Source ROS2
source /opt/ros/jazzy/setup.bash
cd ~/dobot_ws
source install/setup.bash

# Set plugin path
export GZ_PLUGIN_PATH=/home/iudc/dobot_ws/install/gz_conveyorbelt/lib:$GZ_PLUGIN_PATH

echo -e "${YELLOW}Iniciando Gazebo Sim...${NC}"
echo -e "Plugin path: $GZ_PLUGIN_PATH"

# Run Gazebo in background
gz sim -r install/gz_conveyorbelt/share/gz_conveyorbelt/worlds/conveyor_demo.sdf &
GZ_PID=$!

# Wait for Gazebo to start
sleep 8

echo ""
echo -e "${GREEN}=== Simulación iniciada ===${NC}"
echo ""
echo "Modelos disponibles:"
gz model --list 2>/dev/null

echo ""
echo -e "${YELLOW}Comandos disponibles:${NC}"
echo "  - Iniciar cinta (50%):  gz topic -t /conveyor/power -m gz.msgs.Double -p 'data: 50.0'"
echo "  - Iniciar cinta (80%):  gz topic -t /conveyor/power -m gz.msgs.Double -p 'data: 80.0'"
echo "  - Detener cinta:        gz topic -t /conveyor/power -m gz.msgs.Double -p 'data: 0.0'"
echo ""
echo -e "${YELLOW}Enviando comando inicial (50%)...${NC}"
gz topic -t /conveyor/power -m gz.msgs.Double -p 'data: 50.0'

echo ""
echo -e "${GREEN}Presiona Ctrl+C para detener la simulación${NC}"

# Wait for Gazebo
wait $GZ_PID
