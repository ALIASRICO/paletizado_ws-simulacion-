#!/bin/bash
# Script para ejecutar el detector YOLO con ROS2 usando el entorno virtual aislado

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

VENV_PATH="$HOME/dobot_ws/yolo_venv"
WORKSPACE="$HOME/dobot_ws"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Detector YOLOv8-OBB con ROS2        ${NC}"
echo -e "${GREEN}========================================${NC}"

# Verificar que el entorno virtual existe
if [ ! -d "$VENV_PATH" ]; then
    echo -e "${RED}❌ Error: Entorno virtual no encontrado en $VENV_PATH${NC}"
    exit 1
fi

# Source ROS2 workspace
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
    echo -e "${GREEN}✅ ROS2 workspace sourced${NC}"
else
    echo -e "${RED}❌ Error: ROS2 workspace no compilado${NC}"
    echo -e "${YELLOW}   Ejecuta: colcon build --symlink-install${NC}"
    exit 1
fi

# Configurar PYTHONPATH para incluir paquetes ROS2
export PYTHONPATH="$WORKSPACE/install/pruebas_de_vision/lib/python3.12/site-packages:$PYTHONPATH"

# Argumentos por defecto
MODEL_PATH="${1:-$WORKSPACE/runs/obb/kartonger_yolov8l/weights/best.pt}"
CONFIDENCE="${2:-0.5}"

echo -e "${YELLOW}📦 Usando entorno virtual: $VENV_PATH${NC}"
echo -e "${YELLOW}🎯 Modelo: $MODEL_PATH${NC}"
echo -e "${YELLOW}📊 Confianza mínima: $CONFIDENCE${NC}"
echo ""

# Verificar si el modelo existe, si no, usar el pre-entrenado
if [ ! -f "$MODEL_PATH" ]; then
    echo -e "${YELLOW}⚠️  Modelo no encontrado, usando pre-entrenado yolov8n-obb.pt${NC}"
    MODEL_PATH="yolov8n-obb.pt"
fi

# Ejecutar el nodo con el Python del entorno virtual
$VENV_PATH/bin/python3 -c "
import sys
sys.path.insert(0, '$WORKSPACE/src/pruebas_de_vision')
from pruebas_de_vision.yolo_detector import main
import os
os.environ['MODEL_PATH'] = '$MODEL_PATH'
os.environ['CONFIDENCE_THRESHOLD'] = '$CONFIDENCE'
main()
"
