#!/bin/bash
# Script para entrenar el modelo YOLO usando el entorno virtual aislado

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

VENV_PATH="$HOME/dobot_ws/yolo_venv"
SCRIPT_PATH="$HOME/dobot_ws/src/pruebas_de_vision/pruebas_de_vision/train_yolo.py"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Entrenamiento YOLOv8-OBB - Kartonger ${NC}"
echo -e "${GREEN}========================================${NC}"

# Verificar que el entorno virtual existe
if [ ! -d "$VENV_PATH" ]; then
    echo -e "${RED}❌ Error: Entorno virtual no encontrado en $VENV_PATH${NC}"
    echo -e "${YELLOW}   Ejecuta primero: python3 -m venv ~/dobot_ws/yolo_venv${NC}"
    exit 1
fi

# Verificar que el script existe
if [ ! -f "$SCRIPT_PATH" ]; then
    echo -e "${RED}❌ Error: Script de entrenamiento no encontrado${NC}"
    exit 1
fi

echo -e "${YELLOW}📦 Usando entorno virtual: $VENV_PATH${NC}"
echo -e "${YELLOW}🚀 Iniciando entrenamiento...${NC}"
echo ""

# Ejecutar el script con el Python del entorno virtual
$VENV_PATH/bin/python3 $SCRIPT_PATH "$@"
