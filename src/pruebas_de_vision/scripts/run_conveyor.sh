#!/bin/bash
# ============================================================
# Script unificado para ejecutar simulación de cinta transportadora
# con detección YOLO de cajas de cartón
# ============================================================

set -e

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Rutas
WORKSPACE_DIR="$HOME/dobot_ws"
VENV_DIR="$WORKSPACE_DIR/yolo_venv"
VENV_PYTHON="$VENV_DIR/bin/python"
PKG_DIR="$WORKSPACE_DIR/src/pruebas_de_vision"
YOLO_DETECTOR="$PKG_DIR/pruebas_de_vision/yolo_detector.py"
MODEL_PATH="/home/iudc/runs/obb/runs/obb/kartonger_yolov8l/weights/best.pt"
WORLD_FILE="$PKG_DIR/worlds/conveyor_belt.sdf"

# Parámetros por defecto
CONFIDENCE="0.5"
BELT_SPEED="0.1"
NUM_BOXES="5"

# Funciones
log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[OK]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Mostrar uso
usage() {
    echo "Uso: $0 [opciones]"
    echo ""
    echo "Opciones:"
    echo "  --speed VALOR       Velocidad de la cinta (m/s, default: 0.1)"
    echo "  --boxes VALOR       Número de cajas (default: 5)"
    echo "  --confidence VALOR  Confianza mínima YOLO (default: 0.5)"
    echo "  --help              Mostrar esta ayuda"
    echo ""
    echo "Ejemplo:"
    echo "  $0 --speed 0.15 --boxes 8"
    exit 0
}

# Parsear argumentos
while [[ $# -gt 0 ]]; do
    case $1 in
        --speed)
            BELT_SPEED="$2"
            shift 2
            ;;
        --boxes)
            NUM_BOXES="$2"
            shift 2
            ;;
        --confidence)
            CONFIDENCE="$2"
            shift 2
            ;;
        --help|-h)
            usage
            ;;
        *)
            log_error "Opción desconocida: $1"
            usage
            ;;
    esac
done

log_info "=========================================="
log_info "  CINTA TRANSPORTADORA + YOLO DETECTOR"
log_info "=========================================="
echo ""

# Verificar que el workspace existe
if [ ! -d "$WORKSPACE_DIR" ]; then
    log_error "Workspace no encontrado: $WORKSPACE_DIR"
    exit 1
fi

cd "$WORKSPACE_DIR"

# Verificar que el virtual environment existe
if [ ! -f "$VENV_PYTHON" ]; then
    log_error "Virtual environment no encontrado: $VENV_PYTHON"
    log_info "Ejecuta primero: python3 -m venv yolo_venv && source yolo_venv/bin/activate && pip install ultralytics opencv-python numpy"
    exit 1
fi

# Verificar que el modelo existe
if [ ! -f "$MODEL_PATH" ]; then
    log_error "Modelo YOLO no encontrado: $MODEL_PATH"
    exit 1
fi

# Verificar que el mundo existe
if [ ! -f "$WORLD_FILE" ]; then
    log_error "Mundo SDF no encontrado: $WORLD_FILE"
    exit 1
fi

# Source del workspace
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

log_success "Verificaciones completadas"
log_info "Parámetros:"
log_info "  - Velocidad cinta: $BELT_SPEED m/s"
log_info "  - Número de cajas: $NUM_BOXES"
log_info "  - Confianza YOLO: $CONFIDENCE"
echo ""

# Función para limpiar al salir
cleanup() {
    log_warn "Deteniendo procesos..."
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "yolo_detector" 2>/dev/null || true
    pkill -f "conveyor_controller" 2>/dev/null || true
    pkill -f "ros_gz_bridge" 2>/dev/null || true
    pkill -f "parameter_bridge" 2>/dev/null || true
    log_success "Procesos detenidos"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 1. Lanzar Gazebo con el mundo de la cinta transportadora
log_info "Iniciando Gazebo con cinta transportadora..."
gz sim -r "$WORLD_FILE" &
GZ_PID=$!
sleep 10

# 2. Lanzar bridge para la cámara (formato correcto para Gazebo Sim)
log_info "Iniciando bridge ROS2-Gazebo para cámara..."
ros2 run ros_gz_bridge parameter_bridge \
    /camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image &
BRIDGE_PID=$!
sleep 2

# 3. Lanzar controlador de la cinta transportadora
log_info "Iniciando controlador de cinta transportadora..."
ros2 run pruebas_de_vision conveyor_controller \
    --ros-args \
    -p world_name:=conveyor_belt \
    -p belt_speed:="$BELT_SPEED" \
    -p num_boxes:="$NUM_BOXES" &
CONTROLLER_PID=$!
sleep 2

# 4. Lanzar detector YOLO
log_info "Iniciando detector YOLO..."
"$VENV_PYTHON" "$YOLO_DETECTOR" \
    --model "$MODEL_PATH" \
    --confidence "$CONFIDENCE" \
    --image-topic /camera/color/image_raw &
YOLO_PID=$!
sleep 3

log_success "=========================================="
log_success "  Sistema iniciado correctamente!"
log_success "=========================================="
echo ""
log_info "Topics disponibles:"
log_info "  - /camera/color/image_raw (imagen de Gazebo)"
log_info "  - /vision/detections/image (imagen con detecciones)"
log_info "  - /vision/detections/bboxes (bounding boxes)"
echo ""
log_info "Presiona Ctrl+C para detener todo"
echo ""

# Esperar a que terminen los procesos
wait $YOLO_PID 2>/dev/null || true

# Si YOLO termina, limpiar todo
cleanup
