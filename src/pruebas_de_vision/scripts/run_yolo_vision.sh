#!/bin/bash
#
# Script unificado para lanzar todo el sistema de visión YOLO
# Incluye: Gazebo + Cámara + Detector YOLO
#
# Uso:
#   ./run_yolo_vision.sh                    # Modo simulación estática
#   ./run_yolo_vision.sh --dynamic          # Modo cajas dinámicas
#   ./run_yolo_vision.sh --real             # Modo cámara real (RealSense)
#

set -e

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Rutas
WORKSPACE="$HOME/dobot_ws"
VENV_PYTHON="$WORKSPACE/yolo_venv/bin/python3"
YOLO_DETECTOR="$WORKSPACE/src/pruebas_de_vision/pruebas_de_vision/yolo_detector.py"
MODEL_PATH="$HOME/runs/obb/runs/obb/kartonger_yolov8l/weights/best.pt"

# Parámetros por defecto
CONFIDENCE="0.5"
MODE="static"

# Parsear argumentos
while [[ $# -gt 0 ]]; do
    case $1 in
        --dynamic|-d)
            MODE="dynamic"
            shift
            ;;
        --real|-r)
            MODE="real"
            shift
            ;;
        --confidence|-c)
            CONFIDENCE="$2"
            shift 2
            ;;
        --help|-h)
            echo "Uso: $0 [opciones]"
            echo ""
            echo "Opciones:"
            echo "  --dynamic, -d       Usar cajas dinámicas en simulación"
            echo "  --real, -r          Usar cámara RealSense física"
            echo "  --confidence, -c N  Umbral de confianza (default: 0.5)"
            echo "  --help, -h          Mostrar esta ayuda"
            echo ""
            echo "Ejemplos:"
            echo "  $0                      # Simulación estática"
            echo "  $0 --dynamic            # Simulación con cajas dinámicas"
            echo "  $0 --real               # Cámara RealSense"
            echo "  $0 -d -c 0.7            # Dinámico con confianza 0.7"
            exit 0
            ;;
        *)
            echo -e "${RED}Opción desconocida: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}     Sistema de Visión YOLO - Dobot CR20V${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo ""

# Verificar que el workspace existe
if [ ! -d "$WORKSPACE" ]; then
    echo -e "${RED}Error: Workspace no encontrado en $WORKSPACE${NC}"
    exit 1
fi

# Verificar que el virtual environment existe
if [ ! -f "$VENV_PYTHON" ]; then
    echo -e "${RED}Error: Virtual environment no encontrado en $VENV_PYTHON${NC}"
    echo -e "${YELLOW}Ejecuta primero: python3 -m venv ~/dobot_ws/yolo_venv && source ~/dobot_ws/yolo_venv/bin/activate && pip install ultralytics opencv-python${NC}"
    exit 1
fi

# Verificar que el modelo existe
if [ ! -f "$MODEL_PATH" ]; then
    echo -e "${RED}Error: Modelo no encontrado en $MODEL_PATH${NC}"
    exit 1
fi

# Source del workspace
echo -e "${GREEN}[1/3] Sourcing workspace...${NC}"
source "$WORKSPACE/install/setup.bash"

# Función para limpiar procesos al salir
cleanup() {
    echo ""
    echo -e "${YELLOW}Deteniendo todos los procesos...${NC}"
    jobs -p | xargs -r kill 2>/dev/null || true
    # Matar cualquier proceso gz sim restante
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ruby" 2>/dev/null || true  # Gazebo usa ruby
    echo -e "${GREEN}Procesos detenidos.${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Lanzar según el modo
case $MODE in
    static)
        echo -e "${GREEN}[2/3] Lanzando Gazebo con simulación estática...${NC}"
        ros2 launch pruebas_de_vision camera_test.launch.py &
        LAUNCH_PID=$!
        ;;
    dynamic)
        echo -e "${GREEN}[2/3] Lanzando Gazebo con cajas dinámicas...${NC}"
        ros2 launch pruebas_de_vision dynamic_boxes.launch.py &
        LAUNCH_PID=$!
        ;;
    real)
        echo -e "${GREEN}[2/3] Configurando para cámara RealSense...${NC}"
        # Verificar que la cámara está conectada
        if ! lsusb | grep -q "Intel"; then
            echo -e "${YELLOW}Advertencia: No se detectó cámara Intel RealSense.${NC}"
            echo -e "${YELLOW}Continuando de todas formas...${NC}"
        fi
        # Lanzar solo el nodo de cámara RealSense si existe
        if ros2 pkg list | grep -q "realsense2_camera"; then
            ros2 launch realsense2_camera rs_launch.py &
            LAUNCH_PID=$!
        else
            echo -e "${YELLOW}Paquete realsense2_camera no encontrado.${NC}"
            echo -e "${YELLOW}Asegúrate de tener el driver de RealSense instalado.${NC}"
            LAUNCH_PID=""
        fi
        ;;
esac

# Esperar a que Gazebo/cámara esté listo
echo -e "${YELLOW}Esperando a que el sistema esté listo...${NC}"
sleep 10

# Verificar que el topic de cámara existe
echo -e "${GREEN}Verificando topics de cámara...${NC}"
if ! ros2 topic list | grep -q "camera"; then
    echo -e "${RED}Error: No se detectaron topics de cámara después de 10 segundos.${NC}"
    cleanup
    exit 1
fi

echo -e "${GREEN}Topics disponibles:${NC}"
ros2 topic list | grep -E "camera|vision" || true

# Lanzar detector YOLO
echo ""
echo -e "${GREEN}[3/3] Lanzando detector YOLO...${NC}"
echo -e "${BLUE}  Modelo: $MODEL_PATH${NC}"
echo -e "${BLUE}  Confianza: $CONFIDENCE${NC}"
echo ""
echo -e "${YELLOW}═══════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}  Sistema activo. Presiona Ctrl+C para detener.${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════════════════${NC}"
echo ""

# Ejecutar detector YOLO
"$VENV_PYTHON" "$YOLO_DETECTOR" --model "$MODEL_PATH" --confidence "$CONFIDENCE"

# Si el detector termina, limpiar
cleanup
