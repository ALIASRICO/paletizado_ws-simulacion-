#!/bin/bash
# =============================================================================
# Script de Compilación - Workspace dobot_ws para ROS2 Jazzy
# =============================================================================
#
# Uso:
#   ./build_workspace.sh              # Compilación normal
#   ./build_workspace.sh clean        # Limpiar antes de compilar
#   ./build_workspace.sh all          # Compilar todo
#
# =============================================================================

# Colores para输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# =============================================================================
# CONFIGURACIÓN
# =============================================================================

# Directorio del workspace
WORKSPACE_DIR="$HOME/dobot_ws"
LOG_FILE="$WORKSPACE_DIR/build.log"

# Paquetes a compilar (en orden de dependencias)
PACKAGES=(
    "cra_description"
    "dobot_rviz"
    "dobot_msgs_v4"
    "dobot_moveit"
    "cr20_moveit"
)

# =============================================================================
# FUNCIONES
# =============================================================================

print_header() {
    echo -e "\n${BLUE}============================================================${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}============================================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# =============================================================================
# VERIFICACIONES INICIALES
# =============================================================================

check_workspace() {
    print_header "Verificando Workspace"
    
    if [ ! -d "$WORKSPACE_DIR" ]; then
        print_error "Workspace no encontrado en: $WORKSPACE_DIR"
        exit 1
    fi
    
    if [ ! -d "$WORKSPACE_DIR/src" ]; then
        print_error "Directorio src/ no encontrado"
        exit 1
    fi
    
    print_success "Workspace verificado: $WORKSPACE_DIR"
}

check_ros() {
    print_header "Verificando ROS2 Jazzy"
    
    # Verificar si ROS2 está instalado
    if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
        print_error "ROS2 Jazzy no encontrado en /opt/ros/jazzy/"
        print_info "Instala ROS2 Jazzy primero"
        exit 1
    fi
    
    print_success "ROS2 Jazzy encontrado"
    
    # Verificar colcon
    if ! command -v colcon &> /dev/null; then
        print_error "colcon no está instalado"
        print_info "Instala colcon: sudo apt install python3-colcon-common-extensions"
        exit 1
    fi
    
    print_success "colcon disponible"
}

clean_build() {
    print_warning "Limpiando build anterior..."
    
    # Limpiar directorios
    cd "$WORKSPACE_DIR"
    
    if [ -d "build" ]; then
        rm -rf build
        print_info "Directorio build/ eliminado"
    fi
    
    if [ -d "install" ]; then
        rm -rf install
        print_info "Directorio install/ eliminado"
    fi
    
    if [ -d "log" ]; then
        rm -rf log
        print_info "Directorio log/ eliminado"
    fi
    
    print_success "Limpieza completada"
}

# =============================================================================
# COMPILACIÓN
# =============================================================================

build_packages() {
    print_header "Compilando Paquetes"
    
    cd "$WORKSPACE_DIR"
    
    # Source ROS2
    echo "Sourceando ROS2 Jazzy..."
    source /opt/ros/jazzy/setup.bash
    
    # Mostrar paquetes a compilar
    echo -e "\n${BLUE}Paquetes a compilar:${NC}"
    for pkg in "${PACKAGES[@]}"; do
        echo "  - $pkg"
    done
    echo ""
    
    # Comando de compilación
    BUILD_CMD="colcon build"
    
    # Compilar paquetes específicos en orden
    BUILD_CMD="$BUILD_CMD --packages-select ${PACKAGES[*]}"
    
    # Agregar opciones
    BUILD_CMD="$BUILD_CMD --cmake-args -DCMAKE_BUILD_TYPE=Release"
    
    # Ejecutar compilación
    print_info "Ejecutando: $BUILD_CMD"
    echo ""
    
    # Redirigir salida a archivo de log
    $BUILD_CMD 2>&1 | tee "$LOG_FILE"
    BUILD_RESULT=${PIPESTATUS[0]}
    
    echo ""
    
    # =============================================================================
    # VERIFICACIÓN DE RESULTADO
    # =============================================================================
    
    if [ $BUILD_RESULT -eq 0 ]; then
        print_success "Compilación completada exitosamente"
        echo ""
        print_info "Logs guardados en: $LOG_FILE"
        
        # Mostrar resumen
        echo ""
        print_header "Resumen"
        
        for pkg in "${PACKAGES[@]}"; do
            if [ -d "$WORKSPACE_DIR/install/$pkg" ]; then
                print_success "$pkg: Compilado"
            else
                print_warning "$pkg: No se encontró en install/"
            fi
        done
        
        # =============================================================================
        # INSTRUCCIONES FINALES
        # =============================================================================
        
        echo ""
        print_header "Compilación Exitosa"
        echo ""
        echo "Para usar el workspace, ejecuta:"
        echo ""
        echo -e "  ${GREEN}source $WORKSPACE_DIR/install/setup.bash${NC}"
        echo ""
        
        return 0
    else
        print_error "Error en la compilación"
        echo ""
        print_info "Revisa los logs en: $LOG_FILE"
        
        return 1
    fi
}

# =============================================================================
# MAIN
# =============================================================================

main() {
    echo -e "${BLUE}"
    echo "============================================================"
    echo "  DOBOT WORKSPACE - Script de Compilación"
    echo "  Versión: ROS2 Jazzy"
    echo "============================================================"
    echo -e "${NC}"
    
    # Verificar workspace
    check_workspace
    
    # Verificar ROS2
    check_ros
    
    # Verificar argumentos
    if [ "$1" == "clean" ]; then
        clean_build
        shift
    fi
    
    # Compilar
    build_packages "$@"
    
    exit $?
}

# Ejecutar main
main "$@"
