#!/bin/bash
# =============================================================================
# Script de Verificación - Instalación dobot_ws
# =============================================================================

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Results file
RESULTS_FILE="$HOME/dobot_ws/test_results.txt"

# Counters
PASSED=0
FAILED=0

print_header() {
    echo -e "\n${BLUE}============================================================${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}============================================================${NC}\n"
    echo "============================================================" >> "$RESULTS_FILE"
    echo "  $1" >> "$RESULTS_FILE"
    echo "============================================================" >> "$RESULTS_FILE"
}

test_pass() {
    echo -e "${GREEN}✓ PASS${NC}: $1"
    echo "✓ PASS: $1" >> "$RESULTS_FILE"
    ((PASSED++))
}

test_fail() {
    echo -e "${RED}✗ FAIL${NC}: $1"
    echo "✗ FAIL: $1" >> "$RESULTS_FILE"
    ((FAILED++))
}

test_warn() {
    echo -e "${YELLOW}⚠ WARN${NC}: $1"
    echo "⚠ WARN: $1" >> "$RESULTS_FILE"
}

test_info() {
    echo -e "${BLUE}ℹ INFO${NC}: $1"
    echo "ℹ INFO: $1" >> "$RESULTS_FILE"
}

main() {
    echo -e "${BLUE}"
    echo "============================================================"
    echo "  DOBOT WORKSPACE - Test de Verificación"
    echo "============================================================"
    echo -e "${NC}"
    
    echo "Test Results - $(date)" > "$RESULTS_FILE"
    echo "" >> "$RESULTS_FILE"
    
    print_header "1. VERIFICACIONES BÁSICAS"
    
    echo "Test 1.1: ROS2 Jazzy instalado"
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        test_pass "ROS2 Jazzy instalado en /opt/ros/jazzy/"
    else
        test_fail "ROS2 Jazzy NO encontrado"
    fi
    
    echo "Test 1.2: Workspace existe"
    if [ -d "$HOME/dobot_ws" ]; then
        test_pass "Workspace existe: $HOME/dobot_ws"
    else
        test_fail "Workspace NO encontrado"
    fi
    
    echo "Test 1.3: Directorio src/"
    if [ -d "$HOME/dobot_ws/src" ]; then
        test_pass "Directorio src/ existe"
    else
        test_fail "Directorio src/ NO encontrado"
    fi
    
    echo "Test 1.4: Directorio install/"
    if [ -d "$HOME/dobot_ws/install" ]; then
        test_pass "Directorio install/ existe"
    else
        test_warn "Directorio install/ NO existe (primero compila)"
    fi
    
    print_header "2. VERIFICACIONES ROS2"
    
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    
    echo "Test 2.1: colcon disponible"
    if command -v colcon &> /dev/null; then
        test_pass "colcon disponible"
    else
        test_fail "colcon NO disponible"
    fi
    
    echo "Test 2.2: Paquetes del workspace"
    if [ -d "$HOME/dobot_ws/install" ]; then
        source $HOME/dobot_ws/install/setup.bash 2>/dev/null
        
        for pkg in dobot_msgs_v4 dobot_moveit cr20_moveit cra_description; do
            if ros2 pkg list 2>/dev/null | grep -q "^$pkg$"; then
                test_info "Paquete $pkg: OK"
            else
                test_warn "Paquete $pkg: NO encontrado"
            fi
        done
        test_pass "Verificación de paquetes completada"
    else
        test_warn "Workspace no compilado - ejecuta build_workspace.sh primero"
    fi
    
    echo "Test 2.3: Comando ros2"
    if command -v ros2 &> /dev/null; then
        test_pass "Comando ros2 disponible"
    else
        test_fail "Comando ros2 NO disponible"
    fi
    
    print_header "3. VERIFICACIONES ISAAC SIM"
    
    echo "Test 3.1: Isaac Sim"
    ISAAC_PATHS=(
        "/opt/nvidia/isaac_sim"
        "/usr/local/isaac_sim"
    )
    
    ISAAC_FOUND=false
    for path in "${ISAAC_PATHS[@]}"; do
        if [ -d "$path" ]; then
            test_pass "Isaac Sim encontrado en: $path"
            ISAAC_FOUND=true
            break
        fi
    done
    
    if [ "$ISAAC_FOUND" = false ]; then
        test_warn "Isaac Sim NO encontrado en ubicaciones estándar"
    fi
    
    echo "Test 3.2: ros_ign_bridge"
    if ros2 pkg list 2>/dev/null | grep -q "ros_ign_bridge"; then
        test_pass "ros_ign_bridge instalado"
    else
        test_warn "ros_ign_bridge NO instalado"
    fi
    
    print_header "4. VERIFICACIONES FINALES"
    
    echo "Test 4.1: Launch files"
    LAUNCH_OK=true
    for lf in "cr20_moveit/launch/demo.launch.py" "cr20_moveit/launch/dobot_moveit.launch.py" "cr20_moveit/launch/isaac_sim.launch.py"; do
        if [ -f "$HOME/dobot_ws/src/DOBOT_6Axis_ROS2_V4/$lf" ]; then
            test_info "Launch file $lf: OK"
        else
            test_warn "Launch file $lf: NO encontrado"
            LAUNCH_OK=false
        fi
    done
    if [ "$LAUNCH_OK" = true ]; then
        test_pass "Launch files principales disponibles"
    fi
    
    echo "Test 4.2: URDF del robot"
    if [ -f "$HOME/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cra_description/urdf/cr20_robot.urdf" ]; then
        test_pass "URDF cr20_robot disponible"
    else
        test_fail "URDF cr20_robot NO encontrado"
    fi
    
    echo "Test 4.3: Scripts Python"
    if [ -f "$HOME/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cr20_moveit/scripts/cr20_isaac_bridge_node.py" ]; then
        test_pass "Nodo bridge Isaac disponible"
    else
        test_warn "Nodo bridge Isaac NO encontrado"
    fi
    
    print_header "RESUMEN DE RESULTADOS"
    
    echo ""
    echo "========================================"
    echo "  TOTAL: $((PASSED + FAILED)) tests"
    echo "  ✓ PASADOS: $PASSED"
    echo "  ✗ FALLIDOS: $FAILED"
    echo "========================================"
    echo ""
    
    echo "========================================" >> "$RESULTS_FILE"
    echo "  TOTAL: $((PASSED + FAILED)) tests" >> "$RESULTS_FILE"
    echo "  ✓ PASADOS: $PASSED" >> "$RESULTS_FILE"
    echo "  ✗ FALLIDOS: $FAILED" >> "$RESULTS_FILE"
    echo "========================================" >> "$RESULTS_FILE"
    
    if [ $FAILED -eq 0 ]; then
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}  ✓ ¡TODO LISTO PARA USAR!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        echo "Para ejecutar el robot:"
        echo "  source ~/dobot_ws/install/setup.bash"
        echo "  ros2 launch cr20_moveit demo.launch.py"
        echo ""
    else
        echo -e "${RED}========================================${NC}"
        echo -e "${RED}  ⚠ Se requieren correcciones${NC}"
        echo -e "${RED}========================================${NC}"
        echo ""
    fi
    
    echo "Resultados guardados en: $RESULTS_FILE"
}

main
