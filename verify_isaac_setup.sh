#!/bin/bash
# =============================================================================
# Script de Verificación - Setup Isaac Sim + ROS2
# =============================================================================

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Results file
RESULTS_FILE="$HOME/dobot_ws/verification_updated.txt"

# Counters
PASSED=0
FAILED=0
WARNINGS=0

print_header() {
    echo -e "\n${BLUE}============================================================${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}============================================================${NC}"
    echo "" >> "$RESULTS_FILE"
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
    ((WARNINGS++))
}

test_info() {
    echo -e "${CYAN}ℹ INFO${NC}: $1"
    echo "ℹ INFO: $1" >> "$RESULTS_FILE"
}

separator() {
    echo -e "${YELLOW}------------------------------------------------------------${NC}"
    echo "------------------------------------------------------------" >> "$RESULTS_FILE"
}

main() {
    echo -e "${BLUE}"
    echo "============================================================"
    echo "  ISAAC SIM + ROS2 - Verificación Completa"
    echo "============================================================"
    echo -e "${NC}"
    
    echo "Verification Results - $(date)" > "$RESULTS_FILE"
    echo "" >> "$RESULTS_FILE"
    
    # =========================================================================
    # 1. VERIFICACIONES ISAAC SIM
    # =========================================================================
    print_header "1. VERIFICACIONES ISAAC SIM"
    
    # 1.1 Check Isaac Sim directory
    echo "Test 1.1: Directorio Isaac Sim"
    if [ -d "/home/iudc/isaacsim" ]; then
        test_pass "Isaac Sim encontrado en /home/iudc/isaacsim"
        test_info "Contenido:"
        ls -la /home/iudc/isaacsim/ | head -10 >> "$RESULTS_FILE"
    else
        test_fail "Isaac Sim NO encontrado en /home/iudc/isaacsim"
    fi
    
    # 1.2 Check Isaac Sim process
    echo "Test 1.2: Proceso Isaac Sim ejecutándose"
    if pgrep -f "isaacsim" > /dev/null || pgrep -f "kit" > /dev/null; then
        test_pass "Isaac Sim está ejecutándose"
        echo "   Procesos encontrados:" >> "$RESULTS_FILE"
        ps aux | grep -E "(isaacsim|kit)" | grep -v grep | head -5 >> "$RESULTS_FILE" 2>/dev/null
    else
        test_warn "Isaac Sim NO está ejecutándose"
        test_info "Para iniciar: /home/iudc/isaacsim/kit/kit /home/iudc/isaacsim/apps/isaacsim.exp.full.kit"
    fi
    
    # 1.3 Check kit executable
    echo "Test 1.3: Ejecutable kit"
    if [ -f "/home/iudc/isaacsim/kit/kit" ]; then
        test_pass "kit ejecutable disponible"
    else
        test_fail "kit ejecutable NO encontrado"
    fi
    
    # 1.4 Check isaacsim app
    echo "Test 1.4: App isaacsim.exp.full.kit"
    if [ -f "/home/iudc/isaacsim/apps/isaacsim.exp.full.kit" ]; then
        test_pass "App isaacsim.exp.full.kit disponible"
    else
        test_fail "App isaacsim.exp.full.kit NO encontrada"
    fi
    
    # =========================================================================
    # 2. VERIFICACIONES ROS2 JAZZY
    # =========================================================================
    print_header "2. VERIFICACIONES ROS2 JAZZY"
    
    # Source ROS2
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    
    echo "Test 2.1: ROS2 Jazzy"
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        test_pass "ROS2 Jazzy instalado"
    else
        test_fail "ROS2 Jazzy NO encontrado"
    fi
    
    echo "Test 2.2: Comando ros2"
    if command -v ros2 &> /dev/null; then
        test_pass "Comando ros2 disponible"
        echo "   Versión: $(ros2 version 2>/dev/null)" >> "$RESULTS_FILE"
    else
        test_fail "Comando ros2 NO disponible"
    fi
    
    echo "Test 2.3: Workspace compilado"
    if [ -d "$HOME/dobot_ws/install" ]; then
        test_pass "Directorio install/ existe"
    else
        test_fail "Workspace NO compilado"
    fi
    
    # =========================================================================
    # 3. VERIFICACIONES ROS2 - TOPICS Y NODOS
    # =========================================================================
    print_header "3. VERIFICACIONES ROS2 - TOPICS Y NODOS"
    
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    source $HOME/dobot_ws/install/setup.bash 2>/dev/null
    
    echo "Test 3.1: Tópics ROS2 activos"
    TOPICS=$(ros2 topic list 2>/dev/null)
    if [ -n "$TOPICS" ]; then
        test_pass "Tópics disponibles"
        echo "   Tópics encontrados:" >> "$RESULTS_FILE"
        echo "$TOPICS" | head -15 >> "$RESULTS_FILE"
    else
        test_warn "No hay tópicos activos (¿ROS2 corriendo?)"
    fi
    
    echo "Test 3.2: Nodos ROS2 activos"
    NODES=$(ros2 node list 2>/dev/null)
    if [ -n "$NODES" ]; then
        test_pass "Nodos disponibles"
        echo "   Nodos:" >> "$RESULTS_FILE"
        echo "$NODES" >> "$RESULTS_FILE"
    else
        test_warn "No hay nodos activos"
    fi
    
    echo "Test 3.3: Bridge ROS2-Isaac"
    BRIDGE_CHECK=$(ros2 node list 2>/dev/null | grep -i "bridge\|ign\|gz")
    if [ -n "$BRIDGE_CHECK" ]; then
        test_pass "Bridge encontrado: $BRIDGE_CHECK"
    else
        test_warn "Bridge ROS2-Isaac NO activo"
    fi
    
    # =========================================================================
    # 4. VERIFICACIONES CR20
    # =========================================================================
    print_header "4. VERIFICACIONES CR20"
    
    # 4.1 Check URDF
    echo "Test 4.1: cr20.urdf en workspace"
    URDF_WS="$HOME/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cra_description/urdf/cr20_robot.urdf"
    if [ -f "$URDF_WS" ]; then
        test_pass "cr20_robot.urdf encontrado en workspace"
    else
        test_fail "cr20_robot.urdf NO encontrado"
    fi
    
    # 4.2 Check user's CR20 scripts
    echo "Test 4.2: Scripts de importación CR20"
    SCRIPTS_FOUND=0
    for script in "cr20_robot_isaac_fixed.urdf" "isaac_import_cr20.py" "isaac_import_simple.py" "isaac_import_with_meshes.py"; do
        if [ -f "$HOME/$script" ]; then
            test_info "Script encontrado: $script"
            ((SCRIPTS_FOUND++))
        fi
    done
    if [ $SCRIPTS_FOUND -gt 0 ]; then
        test_pass "$SCRIPTS_FOUND scripts de CR20 encontrados"
    else
        test_warn "Scripts de importación NO encontrados en home"
    fi
    
    # 4.3 Check action graph
    echo "Test 4.3: Action Graph para ROS2"
    if [ -f "$HOME/isaac_ros2_action_graph.py" ]; then
        test_pass "isaac_ros2_action_graph.py encontrado"
    else
        test_warn "isaac_ros2_action_graph.py NO encontrado"
    fi
    
    # 4.4 Check MoveIt config
    echo "Test 4.4: Configuración MoveIt para CR20"
    MOVEIT_CONFIG="$HOME/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cr20_moveit"
    if [ -d "$MOVEIT_CONFIG" ]; then
        test_pass "Directorio cr20_moveit existe"
        if [ -f "$MOVEIT_CONFIG/config/cr20_robot.srdf" ]; then
            test_info "SRDF configurado"
        fi
    else
        test_fail "Directorio cr20_moveit NO encontrado"
    fi
    
    # =========================================================================
    # 5. CONEXIÓN ISAAC-ROS2
    # =========================================================================
    print_header "5. CONEXIÓN ISAAC-ROS2"
    
    echo "Test 5.1: Verificar conexión Isaac-ROS2"
    
    # Check for Isaac-related topics
    ISAAC_TOPICS=$(ros2 topic list 2>/dev/null | grep -E "(joint|robot|controller| trajectory)" || true)
    if [ -n "$ISAAC_TOPICS" ]; then
        test_pass "Tópics relacionados con robot encontrados"
        echo "$ISAAC_TOPICS" >> "$RESULTS_FILE"
    else
        test_warn "No se detectan tópicos de robot"
    fi
    
    # Check for controllers
    CONTROLLERS=$(ros2 controller list 2>/dev/null || true)
    if [ -n "$CONTROLLERS" ]; then
        test_pass "Controllers ROS2 activos"
        echo "$CONTROLLERS" >> "$RESULTS_FILE"
    else
        test_warn "No hay controllers activos"
    fi
    
    # =========================================================================
    # RESUMEN FINAL
    # =========================================================================
    print_header "RESUMEN FINAL"
    
    echo ""
    separator
    echo -e "  ${BLUE}RESULTADOS:${NC}"
    separator
    echo "  ✓ PASADOS:    $PASSED"
    echo "  ✗ FALLIDOS:   $FAILED"
    echo "  ⚠ ADVERTENCIAS: $WARNINGS"
    separator
    echo ""
    
    echo "========================================" >> "$RESULTS_FILE"
    echo "  RESUMEN FINAL" >> "$RESULTS_FILE"
    echo "========================================" >> "$RESULTS_FILE"
    echo "  ✓ PASADOS:    $PASSED" >> "$RESULTS_FILE"
    echo "  ✗ FALLIDOS:   $FAILED" >> "$RESULTS_FILE"
    echo "  ⚠ ADVERTENCIAS: $WARNINGS" >> "$RESULTS_FILE"
    echo "========================================" >> "$RESULTS_FILE"
    
    # Determine if ready
    if [ $FAILED -eq 0 ]; then
        if [ $WARNINGS -eq 0 ]; then
            echo -e "${GREEN}========================================${NC}"
            echo -e "${GREEN}  ✓ ¡TODO LISTO PARA OPERAR!${NC}"
            echo -e "${GREEN}========================================${NC}"
            echo ""
            echo "Comandos para iniciar:"
            echo "  1. Iniciar Isaac Sim:"
            echo "     /home/iudc/isaacsim/kit/kit /home/iudc/isaacsim/apps/isaacsim.exp.full.kit"
            echo ""
            echo "  2. En otra terminal:"
            echo "     source ~/dobot_ws/install/setup.bash"
            echo "     ros2 launch cr20_moveit demo.launch.py"
            echo ""
        else
            echo -e "${YELLOW}========================================${NC}"
            echo -e "${YELLOW}  ⚠ PARCIALMENTE FUNCIONAL${NC}"
            echo -e "${YELLOW}========================================${NC}"
            echo ""
            echo "Resolve las advertencias antes de operar."
        fi
    else
        echo -e "${RED}========================================${NC}"
        echo -e "${RED}  ✗ SE REQUIEREN CORRECCIONES${NC}"
        echo -e "${RED}========================================${NC}"
        echo ""
        echo "Revisa los resultados en: $RESULTS_FILE"
    fi
    
    echo ""
    echo "Resultados guardados en: $RESULTS_FILE"
}

main
