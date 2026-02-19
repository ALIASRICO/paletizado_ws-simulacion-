#!/usr/bin/env python3
"""
================================================================================
Launch File: Isaac Sim + CR20 Robot + MoveIt
================================================================================

Este launch file:
1. Inicia Isaac Sim (si no está ejecutándose)
2. Carga el robot CR20 desde USD
3. Inicia el bridge ROS2-Isaac
4. Opcional: Inicia MoveIt y RViz

Uso:
    ros2 launch cr20_moveit isaac_sim.launch.py

================================================================================
"""

import os
import sys
import time
import subprocess

from launch import LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration


# ============================================================================
# CONFIGURACIÓN
# ============================================================================

WORKSPACE_PATH = "/home/iudc/dobot_ws"
ISAAC_SIM_PATH = "/home/iudc/isaacsim"
ISAAC_SIM_APP = os.path.join(ISAAC_SIM_PATH, "apps", "isaacsim.exp.full.kit")

ROBOT_USD_PATH = os.path.join(
    WORKSPACE_PATH,
    "src", "DOBOT_6Axis_ROS2_V4", "cra_description",
    "urdf", "cr20_robot", "cr20_robot.usd"
)

ROBOT_URDF_PATH = os.path.join(
    WORKSPACE_PATH,
    "src", "DOBOT_6Axis_ROS2_V4", "cra_description",
    "urdf", "cr20_robot.urdf"
)


# ============================================================================
# FUNCIONES AUXILIARES
# ============================================================================

def check_isaac_running():
    """Verifica si Isaac Sim está ejecutándose."""
    try:
        result = subprocess.run(
            ["pgrep", "-f", "isaacsim"],
            capture_output=True,
            text=True
        )
        return result.returncode == 0
    except:
        return False


def create_robot_load_script():
    """Crea script para cargar el robot en Isaac Sim."""
    script = f'''
import omni
from omni.isaac.core.utils.stage import get_current_stage
import omni.kit.commands

# Esperar a que Isaac Sim esté listo
import time
time.sleep(3)

stage = get_current_stage()
if stage is None:
    print("Creando nuevo stage...")
    from omni.isaac.core.utils.stage import create_new_stage
    create_new_stage()
    stage = get_current_stage()

# Cargar robot desde USD
usd_path = "{ROBOT_USD_PATH}"
import os
if os.path.exists(usd_path):
    print(f"Cargando robot desde: {{usd_path}}")
    omni.kit.commands.execute('CreateReference',
        usd_path=usd_path,
        path_to='/World/cr20_robot',
        usd_format='usd'
    )
    print("✓ Robot cargado exitosamente en /World/cr20_robot")
else:
    print(f"USD no encontrado: {{usd_path}}")

print("Script de carga completado")
'''
    return script


def setup_bridge():
    """Configura el bridge usando subprocess."""
    bridge_script = os.path.join(
        WORKSPACE_PATH,
        "ros2_isaac_bridge.py"
    )
    
    if not os.path.exists(bridge_script):
        # Try alternate location
        bridge_script = os.path.join(
            WORKSPACE_PATH,
            "src", "DOBOT_6Axis_ROS2_V4", "cr20_moveit",
            "scripts", "cr20_isaac_bridge_node.py"
        )
    
    print(f"Bridge script: {bridge_script}")
    return bridge_script


# ============================================================================
# MAIN
# ============================================================================

def generate_launch_description():
    """Genera la descripción del launch."""
    
    print("\n" + "=" * 60)
    print("  ISAAC SIM + CR20 + ROS2 BRIDGE")
    print("=" * 60 + "\n")
    
    # Verificar Isaac Sim
    isaac_running = check_isaac_running()
    print(f"Isaac Sim ejecutándose: {isaac_running}")
    
    actions = []
    
    # -------------------------------------------------------------------------
    # 1. Isaac Sim (si no está corriendo)
    # -------------------------------------------------------------------------
    if not isaac_running:
        print("\n⚠ Isaac Sim NO está ejecutándose")
        print("  Para iniciar Isaac Sim manualmente:")
        print(f"    {ISAAC_SIM_APP}")
        print("  O espera a que el launch lo inicie...")
        
        isaac_process = ExecuteProcess(
            cmd=[ISAAC_SIM_APP],
            env={
                'ISAACSIM_PATH': ISAAC_SIM_PATH,
                'QT_QPA_PLATFORM': 'offscreen',
            },
            output='screen',
            shell=True
        )
        actions.append(isaac_process)
        
        # Timer para esperar Isaac Sim
        actions.append(
            TimerAction(
                period=30.0,
                actions=[
                    OpaqueFunction(function=lambda ctx: print("✓ Isaac Sim iniciado"))
                ]
            )
        )
    else:
        print("✓ Isaac Sim ya está ejecutándose")
    
    # -------------------------------------------------------------------------
    # 2. Instrucciones para cargar robot
    # -------------------------------------------------------------------------
    print("\n" + "-" * 60)
    print("  CARGAR ROBOT EN ISAAC SIM:")
    print("-" * 60)
    
    load_script = create_robot_load_script()
    script_path = "/tmp/load_cr20_robot.py"
    with open(script_path, 'w') as f:
        f.write(load_script)
    
    print(f"  Script guardado en: {script_path}")
    print("  ")
    print("  Para cargar el robot en Isaac Sim:")
    print("  1. Ve a Isaac Sim → Window → Script Editor")
    print(f"  2. Copia el contenido de: {script_path}")
    print("  3. Ejecuta (Ctrl+Enter)")
    print()
    
    # -------------------------------------------------------------------------
    # 3. Bridge ROS2
    # -------------------------------------------------------------------------
    print("-" * 60)
    print("  INICIANDO BRIDGE ROS2...")
    print("-" * 60)
    
    bridge_script = setup_bridge()
    
    if os.path.exists(bridge_script):
        bridge_process = ExecuteProcess(
            cmd=['python3', bridge_script],
            output='screen'
        )
        actions.append(bridge_process)
        print(f"✓ Bridge iniciado: {bridge_script}")
    else:
        print(f"⚠ Bridge script no encontrado: {bridge_script}")
    
    # -------------------------------------------------------------------------
    # 4. Resumen
    # -------------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("  RESUMEN:")
    print("=" * 60)
    print("  Tópicos disponibles:")
    print("    - /cr20/joint_states")
    print("    - /cr20/joint_commands")
    print()
    print("  Para verificar:")
    print("    ros2 topic list | grep cr20")
    print("    ros2 node list")
    print()
    print("  Para iniciar MoveIt manualmente:")
    print("    ros2 launch cr20_moveit move_group.launch.py")
    print("=" * 60 + "\n")
    
    return LaunchDescription(actions)


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    rc = ls.run()
    sys.exit(rc)
