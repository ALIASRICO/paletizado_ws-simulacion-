#!/usr/bin/env python3
"""
Controlador de cinta transportadora para Gazebo Sim 8
Permite controlar la velocidad de la cinta mediante comandos de Gazebo Transport
"""

import subprocess
import time
import sys
import argparse


def set_conveyor_power(power: float) -> bool:
    """
    Establece la potencia de la cinta transportadora.
    
    Args:
        power: Potencia de 0 a 100 (porcentaje)
    
    Returns:
        True si el comando se envió correctamente
    """
    if not 0 <= power <= 100:
        print(f"Error: La potencia debe estar entre 0 y 100, recibido: {power}")
        return False
    
    cmd = [
        'gz', 'topic', '-t', '/conveyor/power',
        '-m', 'gz.msgs.Double',
        '-p', f'data: {power}'
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✓ Potencia establecida a {power}%")
            return True
        else:
            print(f"✗ Error: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print("✗ Timeout enviando comando")
        return False
    except FileNotFoundError:
        print("✗ Comando 'gz' no encontrado. ¿Está Gazebo Sim instalado?")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Controlador de cinta transportadora para Gazebo Sim 8'
    )
    parser.add_argument(
        'power', type=float, nargs='?', default=50.0,
        help='Potencia de la cinta (0-100). Default: 50'
    )
    parser.add_argument(
        '--ramp', type=float, metavar='TARGET',
        help='Aumentar gradualmente hasta TARGET'
    )
    parser.add_argument(
        '--stop', action='store_true',
        help='Detener la cinta (power=0)'
    )
    parser.add_argument(
        '--cycle', action='store_true',
        help='Ciclo automático: iniciar, esperar, detener'
    )
    
    args = parser.parse_args()
    
    if args.stop:
        print("Deteniendo cinta transportadora...")
        set_conveyor_power(0.0)
        return
    
    if args.cycle:
        print("=== Ciclo automático ===")
        print("Iniciando cinta al 50%...")
        set_conveyor_power(50.0)
        time.sleep(5)
        print("Aumentando a 80%...")
        set_conveyor_power(80.0)
        time.sleep(5)
        print("Deteniendo...")
        set_conveyor_power(0.0)
        print("Ciclo completado.")
        return
    
    if args.ramp is not None:
        target = args.ramp
        if not 0 <= target <= 100:
            print(f"Error: TARGET debe estar entre 0 y 100")
            sys.exit(1)
        
        print(f"=== Rampa de potencia: 0% -> {target}% ===")
        current = 0.0
        step = target / 10.0
        while current <= target:
            set_conveyor_power(current)
            current += step
            time.sleep(0.5)
        set_conveyor_power(target)
        return
    
    # Comando simple de potencia
    set_conveyor_power(args.power)


if __name__ == '__main__':
    main()
