#!/usr/bin/env python3
"""
Conveyor Logic Controller - FASE 3: Industrial Grade
=====================================================

Sistema de banda transportadora con:
- Spawning inteligente (8s intervalo, verificación de colisión)
- Control de flujo con acumulación
- Velocidad industrial 0.1 m/s
- Detección de proximidad (0.18m)

Dimensiones Caja Industrial: 0.12 x 0.12 x 0.10m
Z de spawn: 0.80m (superficie 0.75 + mitad altura 0.05)

Autor: Sistema de Paletizado DOBOT CR20V
ROS2: Jazzy Jalisco | Gazebo: Sim 8 (Harmonic)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Int32, Bool
from std_srvs.srv import Trigger, SetBool
import subprocess
import json
import time
import math
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum


class BoxState(Enum):
    """Estados de una caja en el sistema"""
    MOVING = "moving"
    AT_PICK = "at_pick"
    STOPPED_AHEAD = "stopped_ahead"  # Detenida por caja adelante


@dataclass
class BoxObject:
    """Representa una caja industrial en el sistema"""
    model_name: str
    current_x: float
    current_y: float
    current_z: float
    state: BoxState = BoxState.MOVING
    is_active: bool = True


# ════════════════════════════════════════════════════════════════════════════
# CONFIGURACIÓN INDUSTRIAL
# ════════════════════════════════════════════════════════════════════════════

@dataclass
class IndustrialConfig:
    """Parámetros industriales del sistema"""
    # Dimensiones de caja
    BOX_SIZE_X: float = 0.12      # Largo (m)
    BOX_SIZE_Y: float = 0.12      # Ancho (m)
    BOX_SIZE_Z: float = 0.10      # Alto (m)
    BOX_MASS: float = 1.0         # Masa (kg)
    
    # Posiciones de referencia (valores por defecto, pueden ser sobrescritos)
    SURFACE_Z: float = 0.75       # Altura superficie banda
    SPAWN_X: float = -0.25        # X de spawn
    PICK_X: float = 0.25          # X de zona pick
    BELT_Y: float = 0.0           # Y de la banda
    SPAWN_Z: float = 0.80         # Z centro de caja (superficie + mitad altura)
    
    # Parámetros de movimiento
    CONVEYOR_SPEED: float = 0.1   # Velocidad industrial (m/s)
    UPDATE_RATE: float = 20.0     # Hz (20Hz para suavidad)
    
    # Distancias de seguridad
    MIN_SPAWN_GAP: float = 0.18   # Gap mínimo para spawn
    MIN_MOVE_GAP: float = 0.18    # Gap mínimo para movimiento
    PICK_TOLERANCE: float = 0.02  # Tolerancia zona pick
    
    # Spawning
    SPAWN_INTERVAL: float = 8.0   # Segundos entre spawns
    
    def update_from_params(self, spawn_x=None, pick_x=None, surface_z=None, belt_y=None):
        """Actualiza las posiciones desde parámetros"""
        if spawn_x is not None:
            self.SPAWN_X = spawn_x
            self.SPAWN_Z = surface_z + self.BOX_SIZE_Z / 2 if surface_z else 0.80
        if pick_x is not None:
            self.PICK_X = pick_x
        if surface_z is not None:
            self.SURFACE_Z = surface_z
            self.SPAWN_Z = surface_z + self.BOX_SIZE_Z / 2
        if belt_y is not None:
            self.BELT_Y = belt_y


def generate_box_sdf(model_name: str, x: float, y: float, z: float, 
                     config: IndustrialConfig) -> str:
    """
    Genera el SDF de una caja industrial.
    
    Justificación técnica del Z=0.80m:
    - Superficie de banda: 0.75m
    - Mitad de altura de caja: 0.10/2 = 0.05m
    - Centro de caja: 0.75 + 0.05 = 0.80m
    - Esto evita jitter porque la caja "descansa" exactamente sobre la superficie
      sin penetración inicial ni espacio vacío.
    """
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{model_name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <kinematic>true</kinematic>
      <inertial>
        <mass>{config.BOX_MASS}</mass>
        <inertia>
          <ixx>0.003</ixx><iyy>0.003</iyy><izz>0.003</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{config.BOX_SIZE_X} {config.BOX_SIZE_Y} {config.BOX_SIZE_Z}</size></box></geometry>
      </collision>
      <visual name="body_visual">
        <geometry><box><size>{config.BOX_SIZE_X} {config.BOX_SIZE_Y} {config.BOX_SIZE_Z}</size></box></geometry>
        <material>
          <ambient>0.6 0.5 0.4 1</ambient>
          <diffuse>0.6 0.5 0.4 1</diffuse>
        </material>
      </visual>
      <visual name="label_visual">
        <pose>0 0.061 0 0 0 0</pose>
        <geometry><box><size>0.08 0.002 0.05</size></box></geometry>
        <material>
          <ambient>0.95 0.95 0.95 1</ambient>
          <diffuse>0.95 0.95 0.95 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''


class GazeboClient:
    """
    Cliente para interactuar con Gazebo via CLI.
    
    Nota: En Gazebo Harmonic, los bridges de servicios pueden ser
    problemáticos desde Python. Este cliente usa CLI directamente.
    """
    
    def __init__(self, world_name: str, logger=None):
        self.world_name = world_name
        self.logger = logger
        
    def _log(self, level: str, msg: str):
        if self.logger:
            getattr(self.logger, level)(msg)
            
    def set_model_pose(self, model_name: str, x: float, y: float, z: float) -> bool:
        """Establece la pose de un modelo."""
        cmd = [
            'gz', 'service', '-s',
            f'/world/{self.world_name}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', 
            f'name: "{model_name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1}}'
        ]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            return 'true' in result.stdout.lower()
        except Exception as e:
            self._log('debug', f'set_pose error: {e}')
            return False
            
    def get_model_pose(self, model_name: str) -> Optional[Dict]:
        """Obtiene la pose actual de un modelo."""
        cmd = ['gz', 'model', '-m', model_name, '-p']
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=3.0)
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                for line in lines:
                    if '[' in line and ']' in line:
                        start = line.index('[') + 1
                        end = line.index(']')
                        values = line[start:end].split()
                        if len(values) >= 3:
                            return {
                                'x': float(values[0]),
                                'y': float(values[1]),
                                'z': float(values[2])
                            }
        except Exception as e:
            self._log('debug', f'get_pose error: {e}')
        return None
        
    def spawn_model(self, model_name: str, sdf: str) -> bool:
        """
        Spawnea un modelo usando el servicio create de Gazebo.
        
        En Gazebo Harmonic, el CLI requiere SDF en una sola línea
        con comillas dobles escapadas.
        """
        import re
        
        # Convertir SDF multilínea a una sola línea
        sdf_single_line = re.sub(r'\s+', ' ', sdf.strip())
        
        # Escapar comillas dobles para el CLI
        sdf_escaped = sdf_single_line.replace('"', '\\"')
        
        # Construir comando como string completo
        cmd_str = f'gz service -s /world/{self.world_name}/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 --req \'sdf: "{sdf_escaped}"\''
        
        try:
            result = subprocess.run(cmd_str, shell=True, capture_output=True, text=True, timeout=15.0)
            
            # Verificar si el modelo fue creado
            time.sleep(0.5)
            models = self.list_models()
            if model_name in models:
                self._log('info', f'✅ Modelo creado: {model_name}')
                return True
            else:
                self._log('warn', f'❌ Error spawn: modelo no encontrado')
                return False
            return success
        except Exception as e:
            self._log('error', f'spawn error: {e}')
            return False
            
    def delete_model(self, model_name: str) -> bool:
        """Elimina un modelo de la simulación."""
        cmd = [
            'gz', 'service', '-s',
            f'/world/{self.world_name}/remove',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', f'name: "{model_name}", type: 2'
        ]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10.0)
            return 'true' in result.stdout.lower()
        except Exception as e:
            self._log('error', f'delete error: {e}')
            return False
            
    def list_models(self) -> List[str]:
        """Lista todos los modelos en la simulación."""
        cmd = ['gz', 'model', '--list']
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                models = []
                for line in lines:
                    line = line.strip()
                    if line.startswith('- '):
                        models.append(line[2:])
                return models
        except Exception as e:
            self._log('debug', f'list error: {e}')
        return []


class ConveyorLogicController(Node):
    """
    Controlador de banda transportadora industrial.
    
    Características FASE 3:
    - Spawning inteligente cada 8s con verificación de colisión
    - Control de flujo con acumulación (gap 0.18m)
    - Velocidad industrial 0.1 m/s
    - Loop a 20Hz
    """
    
    def __init__(self):
        super().__init__('conveyor_logic_controller')
        
        # Configuración industrial
        self.config = IndustrialConfig()
        
        # Parámetros ROS2 (pueden sobrescribir config)
        self.declare_parameter('world_name', 'conveyor_world')
        self.declare_parameter('conveyor_speed', self.config.CONVEYOR_SPEED)
        self.declare_parameter('auto_spawn', True)
        self.declare_parameter('spawn_interval', self.config.SPAWN_INTERVAL)
        self.declare_parameter('update_rate', self.config.UPDATE_RATE)
        # Parámetros de posición de la banda
        self.declare_parameter('spawn_x', None)
        self.declare_parameter('pick_x', None)
        self.declare_parameter('surface_z', None)
        self.declare_parameter('belt_y', None)
        
        self.world_name = self.get_parameter('world_name').value
        self.conveyor_speed = self.get_parameter('conveyor_speed').value
        self.auto_spawn = self.get_parameter('auto_spawn').value
        self.spawn_interval = self.get_parameter('spawn_interval').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Actualizar configuración de la banda desde parámetros
        spawn_x = self.get_parameter('spawn_x').value
        pick_x = self.get_parameter('pick_x').value
        surface_z = self.get_parameter('surface_z').value
        belt_y = self.get_parameter('belt_y').value
        
        if spawn_x is not None or pick_x is not None or surface_z is not None or belt_y is not None:
            self.config.update_from_params(spawn_x, pick_x, surface_z, belt_y)
            self.get_logger().info(f'  Banda configurada: spawn_x={self.config.SPAWN_X}, pick_x={self.config.PICK_X}, surface_z={self.config.SURFACE_Z}, belt_y={self.config.BELT_Y}')
        
        # Estado interno
        self.active_boxes: Dict[str, BoxObject] = {}
        self.box_counter = 0
        self.running = True
        self.belt_active = True
        self.last_spawn_time = time.time()
        
        # Cliente Gazebo
        self.gz_client = GazeboClient(self.world_name, self.get_logger())
        
        # Callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # ═══════════════════════════════════════════════════════════════
        # PUBLICADORES
        # ═══════════════════════════════════════════════════════════════
        self.status_pub = self.create_publisher(String, '/conveyor/status', 10)
        self.count_pub = self.create_publisher(Int32, '/conveyor/objects/count', 10)
        self.pick_pub = self.create_publisher(String, '/conveyor/next_pick', 10)
        self.spawn_pub = self.create_publisher(String, '/conveyor/spawned', 10)
        
        # ═══════════════════════════════════════════════════════════════
        # TIMERS
        # ═══════════════════════════════════════════════════════════════
        
        # Timer principal de control (20Hz)
        control_period = 1.0 / self.update_rate
        self.control_timer = self.create_timer(control_period, self.control_loop)
        
        # Timer de estado (1Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Variable para controlar spawn por posición
        self.last_spawn_x = None  # X de la última caja spawneada
        
        # ═══════════════════════════════════════════════════════════════
        # SERVICIOS
        # ═══════════════════════════════════════════════════════════════
        self.start_srv = self.create_service(Trigger, '/conveyor/start', self.start_callback)
        self.stop_srv = self.create_service(Trigger, '/conveyor/stop', self.stop_callback)
        self.spawn_srv = self.create_service(Trigger, '/conveyor/spawn_box', self.spawn_callback)
        
        # ═══════════════════════════════════════════════════════════════
        # INICIALIZACIÓN
        # ═══════════════════════════════════════════════════════════════
        self._register_existing_boxes()
        
        self.get_logger().info('='*60)
        self.get_logger().info('CONVEYOR LOGIC CONTROLLER - FASE 3 Industrial')
        self.get_logger().info('='*60)
        self.get_logger().info(f'  Velocidad: {self.conveyor_speed} m/s')
        self.get_logger().info(f'  Spawn interval: {self.spawn_interval}s')
        self.get_logger().info(f'  Update rate: {self.update_rate}Hz')
        self.get_logger().info(f'  Gap mínimo: {self.config.MIN_MOVE_GAP}m')
        self.get_logger().info(f'  Caja: {self.config.BOX_SIZE_X}x{self.config.BOX_SIZE_Y}x{self.config.BOX_SIZE_Z}m')
        self.get_logger().info('='*60)
        
    def _register_existing_boxes(self):
        """Registra cajas existentes en la simulación."""
        models = self.gz_client.list_models()
        
        for model_name in models:
            if model_name.startswith('box_') or model_name.startswith('item_box_'):
                pose = self.gz_client.get_model_pose(model_name)
                if pose:
                    self.active_boxes[model_name] = BoxObject(
                        model_name=model_name,
                        current_x=pose['x'],
                        current_y=pose['y'],
                        current_z=pose['z'],
                        state=BoxState.MOVING
                    )
                    # Actualizar contador
                    try:
                        num = int(model_name.split('_')[-1])
                        self.box_counter = max(self.box_counter, num)
                    except:
                        pass
                        
                    self.get_logger().info(f'📦 Caja existente: {model_name} en X={pose["x"]:.3f}')
                    
    def _get_sorted_boxes(self) -> List[Tuple[str, BoxObject]]:
        """Retorna cajas ordenadas por posición X (descendente)."""
        return sorted(
            [(name, box) for name, box in self.active_boxes.items() if box.is_active],
            key=lambda x: x[1].current_x,
            reverse=True
        )
        
    def _check_spawn_space(self) -> bool:
        """
        Verifica si hay espacio suficiente para spawnear.
        
        Retorna True si NO hay cajas dentro del gap mínimo.
        """
        for box in self.active_boxes.values():
            if box.is_active:
                distance = abs(box.current_x - self.config.SPAWN_X)
                if distance < self.config.MIN_SPAWN_GAP:
                    return False
        return True
        
    def _find_box_ahead(self, box: BoxObject) -> Optional[BoxObject]:
        """
        Encuentra la caja inmediatamente adelante de la dada.
        
        Retorna None si no hay caja adelante.
        """
        min_distance = float('inf')
        ahead_box = None
        
        for other in self.active_boxes.values():
            if other.model_name == box.model_name or not other.is_active:
                continue
                
            # Solo cajas adelante (mayor X)
            if other.current_x > box.current_x:
                distance = other.current_x - box.current_x
                if distance < min_distance:
                    min_distance = distance
                    ahead_box = other
                    
        return ahead_box
        
    def spawn_loop(self):
        """
        Spawn basado en POSICIÓN, no en tiempo.
        
        Lógica: Cuando la última caja spawneada avanza lo suficiente
        (X > SPAWN_X + MIN_SPAWN_GAP), spawnear una nueva caja.
        """
        if not self.auto_spawn or not self.running:
            return
            
        # Verificar espacio
        if not self._check_spawn_space():
            return
            
        # Spawnear nueva caja
        self._spawn_new_box()
        
    def _spawn_new_box(self) -> Optional[str]:
        """Crea una nueva caja industrial en posición de spawn."""
        self.box_counter += 1
        model_name = f"box_{self.box_counter}"
        
        # Generar SDF
        sdf = generate_box_sdf(
            model_name,
            self.config.SPAWN_X,
            self.config.BELT_Y,
            self.config.SPAWN_Z,
            self.config
        )
        
        # Spawn
        success = self.gz_client.spawn_model(model_name, sdf)
        
        if success:
            self.active_boxes[model_name] = BoxObject(
                model_name=model_name,
                current_x=self.config.SPAWN_X,
                current_y=self.config.BELT_Y,
                current_z=self.config.SPAWN_Z,
                state=BoxState.MOVING
            )
            
            # Notificar
            msg = String()
            msg.data = model_name
            self.spawn_pub.publish(msg)
            
            return model_name
            
        self.box_counter -= 1
        return None
        
    def control_loop(self):
        """
        Loop principal de control (20Hz).
        
        Implementa:
        1. Movimiento de cajas
        2. Detección de zona pick
        3. Acumulación (stop si caja adelante muy cerca)
        4. Resume cuando se despeja
        """
        if not self.running or not self.belt_active:
            return
            
        dt = 1.0 / self.update_rate
        displacement = self.conveyor_speed * dt
        
        # Obtener cajas ordenadas por X (de adelante hacia atrás)
        sorted_boxes = self._get_sorted_boxes()
        
        for model_name, box in sorted_boxes:
            if not box.is_active:
                continue
                
            # ═══════════════════════════════════════════════════════════
            # CASO 1: Ya está en zona pick - NO mover
            # ═══════════════════════════════════════════════════════════
            if box.state == BoxState.AT_PICK:
                continue
                
            # ═══════════════════════════════════════════════════════════
            # CASO 2: Verificar si llegó a zona pick
            # ═══════════════════════════════════════════════════════════
            if box.current_x >= (self.config.PICK_X - self.config.PICK_TOLERANCE):
                # Actualizar estado
                box.state = BoxState.AT_PICK
                box.current_x = self.config.PICK_X
                
                # Fijar posición
                self.gz_client.set_model_pose(
                    model_name,
                    self.config.PICK_X,
                    box.current_y,
                    box.current_z
                )
                
                self.get_logger().info(f'🎯 {model_name} en zona PICK')
                
                # Notificar
                msg = String()
                msg.data = model_name
                self.pick_pub.publish(msg)
                continue
                
            # ═══════════════════════════════════════════════════════════
            # CASO 3: Verificar caja adelante (acumulación)
            # ═══════════════════════════════════════════════════════════
            ahead_box = self._find_box_ahead(box)
            
            if ahead_box:
                distance = ahead_box.current_x - box.current_x
                
                # Si la caja de adelante está muy cerca o detenida
                if distance < self.config.MIN_MOVE_GAP or ahead_box.state in [BoxState.AT_PICK, BoxState.STOPPED_AHEAD]:
                    box.state = BoxState.STOPPED_AHEAD
                    continue  # No mover
                    
            # ═══════════════════════════════════════════════════════════
            # CASO 4: Movimiento normal
            # ═══════════════════════════════════════════════════════════
            box.state = BoxState.MOVING
            new_x = box.current_x + displacement
            
            # Actualizar pose
            success = self.gz_client.set_model_pose(
                model_name,
                new_x,
                box.current_y,
                box.current_z
            )
            
            if success:
                box.current_x = new_x
                
        # ═══════════════════════════════════════════════════════════════
        # SPAWN POR POSICIÓN
        # Spawnear nueva caja cuando la última haya avanzado lo suficiente
        # ═══════════════════════════════════════════════════════════════
        if self.auto_spawn:
            # Encontrar la caja más cercana al spawn
            min_x = float('inf')
            for box in self.active_boxes.values():
                if box.is_active and box.current_x < min_x:
                    min_x = box.current_x
                    
            # Si no hay cajas o la más cercana ha avanzado lo suficiente, spawnear
            spawn_threshold = self.config.SPAWN_X + self.config.MIN_SPAWN_GAP
            if min_x == float('inf') or min_x >= spawn_threshold:
                self.spawn_loop()
                
    def publish_status(self):
        """Publica estado del sistema."""
        # Contar cajas por estado
        active_count = sum(1 for b in self.active_boxes.values() if b.is_active)
        at_pick_count = sum(1 for b in self.active_boxes.values() if b.state == BoxState.AT_PICK)
        stopped_count = sum(1 for b in self.active_boxes.values() if b.state == BoxState.STOPPED_AHEAD)
        
        # Publicar conteo
        count_msg = Int32()
        count_msg.data = active_count
        self.count_pub.publish(count_msg)
        
        # Publicar estado JSON
        status = {
            'running': self.running,
            'belt_active': self.belt_active,
            'speed': self.conveyor_speed,
            'active_boxes': active_count,
            'boxes_at_pick': at_pick_count,
            'boxes_stopped': stopped_count,
            'box_counter': self.box_counter
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)
        
    # ═══════════════════════════════════════════════════════════════════════
    # CALLBACKS DE SERVICIOS
    # ═══════════════════════════════════════════════════════════════════════
    
    def start_callback(self, request, response):
        self.belt_active = True
        self.get_logger().info('✅ Banda INICIADA')
        response.success = True
        response.message = f'Banda iniciada a {self.conveyor_speed} m/s'
        return response
        
    def stop_callback(self, request, response):
        self.belt_active = False
        self.get_logger().info('⏹️ Banda DETENIDA')
        response.success = True
        response.message = 'Banda detenida'
        return response
        
    def spawn_callback(self, request, response):
        model_name = self._spawn_new_box()
        if model_name:
            response.success = True
            response.message = f'Caja {model_name} creada'
        else:
            response.success = False
            response.message = 'Error al crear caja o espacio insuficiente'
        return response
        
    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ConveyorLogicController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
