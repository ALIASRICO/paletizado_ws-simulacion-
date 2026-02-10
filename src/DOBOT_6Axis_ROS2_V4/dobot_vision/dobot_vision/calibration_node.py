#!/usr/bin/env python3
"""
calibration_node.py — Calibración cámara→robot para Dobot CR20 Palletizing

Proceso interactivo:
  1. La cámara detecta los 8 AprilTags (10-13 cinta, 20-23 palet)
  2. El operador mueve el TCP del robot al centro de cada tag (teach/drag)
  3. Para cada tag: se registra (X_cam, Y_cam) y (X_rob, Y_rob)
  4. Se calcula la transformación Afín 2D: cam → robot
  5. Se miden Z_pick (conveyor) y Z_palet (base palet)
  6. Se guarda calibration.yaml

Uso:
  1. Lanzar driver del robot
  2. Lanzar visión real (cámara + apriltag)
  3. Lanzar este nodo
  4. Seguir instrucciones en terminal

REGLAS APLICADAS:
  #1 Safety First  — velocidad baja, confirmación antes de cada paso
  #2 Desacoplado   — usa servicios del driver, no TCP directo
  #3 Configurable  — tag IDs, paths, todo por parámetros
  #5 Clean Code    — tipado, docstrings, logging
"""

from typing import Dict, List, Optional, Tuple
import os
import time

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from dobot_msgs_v4.srv import GetPose, GetAngle, GetErrorID
from geometry_msgs.msg import PoseStamped


class CalibrationNode(Node):
    """Nodo interactivo de calibración cámara→robot."""

    def __init__(self) -> None:
        super().__init__('calibration_node')

        # ─── Parámetros ───
        self.declare_parameter('conveyor_tag_ids', [10, 11, 12, 13])
        self.declare_parameter('palet_tag_ids', [20, 21, 22, 23])
        self.declare_parameter('tag_size', 0.10)
        self.declare_parameter('detection_topic', '/apriltag/detections')
        self.declare_parameter('output_file', '')  # vacío = auto en config/
        self.declare_parameter('min_decision_margin', 30.0)
        self.declare_parameter('samples_per_tag', 20)

        self._conveyor_ids: List[int] = self.get_parameter('conveyor_tag_ids').value
        self._palet_ids: List[int] = self.get_parameter('palet_tag_ids').value
        self._all_tag_ids: List[int] = self._conveyor_ids + self._palet_ids
        self._detection_topic: str = self.get_parameter('detection_topic').value
        self._output_file: str = self.get_parameter('output_file').value
        self._min_margin: float = self.get_parameter('min_decision_margin').value
        self._samples_per_tag: int = self.get_parameter('samples_per_tag').value

        # Si no se especifica output, usar config/ del paquete
        if not self._output_file:
            pkg_share = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self._output_file = os.path.join(pkg_share, 'config', 'calibration.yaml')

        # ─── Estado ───
        self._latest_detections: Dict[int, Dict] = {}  # tag_id → {cx, cy, margin}
        self._camera_points: Dict[int, Tuple[float, float]] = {}  # tag_id → (x_cam, y_cam)
        self._robot_points: Dict[int, Tuple[float, float, float]] = {}  # tag_id → (x, y, z)
        self._z_pick: Optional[float] = None
        self._z_palet: Optional[float] = None

        # ─── Clientes de servicio del robot ───
        self._get_pose_client = self.create_client(
            GetPose, '/dobot_bringup_ros2/srv/GetPose'
        )
        self._get_error_client = self.create_client(
            GetErrorID, '/dobot_bringup_ros2/srv/GetErrorID'
        )

        # ─── Suscriptor de detecciones ───
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self._sub = self.create_subscription(
            AprilTagDetectionArray,
            self._detection_topic,
            self._detection_callback,
            qos,
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('CALIBRACIÓN CÁMARA → ROBOT')
        self.get_logger().info(f'  Tags cinta:  {self._conveyor_ids}')
        self.get_logger().info(f'  Tags palet:  {self._palet_ids}')
        self.get_logger().info(f'  Output:      {self._output_file}')
        self.get_logger().info('=' * 60)

    # ─────────────────────────────────────────────────────────────
    # Callback de detecciones
    # ─────────────────────────────────────────────────────────────
    def _detection_callback(self, msg: AprilTagDetectionArray) -> None:
        """Almacena las últimas detecciones de tags."""
        for det in msg.detections:
            tag_id = det.id
            if tag_id in self._all_tag_ids:
                # Centro del tag en píxeles de la imagen
                cx = det.centre.x
                cy = det.centre.y
                margin = det.decision_margin
                if margin >= self._min_margin:
                    self._latest_detections[tag_id] = {
                        'cx': cx, 'cy': cy, 'margin': margin
                    }

    # ─────────────────────────────────────────────────────────────
    # Promediar detecciones de un tag (reduce ruido)
    # ─────────────────────────────────────────────────────────────
    def _average_tag_position(self, tag_id: int) -> Optional[Tuple[float, float]]:
        """Acumula N muestras del tag y retorna promedio (cx, cy)."""
        samples_cx: List[float] = []
        samples_cy: List[float] = []

        self.get_logger().info(
            f'  Recolectando {self._samples_per_tag} muestras del tag {tag_id}...'
        )

        attempts = 0
        max_attempts = self._samples_per_tag * 10  # Timeout

        while len(samples_cx) < self._samples_per_tag and attempts < max_attempts:
            rclpy.spin_once(self, timeout_sec=0.2)
            attempts += 1

            if tag_id in self._latest_detections:
                det = self._latest_detections[tag_id]
                samples_cx.append(det['cx'])
                samples_cy.append(det['cy'])
                # Limpiar para obtener muestra fresca
                del self._latest_detections[tag_id]

        if len(samples_cx) < 3:
            self.get_logger().warn(
                f'  Solo se obtuvieron {len(samples_cx)} muestras para tag {tag_id}'
            )
            return None

        avg_cx = float(np.mean(samples_cx))
        avg_cy = float(np.mean(samples_cy))
        std_cx = float(np.std(samples_cx))
        std_cy = float(np.std(samples_cy))

        self.get_logger().info(
            f'  Tag {tag_id}: centro=({avg_cx:.1f}, {avg_cy:.1f}) px, '
            f'std=({std_cx:.2f}, {std_cy:.2f}), muestras={len(samples_cx)}'
        )

        return (avg_cx, avg_cy)

    # ─────────────────────────────────────────────────────────────
    # Leer pose del robot via servicio
    # ─────────────────────────────────────────────────────────────
    def _get_robot_pose(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """Llama a GetPose y retorna (x, y, z, rx, ry, rz) en mm/grados."""
        if not self._get_pose_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Servicio GetPose no disponible')
            return None

        future = self._get_pose_client.call_async(GetPose.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('GetPose no respondió')
            return None

        response = future.result()
        if response.res != 0:
            self.get_logger().error(f'GetPose error: res={response.res}')
            return None

        # Parsear '{x,y,z,rx,ry,rz}'
        values_str = response.robot_return.strip('{}')
        values = [float(v) for v in values_str.split(',')]
        if len(values) != 6:
            self.get_logger().error(f'GetPose formato inesperado: {response.robot_return}')
            return None

        return tuple(values)

    # ─────────────────────────────────────────────────────────────
    # Verificar errores del robot
    # ─────────────────────────────────────────────────────────────
    def _check_robot_errors(self) -> bool:
        """Retorna True si el robot NO tiene errores."""
        if not self._get_error_client.wait_for_service(timeout_sec=3.0):
            return False

        future = self._get_error_client.call_async(GetErrorID.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            return False

        result_str = future.result().robot_return
        return '[]' in result_str

    # ─────────────────────────────────────────────────────────────
    # Calcular transformación afín 2D
    # ─────────────────────────────────────────────────────────────
    def _compute_affine_transform(
        self,
        cam_points: np.ndarray,
        rob_points: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Calcula la transformación afín 2D: robot = A @ cam + t

        Args:
            cam_points: (N, 2) — puntos en coords cámara (px)
            rob_points: (N, 2) — puntos en coords robot (mm)

        Returns:
            A: (2, 2) — matriz de transformación
            t: (2,) — vector de traslación
            error_rms: error RMS de reproyección (mm)
        """
        n = cam_points.shape[0]
        assert n >= 3, f'Se necesitan al menos 3 puntos, hay {n}'

        # Construir sistema sobredeterminado:
        # [x_cam, y_cam, 1, 0, 0, 0] [a]   [x_rob]
        # [0, 0, 0, x_cam, y_cam, 1] [b] = [y_rob]
        #                              [tx]
        #                              [c]
        #                              [d]
        #                              [ty]

        A_mat = np.zeros((2 * n, 6))
        b_vec = np.zeros(2 * n)

        for i in range(n):
            cx, cy = cam_points[i]
            rx, ry = rob_points[i]

            A_mat[2 * i] = [cx, cy, 1, 0, 0, 0]
            A_mat[2 * i + 1] = [0, 0, 0, cx, cy, 1]
            b_vec[2 * i] = rx
            b_vec[2 * i + 1] = ry

        # Resolver por mínimos cuadrados
        result, residuals, rank, sv = np.linalg.lstsq(A_mat, b_vec, rcond=None)

        a, b, tx, c, d, ty = result

        A = np.array([[a, b], [c, d]])
        t = np.array([tx, ty])

        # Calcular error de reproyección
        predicted = (A @ cam_points.T).T + t
        errors = np.linalg.norm(predicted - rob_points, axis=1)
        error_rms = float(np.sqrt(np.mean(errors ** 2)))

        return A, t, error_rms

    # ─────────────────────────────────────────────────────────────
    # Proceso principal de calibración
    # ─────────────────────────────────────────────────────────────
    def run_calibration(self) -> bool:
        """Ejecuta el proceso interactivo de calibración."""

        print('\n' + '=' * 60)
        print('  CALIBRACIÓN CÁMARA → ROBOT')
        print('  Dobot CR20 Palletizing System')
        print('=' * 60)
        print()
        print('REQUISITOS:')
        print('  1. Driver del robot lanzado y conectado')
        print('  2. Robot habilitado (EnableRobot)')
        print('  3. Visión real lanzada (cámara + apriltag)')
        print('  4. Todos los tags visibles (10-13 cinta, 20-23 palet)')
        print()

        # ─── Verificar robot ───
        print('[1/5] Verificando conexión con robot...')
        if not self._check_robot_errors():
            print('  ERROR: Robot no accesible o tiene errores.')
            print('  Ejecute: ClearError + EnableRobot')
            return False
        pose = self._get_robot_pose()
        if pose is None:
            print('  ERROR: No se puede leer la pose del robot.')
            return False
        print(f'  OK — Pose actual: X={pose[0]:.1f}, Y={pose[1]:.1f}, Z={pose[2]:.1f} mm')

        # ─── Verificar tags visibles ───
        print('\n[2/5] Verificando tags visibles...')
        print('  Esperando detecciones (5 segundos)...')
        start = time.time()
        while time.time() - start < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        detected = list(self._latest_detections.keys())
        print(f'  Tags detectados: {sorted(detected)}')

        missing_conveyor = [t for t in self._conveyor_ids if t not in detected]
        missing_palet = [t for t in self._palet_ids if t not in detected]

        if missing_conveyor:
            print(f'  AVISO: Tags de CINTA no detectados: {missing_conveyor}')
        if missing_palet:
            print(f'  AVISO: Tags de PALET no detectados: {missing_palet}')

        available_tags = [t for t in self._all_tag_ids if t in detected]
        if len(available_tags) < 3:
            print(f'  ERROR: Se necesitan al menos 3 tags, solo hay {len(available_tags)}')
            return False

        print(f'  Se usarán {len(available_tags)} tags: {available_tags}')

        # ─── Recolectar puntos ───
        print('\n[3/5] RECOLECCIÓN DE PUNTOS')
        print('-' * 60)
        print('INSTRUCCIONES:')
        print('  - Para cada tag, mueva el TCP del robot hasta que la')
        print('    punta toque el CENTRO EXACTO del tag.')
        print('  - Use modo teach/drag o el pendant del robot.')
        print('  - Cuando esté posicionado, presione ENTER.')
        print('  - Para saltar un tag, escriba "s" + ENTER.')
        print('-' * 60)

        calibrated_tags: List[int] = []

        for tag_id in available_tags:
            zone = 'CINTA' if tag_id in self._conveyor_ids else 'PALET'
            print(f'\n  >>> Tag {tag_id} ({zone})')
            print(f'      Mueva el TCP al centro del tag {tag_id}')

            user_input = input('      [ENTER=registrar, s=saltar]: ').strip().lower()
            if user_input == 's':
                print(f'      Saltado tag {tag_id}')
                continue

            # Leer posición cámara (promedio de N muestras)
            cam_pos = self._average_tag_position(tag_id)
            if cam_pos is None:
                print(f'      ERROR: Tag {tag_id} no detectado, saltando.')
                continue

            # Leer posición robot
            robot_pose = self._get_robot_pose()
            if robot_pose is None:
                print(f'      ERROR: No se pudo leer pose del robot, saltando.')
                continue

            x_rob, y_rob, z_rob = robot_pose[0], robot_pose[1], robot_pose[2]

            self._camera_points[tag_id] = cam_pos
            self._robot_points[tag_id] = (x_rob, y_rob, z_rob)
            calibrated_tags.append(tag_id)

            print(f'      Cámara: ({cam_pos[0]:.1f}, {cam_pos[1]:.1f}) px')
            print(f'      Robot:  ({x_rob:.1f}, {y_rob:.1f}, {z_rob:.1f}) mm')
            print(f'      ✓ Tag {tag_id} registrado')

        if len(calibrated_tags) < 3:
            print(f'\n  ERROR: Solo {len(calibrated_tags)} tags calibrados, se necesitan 3+')
            return False

        # ─── Medir Z pick y Z palet ───
        print('\n[4/5] MEDICIÓN DE ALTURAS Z')
        print('-' * 60)

        # Calcular Z promedio de tags de cinta y palet
        conveyor_calibrated = [t for t in calibrated_tags if t in self._conveyor_ids]
        palet_calibrated = [t for t in calibrated_tags if t in self._palet_ids]

        if conveyor_calibrated:
            z_vals = [self._robot_points[t][2] for t in conveyor_calibrated]
            self._z_pick = float(np.mean(z_vals))
            print(f'  Z_pick (cinta) = {self._z_pick:.1f} mm  '
                  f'(promedio de tags {conveyor_calibrated})')
        else:
            print('  AVISO: No hay tags de cinta calibrados.')
            print('  Mueva el TCP a la superficie de la cinta y presione ENTER.')
            input('  [ENTER]: ')
            pose = self._get_robot_pose()
            if pose:
                self._z_pick = pose[2]
                print(f'  Z_pick (cinta) = {self._z_pick:.1f} mm')

        if palet_calibrated:
            z_vals = [self._robot_points[t][2] for t in palet_calibrated]
            self._z_palet = float(np.mean(z_vals))
            print(f'  Z_palet (base) = {self._z_palet:.1f} mm  '
                  f'(promedio de tags {palet_calibrated})')
        else:
            print('  AVISO: No hay tags de palet calibrados.')
            print('  Mueva el TCP a la superficie del palet y presione ENTER.')
            input('  [ENTER]: ')
            pose = self._get_robot_pose()
            if pose:
                self._z_palet = pose[2]
                print(f'  Z_palet (base) = {self._z_palet:.1f} mm')

        # ─── Calcular transformación ───
        print('\n[5/5] CÁLCULO DE TRANSFORMACIÓN AFÍN')
        print('-' * 60)

        cam_pts = np.array([self._camera_points[t] for t in calibrated_tags])
        rob_pts = np.array([
            [self._robot_points[t][0], self._robot_points[t][1]]
            for t in calibrated_tags
        ])

        A, t, error_rms = self._compute_affine_transform(cam_pts, rob_pts)

        print(f'  Matriz A:')
        print(f'    [{A[0, 0]:+.6f}  {A[0, 1]:+.6f}]')
        print(f'    [{A[1, 0]:+.6f}  {A[1, 1]:+.6f}]')
        print(f'  Traslación t:')
        print(f'    [{t[0]:+.2f}, {t[1]:+.2f}] mm')
        print(f'  Error RMS: {error_rms:.2f} mm')

        if error_rms > 10.0:
            print(f'  ⚠ AVISO: Error RMS alto ({error_rms:.1f} mm). '
                  f'Revisar precisión de posicionamiento.')

        # ─── Validación cruzada ───
        print('\n  Validación punto a punto:')
        for i, tag_id in enumerate(calibrated_tags):
            predicted = A @ cam_pts[i] + t
            actual = rob_pts[i]
            err = np.linalg.norm(predicted - actual)
            zone = 'CINTA' if tag_id in self._conveyor_ids else 'PALET'
            status = '✓' if err < 5.0 else '⚠'
            print(f'    Tag {tag_id} ({zone}): '
                  f'pred=({predicted[0]:.1f}, {predicted[1]:.1f}), '
                  f'real=({actual[0]:.1f}, {actual[1]:.1f}), '
                  f'err={err:.2f} mm {status}')

        # ─── Guardar calibración ───
        calibration_data = {
            'calibration': {
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'num_points': len(calibrated_tags),
                'error_rms_mm': round(error_rms, 3),
                'tags_used': calibrated_tags,

                'affine_transform': {
                    'description': 'robot_xy = A @ camera_px + t',
                    'A': A.tolist(),
                    't': t.tolist(),
                },

                'z_heights_mm': {
                    'z_pick': round(self._z_pick, 2) if self._z_pick else None,
                    'z_palet_base': round(self._z_palet, 2) if self._z_palet else None,
                    'z_approach_offset': 50.0,   # mm sobre z_pick para aproximación
                    'z_place_offset': 5.0,       # mm sobre z_palet para soltar
                },

                'tag_positions_camera_px': {
                    str(tag_id): {
                        'cx': round(self._camera_points[tag_id][0], 2),
                        'cy': round(self._camera_points[tag_id][1], 2),
                    }
                    for tag_id in calibrated_tags
                },

                'tag_positions_robot_mm': {
                    str(tag_id): {
                        'x': round(self._robot_points[tag_id][0], 2),
                        'y': round(self._robot_points[tag_id][1], 2),
                        'z': round(self._robot_points[tag_id][2], 2),
                    }
                    for tag_id in calibrated_tags
                },
            }
        }

        # Guardar en la carpeta config del paquete source
        src_config_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config'
        )
        output_path = os.path.join(src_config_dir, 'calibration.yaml')
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        with open(output_path, 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False, sort_keys=False)

        print(f'\n  ✓ Calibración guardada en: {output_path}')

        # ─── Resumen ───
        print('\n' + '=' * 60)
        print('  RESUMEN DE CALIBRACIÓN')
        print('=' * 60)
        print(f'  Tags calibrados:    {len(calibrated_tags)}')
        print(f'  Error RMS:          {error_rms:.2f} mm')
        print(f'  Z pick (cinta):     {self._z_pick:.1f} mm' if self._z_pick else '')
        print(f'  Z palet (base):     {self._z_palet:.1f} mm' if self._z_palet else '')
        print(f'  Archivo:            {output_path}')
        print('=' * 60)

        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibrationNode()

    try:
        success = node.run_calibration()
        if success:
            print('\n✓ Calibración completada exitosamente.')
        else:
            print('\n✗ Calibración falló. Revise los errores arriba.')
    except KeyboardInterrupt:
        print('\n\nCalibración cancelada por el usuario.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
