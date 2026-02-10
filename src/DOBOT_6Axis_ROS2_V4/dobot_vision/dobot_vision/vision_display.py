#!/usr/bin/env python3
"""
vision_display.py — Nodo de Visualización + Coordenadas 3D (ROS2 nativo)

Suscribe a topics ROS2 estándar:
  - /camera/camera/color/image_raw        (imagen RGB de realsense2_camera)
  - /camera/camera/aligned_depth_to_color/image_raw  (depth alineado)
  - /camera/camera/color/camera_info      (intrínsecos para deproyección)
  - /apriltag/detections                  (detecciones de apriltag_ros)

Publica:
  - /vision/tag_poses       (PoseStamped por cada tag, frame world)
  - /vision/all_tag_poses   (PoseArray con todos los tags del frame)
  - /vision/camera_info     (reenvía camera_info para calibración)
  - TF: vision/tag_<id>     (para visualización en RViz)

Muestra ventana OpenCV con:
  - Imagen de la cámara en vivo
  - Bordes de cada tag detectado
  - Centro de cada tag en VERDE
  - ID y coordenadas 3D sobreimpresas

REGLAS:
  #1 Safety First  — bounding box antes de publicar
  #2 Desacoplado   — solo percibe, no decide movimiento
  #3 Configurable  — todo por params ROS2
  #5 Clean Code    — tipado estático, tf2
"""

from typing import Dict, Optional, Deque, Tuple
from collections import deque

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class VisionDisplayNode(Node):
    """Nodo ROS2 de percepción: visualiza cámara, detecta AprilTag, publica poses 3D."""

    def __init__(self) -> None:
        super().__init__('vision_display')

        # ─── Parámetros (REGLA #3) ───
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('tag_size_m', 0.10)
        self.declare_parameter('min_decision_margin', 20.0)
        self.declare_parameter('pose_smoothing_window', 5)
        self.declare_parameter('target_tag_ids', [10, 11, 12, 13, 20, 21, 22, 23])
        self.declare_parameter('show_window', True)
        self.declare_parameter('window_name', 'Dobot Vision')

        # Workspace bounds (REGLA #1)
        self.declare_parameter('workspace_bounds.x_min', -2.0)
        self.declare_parameter('workspace_bounds.x_max', 2.0)
        self.declare_parameter('workspace_bounds.y_min', -2.0)
        self.declare_parameter('workspace_bounds.y_max', 2.0)
        self.declare_parameter('workspace_bounds.z_min', 0.0)
        self.declare_parameter('workspace_bounds.z_max', 4.0)

        # Leer parámetros
        self._ref_frame: str = self.get_parameter('reference_frame').value
        self._cam_frame: str = self.get_parameter('camera_frame').value
        self._tag_size: float = self.get_parameter('tag_size_m').value
        self._min_margin: float = self.get_parameter('min_decision_margin').value
        self._smooth_win: int = self.get_parameter('pose_smoothing_window').value
        self._target_ids: list = self.get_parameter('target_tag_ids').value
        self._show_window: bool = self.get_parameter('show_window').value
        self._window_name: str = self.get_parameter('window_name').value

        self._bounds = {
            'x_min': self.get_parameter('workspace_bounds.x_min').value,
            'x_max': self.get_parameter('workspace_bounds.x_max').value,
            'y_min': self.get_parameter('workspace_bounds.y_min').value,
            'y_max': self.get_parameter('workspace_bounds.y_max').value,
            'z_min': self.get_parameter('workspace_bounds.z_min').value,
            'z_max': self.get_parameter('workspace_bounds.z_max').value,
        }

        # ─── Estado interno ───
        self._latest_image: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._camera_info: Optional[CameraInfo] = None
        self._intrinsics: Optional[Dict] = None  # fx, fy, cx, cy
        self._detections: Optional[AprilTagDetectionArray] = None
        self._pose_history: Dict[int, Deque[Tuple[float, float, float]]] = {}

        # ─── QoS ───
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ─── Subscribers ───
        self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self._image_cb, sensor_qos)
        self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self._depth_cb, sensor_qos)
        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self._camera_info_cb, sensor_qos)
        self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections',
            self._detection_cb, reliable_qos)

        # ─── Publishers ───
        self._pose_pub = self.create_publisher(PoseStamped, '/vision/tag_poses', reliable_qos)
        self._pose_array_pub = self.create_publisher(PoseArray, '/vision/all_tag_poses', reliable_qos)
        self._cam_info_pub = self.create_publisher(CameraInfo, '/vision/camera_info', reliable_qos)
        self._tf_broadcaster = TransformBroadcaster(self)

        # ─── Timer para display (30 Hz) ───
        self._timer = self.create_timer(1.0 / 30.0, self._display_loop)
        self._running = True

        # ─── Ventana OpenCV ───
        if self._show_window:
            cv2.namedWindow(self._window_name, cv2.WINDOW_AUTOSIZE)

        self.get_logger().info('=' * 60)
        self.get_logger().info('DOBOT VISION — Display + 3D Pose Node (ROS2)')
        self.get_logger().info(f'  Tag family: 36h11, size: {self._tag_size}m')
        self.get_logger().info(f'  Target IDs: {self._target_ids}')
        self.get_logger().info(f'  Show window: {self._show_window}')
        self.get_logger().info(f'  Workspace bounds: {self._bounds}')
        self.get_logger().info('=' * 60)

    # ─────────────────────────────────────────────────────────────
    # Callbacks (reciben datos ROS2)
    # ─────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image) -> None:
        """Convierte Image ROS2 a numpy (sin cv_bridge, evita conflicto NumPy2)."""
        try:
            if msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                self._latest_image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                self._latest_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            elif msg.encoding == 'bgra8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
                self._latest_image = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            else:
                # Intentar como RGB genérico
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                self._latest_image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().warn(f'Error convirtiendo imagen: {e}', throttle_duration_sec=5.0)

    def _depth_cb(self, msg: Image) -> None:
        """Convierte depth Image a numpy array (metros)."""
        try:
            if msg.encoding == '16UC1':
                depth_raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                self._latest_depth = depth_raw.astype(np.float32) / 1000.0  # mm → metros
            elif msg.encoding == '32FC1':
                self._latest_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                    msg.height, msg.width)
            else:
                self.get_logger().warn(
                    f'Depth encoding no soportado: {msg.encoding}', throttle_duration_sec=10.0)
        except Exception as e:
            self.get_logger().warn(f'Error convirtiendo depth: {e}', throttle_duration_sec=5.0)

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        """Extrae intrínsecos de camera_info."""
        self._camera_info = msg
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        if len(msg.k) == 9:
            self._intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5],
                'width': msg.width,
                'height': msg.height,
            }
        # Reenviar camera_info para calibración
        self._cam_info_pub.publish(msg)

    def _detection_cb(self, msg: AprilTagDetectionArray) -> None:
        """Almacena las últimas detecciones."""
        self._detections = msg

    # ─────────────────────────────────────────────────────────────
    # Loop principal (display + publicación)
    # ─────────────────────────────────────────────────────────────

    def _display_loop(self) -> None:
        """Timer callback: dibuja detecciones y publica poses 3D."""
        if not self._running:
            return

        if self._latest_image is None:
            return

        display = self._latest_image.copy()
        detections = self._detections
        has_depth = self._latest_depth is not None
        has_intrinsics = self._intrinsics is not None

        # Info en esquina superior
        h, w = display.shape[:2]
        info_text = f'{w}x{h}'
        if has_depth:
            info_text += ' | Depth OK'
        if has_intrinsics:
            info_text += ' | CamInfo OK'
        cv2.putText(display, info_text, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # PoseArray para este frame
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self._cam_frame

        if detections is not None and detections.detections:
            for det in detections.detections:
                tag_id: int = det.id

                # Filtro: ¿es un tag de interés?
                if self._target_ids and tag_id not in self._target_ids:
                    continue

                # Filtro: confianza mínima
                if det.decision_margin < self._min_margin:
                    continue

                # Centro en píxeles
                cx_px = int(det.centre.x)
                cy_px = int(det.centre.y)

                # Esquinas en píxeles (TL, TR, BR, BL)
                corners_px = np.array(
                    [[int(c.x), int(c.y)] for c in det.corners],
                    dtype=np.int32
                )

                # ─── Calcular 3D si tenemos depth + intrínsecos ───
                x3d, y3d, z3d = 0.0, 0.0, 0.0
                has_3d = False

                if has_depth and has_intrinsics:
                    z3d = self._get_depth_at(cx_px, cy_px)
                    if z3d > 0.05:  # mínimo 5cm
                        x3d, y3d, z3d = self._deproject(cx_px, cy_px, z3d)
                        # Suavizar
                        x3d, y3d, z3d = self._smooth(tag_id, x3d, y3d, z3d)
                        has_3d = True

                # ─── REGLA #1: Safety — validar bounding box ───
                if has_3d and not self._in_bounds(x3d, y3d, z3d):
                    self.get_logger().warn(
                        f'Tag {tag_id} FUERA de workspace: '
                        f'({x3d:.3f}, {y3d:.3f}, {z3d:.3f})',
                        throttle_duration_sec=2.0)
                    # Dibujar en rojo si está fuera
                    cv2.polylines(display, [corners_px], True, (0, 0, 255), 2)
                    cv2.circle(display, (cx_px, cy_px), 6, (0, 0, 255), -1)
                    cv2.putText(display, f'ID {tag_id} FUERA', (cx_px - 40, cy_px - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    continue

                # ─── Publicar pose 3D ───
                if has_3d:
                    pose = PoseStamped()
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.header.frame_id = self._cam_frame
                    pose.pose.position.x = float(x3d)
                    pose.pose.position.y = float(y3d)
                    pose.pose.position.z = float(z3d)
                    pose.pose.orientation.w = 1.0  # identidad
                    self._pose_pub.publish(pose)
                    pose_array.poses.append(pose.pose)

                    # Broadcast TF
                    self._broadcast_tf(tag_id, pose)

                # ─── Dibujar en la imagen ───
                color = (0, 255, 0)  # VERDE para tags válidos

                # Bordes del tag
                cv2.polylines(display, [corners_px], True, color, 2)

                # Centro VERDE (punto grande)
                cv2.circle(display, (cx_px, cy_px), 6, color, -1)

                # Texto: ID
                label = f'ID:{tag_id}'
                if has_3d:
                    label += f' ({x3d:.3f},{y3d:.3f},{z3d:.3f})'
                cv2.putText(display, label, (cx_px - 40, cy_px - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

            n_tags = len(detections.detections)
            cv2.putText(display, f'Tags: {n_tags}', (10, h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publicar PoseArray
        if pose_array.poses:
            self._pose_array_pub.publish(pose_array)

        # ─── Mostrar ventana ───
        if self._show_window:
            cv2.imshow(self._window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Cerrando ventana (q pressed)')
                self._running = False
                self._timer.cancel()
                cv2.destroyAllWindows()
                rclpy.shutdown()

    # ─────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────

    def _get_depth_at(self, u: int, v: int, radius: int = 2) -> float:
        """Lee profundidad con ventana promediada alrededor del píxel."""
        depth = self._latest_depth
        h, w = depth.shape
        u = max(radius, min(u, w - radius - 1))
        v = max(radius, min(v, h - radius - 1))

        patch = depth[v - radius:v + radius + 1, u - radius:u + radius + 1]
        valid = patch[(patch > 0.05) & (patch < 10.0)]

        if len(valid) >= 3:
            return float(np.median(valid))
        return 0.0

    def _deproject(self, u: int, v: int, z: float) -> Tuple[float, float, float]:
        """Deproyecta pixel (u,v) + depth → punto 3D en frame cámara."""
        intr = self._intrinsics
        x = (u - intr['cx']) * z / intr['fx']
        y = (v - intr['cy']) * z / intr['fy']
        return x, y, z

    def _smooth(self, tag_id: int, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Moving median para reducir ruido."""
        if tag_id not in self._pose_history:
            self._pose_history[tag_id] = deque(maxlen=self._smooth_win)

        buf = self._pose_history[tag_id]

        # Si hubo salto grande, resetear historial
        if buf:
            lx, ly, lz = buf[-1]
            dist = np.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2)
            if dist > 0.05:  # 5cm de salto → resetear
                buf.clear()

        buf.append((x, y, z))

        if len(buf) < 3:
            return x, y, z

        arr = np.array(list(buf))
        return float(np.median(arr[:, 0])), float(np.median(arr[:, 1])), float(np.median(arr[:, 2]))

    def _in_bounds(self, x: float, y: float, z: float) -> bool:
        """REGLA #1: Valida que el punto esté dentro del workspace."""
        b = self._bounds
        return (b['x_min'] <= x <= b['x_max']
                and b['y_min'] <= y <= b['y_max']
                and b['z_min'] <= z <= b['z_max'])

    def _broadcast_tf(self, tag_id: int, pose: PoseStamped) -> None:
        """Publica TF del tag para RViz."""
        t = TransformStamped()
        t.header.stamp = pose.header.stamp
        t.header.frame_id = self._cam_frame
        t.child_frame_id = f'vision/tag_{tag_id}'
        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        t.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)

    def destroy_node(self) -> None:
        self._running = False
        if self._show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down vision_display...')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
