#!/usr/bin/env python3
"""
pose_publisher.py — Perception Layer (Dobot CR20 Palletizing System)

Recibe detecciones de apriltag_ros, filtra por confianza y espacio de trabajo,
aplica suavizado, y publica:
  - PoseStamped en /vision/tag_poses (para el Coordinador)
  - TF transforms (para visualización en RViz y cálculos tf2)

Arquitectura:  apriltag_ros → [pose_publisher] → Coordinador
               (2D+3D detect)   (filtra, valida)   (decide acción)

REGLAS APLICADAS:
  #1 Safety First  — bounding box validation antes de publicar
  #2 Desacoplado   — solo publica poses, no toma decisiones de movimiento
  #3 Configurable  — todos los umbrales vienen de params ROS2
  #5 Clean Code    — tipado estático, tf2 oficial
"""

from typing import Dict, List, Optional, Deque
from collections import deque
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

# Para transformar poses entre frames
import tf2_geometry_msgs  # noqa: F401  (registra el converter)


class PosePublisherNode(Node):
    """Nodo de percepción: procesa detecciones AprilTag y publica poses validadas."""

    def __init__(self) -> None:
        super().__init__('pose_publisher')

        # ─── Declarar Parámetros (REGLA #3: todo configurable) ───
        self.declare_parameter('detection_topic', '/apriltag/detections')
        self.declare_parameter('pose_topic', '/vision/tag_poses')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('min_decision_margin', 30.0)
        self.declare_parameter('pose_smoothing_window', 5)
        self.declare_parameter('max_publish_rate', 10.0)
        self.declare_parameter('target_tag_ids', [10, 11, 12, 13, 20, 21, 22, 23])
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('verbose', False)

        # Workspace bounds (REGLA #1: Safety First)
        self.declare_parameter('workspace_bounds.x_min', -1.5)
        self.declare_parameter('workspace_bounds.x_max', 1.5)
        self.declare_parameter('workspace_bounds.y_min', -1.5)
        self.declare_parameter('workspace_bounds.y_max', 1.5)
        self.declare_parameter('workspace_bounds.z_min', 0.0)
        self.declare_parameter('workspace_bounds.z_max', 2.5)

        # ─── Leer Parámetros ───
        self._detection_topic: str = self.get_parameter('detection_topic').value
        self._pose_topic: str = self.get_parameter('pose_topic').value
        self._reference_frame: str = self.get_parameter('reference_frame').value
        self._camera_frame: str = self.get_parameter('camera_frame').value
        self._min_margin: float = self.get_parameter('min_decision_margin').value
        self._smooth_window: int = self.get_parameter('pose_smoothing_window').value
        self._max_rate: float = self.get_parameter('max_publish_rate').value
        self._target_ids: List[int] = self.get_parameter('target_tag_ids').value
        self._publish_tf: bool = self.get_parameter('publish_tf').value
        self._verbose: bool = self.get_parameter('verbose').value

        self._bounds = {
            'x_min': self.get_parameter('workspace_bounds.x_min').value,
            'x_max': self.get_parameter('workspace_bounds.x_max').value,
            'y_min': self.get_parameter('workspace_bounds.y_min').value,
            'y_max': self.get_parameter('workspace_bounds.y_max').value,
            'z_min': self.get_parameter('workspace_bounds.z_min').value,
            'z_max': self.get_parameter('workspace_bounds.z_max').value,
        }

        # ─── TF2 ───
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        # ─── Smoothing buffers: tag_id → deque de PoseStamped ───
        self._pose_buffers: Dict[int, Deque[PoseStamped]] = {}

        # ─── Rate limiting ───
        self._last_publish_time: Dict[int, float] = {}
        self._min_period: float = 1.0 / self._max_rate if self._max_rate > 0 else 0.0

        # ─── Publishers ───
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self._pose_pub = self.create_publisher(PoseStamped, self._pose_topic, qos)
        self._pose_array_pub = self.create_publisher(
            PoseArray, '/vision/all_tag_poses', qos
        )

        # ─── Subscriber ───
        self._sub = self.create_subscription(
            AprilTagDetectionArray,
            self._detection_topic,
            self._detection_callback,
            qos,
        )

        # ─── Log de inicio ───
        self.get_logger().info('=' * 60)
        self.get_logger().info('DOBOT VISION — Pose Publisher Node')
        self.get_logger().info(f'  Subscribed to: {self._detection_topic}')
        self.get_logger().info(f'  Publishing to: {self._pose_topic}')
        self.get_logger().info(f'  Target tag IDs: {self._target_ids}')
        self.get_logger().info(f'  Reference frame: {self._reference_frame}')
        self.get_logger().info(f'  Min decision margin: {self._min_margin}')
        self.get_logger().info(f'  Smoothing window: {self._smooth_window}')
        self.get_logger().info(f'  Workspace bounds: {self._bounds}')
        self.get_logger().info('=' * 60)

    # ─────────────────────────────────────────────────────────────
    # Callback principal
    # ─────────────────────────────────────────────────────────────
    def _detection_callback(self, msg: AprilTagDetectionArray) -> None:
        """Procesa cada frame de detecciones AprilTag."""
        if not msg.detections:
            return

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self._reference_frame

        for detection in msg.detections:
            tag_id: int = detection.id

            # ─── Filtro 1: ¿Es un tag de interés? ───
            if self._target_ids and tag_id not in self._target_ids:
                if self._verbose:
                    self.get_logger().debug(f'Tag {tag_id} ignorado (no en target_ids)')
                continue

            # ─── Filtro 2: Confianza mínima ───
            if detection.decision_margin < self._min_margin:
                if self._verbose:
                    self.get_logger().debug(
                        f'Tag {tag_id} descartado: margin={detection.decision_margin:.1f} '
                        f'< {self._min_margin}'
                    )
                continue

            # ─── Obtener pose 3D del tag via TF ───
            # apriltag_ros publica TF de cada tag detectado.
            # Leemos la transform del tag en el frame de referencia.
            tag_frame = self._get_tag_frame_name(tag_id)
            pose_stamped = self._lookup_tag_pose(tag_frame, msg.header.stamp)

            if pose_stamped is None:
                if self._verbose:
                    self.get_logger().debug(
                        f'Tag {tag_id}: TF "{tag_frame}" no disponible aún'
                    )
                continue

            # ─── Filtro 3: SEGURIDAD — bounding box (REGLA #1) ───
            if not self._is_within_workspace(pose_stamped):
                self.get_logger().warn(
                    f'⚠ Tag {tag_id} FUERA del workspace: '
                    f'({pose_stamped.pose.position.x:.3f}, '
                    f'{pose_stamped.pose.position.y:.3f}, '
                    f'{pose_stamped.pose.position.z:.3f}) — DESCARTADO'
                )
                continue

            # ─── Suavizado ───
            smoothed = self._smooth_pose(tag_id, pose_stamped)

            # ─── Rate limiting ───
            now_sec = self.get_clock().now().nanoseconds / 1e9
            last = self._last_publish_time.get(tag_id, 0.0)
            if self._min_period > 0 and (now_sec - last) < self._min_period:
                continue
            self._last_publish_time[tag_id] = now_sec

            # ─── Publicar pose individual ───
            self._pose_pub.publish(smoothed)

            # ─── Publicar TF ───
            if self._publish_tf:
                self._broadcast_tag_tf(tag_id, smoothed)

            # Agregar al PoseArray
            pose_array.poses.append(smoothed.pose)

            if self._verbose:
                p = smoothed.pose.position
                self.get_logger().info(
                    f'Tag {tag_id}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})'
                )

        # Publicar array de todas las poses válidas del frame
        if pose_array.poses:
            self._pose_array_pub.publish(pose_array)

    # ─────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────
    def _get_tag_frame_name(self, tag_id: int) -> str:
        """Genera el nombre del frame TF de un tag según apriltag_ros."""
        # apriltag_ros publica frames con el nombre configurado en tags_36h11.yaml
        # o como "tag_<family>:<id>" por defecto
        return f'tag36h11:{tag_id}'

    def _lookup_tag_pose(
        self, tag_frame: str, stamp: 'rclpy.time.Time'
    ) -> Optional[PoseStamped]:
        """Busca la pose de un tag en el frame de referencia vía tf2."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._reference_frame,
                tag_frame,
                Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self._reference_frame
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except Exception:
            return None

    def _is_within_workspace(self, pose: PoseStamped) -> bool:
        """REGLA #1 — Safety First: valida que la pose esté dentro del bounding box."""
        p = pose.pose.position
        b = self._bounds
        return (
            b['x_min'] <= p.x <= b['x_max']
            and b['y_min'] <= p.y <= b['y_max']
            and b['z_min'] <= p.z <= b['z_max']
        )

    def _smooth_pose(self, tag_id: int, pose: PoseStamped) -> PoseStamped:
        """Promedia las últimas N poses para reducir ruido (moving average)."""
        if tag_id not in self._pose_buffers:
            self._pose_buffers[tag_id] = deque(maxlen=self._smooth_window)

        self._pose_buffers[tag_id].append(pose)
        buf = self._pose_buffers[tag_id]

        if len(buf) == 1:
            return pose

        # Promediar posición (para orientación usamos la más reciente,
        # promediar quaternions requiere SLERP que es complejo aquí)
        avg = PoseStamped()
        avg.header = pose.header
        avg.pose.position.x = sum(p.pose.position.x for p in buf) / len(buf)
        avg.pose.position.y = sum(p.pose.position.y for p in buf) / len(buf)
        avg.pose.position.z = sum(p.pose.position.z for p in buf) / len(buf)
        # Orientación: usar la última (más reciente)
        avg.pose.orientation = pose.pose.orientation

        return avg

    def _broadcast_tag_tf(self, tag_id: int, pose: PoseStamped) -> None:
        """Publica TF del tag suavizado para visualización en RViz."""
        t = TransformStamped()
        t.header.stamp = pose.header.stamp
        t.header.frame_id = self._reference_frame
        t.child_frame_id = f'vision/tag_{tag_id}'
        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        t.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main(args: Optional[list] = None) -> None:
    """Entry point."""
    rclpy.init(args=args)
    node = PosePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down pose_publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
