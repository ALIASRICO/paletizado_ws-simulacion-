#!/usr/bin/env python3
"""
=================================================================
Nodo ROS2 de Detección YOLOv8-OBB para Cámara de Gazebo
=================================================================
Detecta cajas de cartón usando YOLOv8-OBB y publica:
  - Imagen con detecciones anotadas
  - Posiciones 3D de objetos detectados
  - Bounding boxes orientados

Topics:
  Subscribed:
    /camera/color/image_raw (sensor_msgs/Image)
    /camera/depth/image_raw (sensor_msgs/Image) - opcional
  
  Published:
    /vision/detections/image (sensor_msgs/Image) - Imagen anotada
    /vision/detections/poses (geometry_msgs/PoseArray) - Poses 3D
    /vision/detections/bboxes (vision_msgs/Detection2DArray)

Uso:
    # Con ROS2:
    ros2 run pruebas_de_vision yolo_detector
    ros2 run pruebas_de_vision yolo_detector --ros-args -p model_path:=/path/to/best.pt
    
    # Con entorno virtual (directo):
    ~/dobot_ws/yolo_venv/bin/python3 yolo_detector.py --model /path/to/best.pt
=================================================================
"""

import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from std_msgs.msg import Header

from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional, List, Tuple
from pathlib import Path

# YOLO imports
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("WARNING: ultralytics no instalado. Ejecuta: pip install ultralytics")


class YOLODetectorNode(Node):
    """
    Nodo de detección YOLOv8-OBB para integración con ROS2.
    
    Detecta objetos en imágenes de cámara y publica resultados
    en múltiples formatos para integración con el sistema de paletizado.
    """
    
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Declarar parámetros
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('publish_poses', True)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('show_window', True)
        
        # Obtener parámetros
        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        self.publish_poses = self.get_parameter('publish_poses').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.show_window = self.get_parameter('show_window').value
        
        # Inicializar CV Bridge
        self.bridge = CvBridge()
        
        # Contador de frames
        self.frame_count = 0
        self.detection_count = 0
        
        # Buscar modelo
        self.model = self._load_model()
        
        # QoS para cámara (best effort para streaming)
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Suscriptores
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            camera_qos
        )
        
        # Depth es opcional
        self.depth_sub = None
        self.latest_depth = None
        
        # Publicadores
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image,
                '/vision/detections/image',
                10
            )
        
        if self.publish_poses:
            self.poses_pub = self.create_publisher(
                PoseArray,
                '/vision/detections/poses',
                10
            )
        
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/vision/detections/bboxes',
            10
        )
        
        # Log de inicio
        self._log_startup_info()
    
    def _load_model(self):
        """Carga el modelo YOLOv8-OBB."""
        if not YOLO_AVAILABLE:
            self.get_logger().error("ultralytics no está instalado!")
            return None
        
        # Buscar modelo en ubicaciones posibles (incluyendo la ruta real del modelo entrenado)
        search_paths = [
            self.model_path,
            # Ruta real del modelo entrenado (nota: tiene runs/obb duplicado)
            str(Path.home() / 'runs/obb/runs/obb/kartonger_yolov8l/weights/best.pt'),
            str(Path.home() / 'runs/obb/runs/obb/kartonger_yolov8l/weights/last.pt'),
            # Rutas alternativas
            'runs/obb/kartonger_yolov8l/weights/best.pt',
            'runs/obb/kartonger_yolov8l/weights/best.onnx',
            str(Path.home() / 'dobot_ws' / 'runs/obb/kartonger_yolov8l/weights/best.pt'),
        ]
        
        for path in search_paths:
            if path and Path(path).exists():
                self.get_logger().info(f"Cargando modelo: {path}")
                return YOLO(path)
        
        # Si no encuentra modelo entrenado, usar modelo preentrenado
        self.get_logger().warning("No se encontró modelo entrenado. Usando yolov8n-obb.pt (preentrenado)")
        return YOLO('yolov8n-obb.pt')
    
    def _log_startup_info(self):
        """Muestra información de inicio."""
        self.get_logger().info("="*60)
        self.get_logger().info("YOLOv8-OBB Detector Node Iniciado")
        self.get_logger().info("="*60)
        self.get_logger().info(f"  Topic de imagen: {self.image_topic}")
        self.get_logger().info(f"  Confianza mínima: {self.conf_threshold}")
        self.get_logger().info(f"  IOU threshold: {self.iou_threshold}")
        self.get_logger().info(f"  Mostrar ventana: {self.show_window}")
        self.get_logger().info(f"  Modelo: {'Cargado' if self.model else 'NO DISPONIBLE'}")
        self.get_logger().info("="*60)
    
    def image_callback(self, msg: Image):
        """
        Callback principal para procesar imágenes de la cámara.
        
        Args:
            msg: Mensaje de imagen ROS2
        """
        if self.model is None:
            return
        
        try:
            self.frame_count += 1
            
            # Convertir a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Ejecutar detección YOLO
            results = self.model.predict(
                source=cv_image,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False,
                device=0,  # GPU
            )
            
            # Procesar resultados
            detections, annotated_image = self._process_results(results[0], cv_image)
            
            # Publicar resultados
            self._publish_detections(detections, msg.header)
            
            if self.publish_annotated and annotated_image is not None:
                self._publish_annotated_image(annotated_image, msg.header)
            
            # Mostrar ventana si está habilitado
            if self.show_window and annotated_image is not None:
                cv2.imshow('YOLOv8-OBB Detections', annotated_image)
                key = cv2.waitKey(1)
                if key == ord('q'):
                    self.get_logger().info("Cerrando por solicitud del usuario...")
                    rclpy.shutdown()
            
            # Log periódico
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f"Frames: {self.frame_count} | Detecciones: {len(detections)}"
                )
        
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")
    
    def _process_results(self, result, image: np.ndarray) -> Tuple[List[dict], np.ndarray]:
        """
        Procesa los resultados de YOLO y extrae detecciones.
        
        Args:
            result: Resultado de YOLO
            image: Imagen original
        
        Returns:
            Tuple de (lista de detecciones, imagen anotada)
        """
        detections = []
        
        # Verificar si hay detecciones OBB
        if result.obb is None or len(result.obb) == 0:
            return detections, image.copy() if self.show_window else None
        
        # Imagen para anotaciones
        annotated = image.copy() if self.show_window or self.publish_annotated else None
        
        # Iterar sobre detecciones OBB
        for i, obb in enumerate(result.obb):
            # Obtener datos de la detección
            confidence = float(obb.conf)
            class_id = int(obb.cls)
            class_name = result.names[class_id] if class_id in result.names else f"class_{class_id}"
            
            # Obtener bounding box orientado (formato: cx, cy, w, h, angle)
            # OBB en formato xywhr (center_x, center_y, width, height, rotation_rad)
            if hasattr(obb, 'xywhr'):
                xywhr = obb.xywhr[0].cpu().numpy()
                cx, cy, w, h, angle = xywhr
            else:
                # Fallback a xyxyxyxy (4 esquinas)
                xyxyxyxy = obb.xyxyxyxy[0].cpu().numpy()
                cx = np.mean(xyxyxyxy[::2])
                cy = np.mean(xyxyxyxy[1::2])
                w = np.linalg.norm(xyxyxyxy[0:2] - xyxyxyxy[2:4])
                h = np.linalg.norm(xyxyxyxy[2:4] - xyxyxyxy[4:6])
                angle = np.arctan2(xyxyxyxy[3] - xyxyxyxy[1], xyxyxyxy[2] - xyxyxyxy[0])
            
            detection = {
                'class_id': class_id,
                'class_name': class_name,
                'confidence': confidence,
                'bbox': {
                    'cx': float(cx),
                    'cy': float(cy),
                    'width': float(w),
                    'height': float(h),
                    'angle_rad': float(angle),
                    'angle_deg': float(np.degrees(angle))
                },
                'corners': self._get_obb_corners(cx, cy, w, h, angle)
            }
            
            detections.append(detection)
            self.detection_count += 1
            
            # Dibujar en imagen anotada
            if annotated is not None:
                self._draw_obb(annotated, detection, i)
        
        return detections, annotated
    
    def _get_obb_corners(self, cx: float, cy: float, w: float, h: float, angle: float) -> List[Tuple[float, float]]:
        """
        Calcula las 4 esquinas del bounding box orientado.
        
        Args:
            cx, cy: Centro del bbox
            w, h: Ancho y alto
            angle: Ángulo de rotación en radianes
        
        Returns:
            Lista de 4 tuplas (x, y) con las esquinas
        """
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        
        # Half dimensions
        hw, hh = w / 2, h / 2
        
        # Esquinas relativas al centro (sin rotar)
        corners_local = [
            (-hw, -hh),
            (hw, -hh),
            (hw, hh),
            (-hw, hh)
        ]
        
        # Rotar y trasladar
        corners = []
        for lx, ly in corners_local:
            x = cx + lx * cos_a - ly * sin_a
            y = cy + lx * sin_a + ly * cos_a
            corners.append((x, y))
        
        return corners
    
    def _draw_obb(self, image: np.ndarray, detection: dict, idx: int):
        """
        Dibuja un bounding box orientado en la imagen.
        
        Args:
            image: Imagen donde dibujar
            detection: Diccionario con datos de detección
            idx: Índice de la detección
        """
        corners = detection['corners']
        
        # Convertir a enteros
        pts = np.array(corners, dtype=np.int32)
        
        # Color basado en clase (verde para cajas)
        color = (0, 255, 0)
        
        # Dibujar polígono
        cv2.polylines(image, [pts], isClosed=True, color=color, thickness=2)
        
        # Dibujar centro
        cx, cy = int(detection['bbox']['cx']), int(detection['bbox']['cy'])
        cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)
        
        # Etiqueta
        label = f"{detection['class_name']}: {detection['confidence']:.2f}"
        
        # Posición de texto (esquina superior izquierda)
        text_pos = (int(corners[0][0]), int(corners[0][1]) - 10)
        
        # Fondo para texto
        (text_w, text_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(image, 
                     (text_pos[0] - 2, text_pos[1] - text_h - 2),
                     (text_pos[0] + text_w + 2, text_pos[1] + baseline),
                     (0, 0, 0), -1)
        
        # Texto
        cv2.putText(image, label, text_pos, 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Dibujar dirección (indicador de orientación)
        angle = detection['bbox']['angle_rad']
        arrow_len = min(detection['bbox']['width'], detection['bbox']['height']) / 2
        end_x = int(cx + arrow_len * np.cos(angle))
        end_y = int(cy + arrow_len * np.sin(angle))
        cv2.arrowedLine(image, (cx, cy), (end_x, end_y), (255, 0, 0), 2)
    
    def _publish_detections(self, detections: List[dict], header: Header):
        """
        Publica las detecciones en formato ROS2.
        
        Args:
            detections: Lista de detecciones procesadas
            header: Header del mensaje original
        """
        # Publicar Detection2DArray
        det_msg = Detection2DArray()
        det_msg.header = header
        
        for det in detections:
            detection = Detection2D()
            detection.bbox.center.position.x = det['bbox']['cx']
            detection.bbox.center.position.y = det['bbox']['cy']
            detection.bbox.size_x = det['bbox']['width']
            detection.bbox.size_y = det['bbox']['height']
            
            # Hipótesis (clase + confianza)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det['class_id'])
            hypothesis.hypothesis.score = det['confidence']
            detection.results.append(hypothesis)
            
            det_msg.detections.append(detection)
        
        self.detections_pub.publish(det_msg)
        
        # Publicar PoseArray si está habilitado
        if self.publish_poses:
            pose_array = PoseArray()
            pose_array.header = header
            pose_array.header.frame_id = self.camera_frame
            
            for det in detections:
                pose = Pose()
                pose.position.x = det['bbox']['cx']
                pose.position.y = det['bbox']['cy']
                pose.position.z = 0.0  # Sin profundidad por ahora
                
                # Orientación desde el ángulo del OBB
                angle = det['bbox']['angle_rad']
                # Convertir a quaternion (rotación en Z)
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = np.sin(angle / 2)
                pose.orientation.w = np.cos(angle / 2)
                
                pose_array.poses.append(pose)
            
            self.poses_pub.publish(pose_array)
    
    def _publish_annotated_image(self, image: np.ndarray, header: Header):
        """
        Publica la imagen anotada.
        
        Args:
            image: Imagen OpenCV anotada
            header: Header del mensaje original
        """
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header = header
        self.annotated_pub.publish(msg)


def parse_args():
    """Parsea argumentos de línea de comandos."""
    parser = argparse.ArgumentParser(
        description='Nodo YOLOv8-OBB Detector para ROS2'
    )
    parser.add_argument(
        '--model', '-m',
        type=str,
        default='',
        help='Ruta al modelo YOLO (.pt o .onnx)'
    )
    parser.add_argument(
        '--confidence', '-c',
        type=float,
        default=0.25,
        help='Umbral de confianza mínima (default: 0.25)'
    )
    parser.add_argument(
        '--iou',
        type=float,
        default=0.5,
        help='Umbral IOU para NMS (default: 0.5)'
    )
    parser.add_argument(
        '--image-topic',
        type=str,
        default='/camera/color/image_raw',
        help='Topic de imagen (default: /camera/color/image_raw)'
    )
    parser.add_argument(
        '--show-window', '-s',
        action='store_true',
        default=True,
        help='Mostrar ventana de visualización'
    )
    parser.add_argument(
        '--no-window',
        action='store_true',
        help='No mostrar ventana de visualización'
    )
    
    return parser.parse_args()


def main(args=None):
    # Parsear argumentos de línea de comandos
    cli_args = parse_args()
    
    # Preparar argumentos para ROS2
    ros_args = []
    if cli_args.model:
        ros_args.extend(['-p', f'model_path:={cli_args.model}'])
    ros_args.extend(['-p', f'confidence_threshold:={cli_args.confidence}'])
    ros_args.extend(['-p', f'iou_threshold:={cli_args.iou}'])
    ros_args.extend(['-p', f'image_topic:={cli_args.image_topic}'])
    
    show_window = cli_args.show_window and not cli_args.no_window
    ros_args.extend(['-p', f'show_window:={show_window}'])
    
    # Inicializar ROS2 con argumentos
    rclpy.init(args=['--ros-args'] + ros_args if ros_args else None)
    
    try:
        node = YOLODetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nInterrumpido por usuario")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
