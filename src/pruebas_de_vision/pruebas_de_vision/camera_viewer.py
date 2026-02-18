#!/usr/bin/env python3
"""
Script simple para visualizar la cámara de Gazebo usando OpenCV.
No depende de rqt_image_view ni rviz.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Topic del bridge ROS2-Gazebo
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Topic del bridge
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.frame_count = 0
        self.get_logger().info('='*50)
        self.get_logger().info('Camera viewer iniciado')
        self.get_logger().info('Topic: /camera/color/image_raw')
        self.get_logger().info('Esperando imágenes de Gazebo...')
        self.get_logger().info('Presiona "q" para salir')
        self.get_logger().info('='*50)
        
    def image_callback(self, msg):
        try:
            self.frame_count += 1
            
            # Convertir mensaje ROS a imagen OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Añadir contador en la imagen
            cv2.putText(cv_image, f'Frame: {self.frame_count}', (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(cv_image, f'{msg.width}x{msg.height}', (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Mostrar imagen
            cv2.imshow('Camera View - Gazebo', cv_image)
            
            # Log cada 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Recibidos {self.frame_count} frames')
            
            # Esperar tecla (1ms)
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.get_logger().info('Cerrando viewer...')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    viewer = CameraViewer()
    
    # Mostrar mensaje mientras espera
    print('\n' + '='*50)
    print('ESPERANDO IMAGENES...')
    print('Topic: /camera/color/image_raw')
    print('='*50 + '\n')
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        print('\nInterrumpido por usuario')
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
