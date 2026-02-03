#!/usr/bin/env python3
"""
ROS2 USB Camera Node
Captures images from USB camera and publishes to ROS2 topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_index', 2)
        self.declare_parameter('camera_width', 1920)
        self.declare_parameter('camera_height', 1080)
        self.declare_parameter('publish_rate', 30.0)
        
        # Get parameters
        self.camera_index = self.get_parameter('camera_index').value
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # CV Bridge for converting OpenCV images to ROS messages
        self.bridge = CvBridge()
        
        # Publisher
        self.image_pub = self.create_publisher(
            Image, 
            'camera/image_raw', 
            10
        )
        
        # Open camera
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'USB Kamera açılamadı: /dev/video{self.camera_index}')
            raise RuntimeError('Kamera açılamadı')
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        
        # Log camera info
        self.log_camera_info()
        
        # Create timer for publishing images
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'USB Camera Node başlatıldı')
        self.get_logger().info(f'Kamera: /dev/video{self.camera_index}')
        self.get_logger().info(f'Çözünürlük: {self.camera_width}x{self.camera_height}')
        self.get_logger().info(f'Yayın hızı: {self.publish_rate} Hz')
        self.get_logger().info('Topic: /camera/image_raw (kamera görüntüsü)')
        
    def log_camera_info(self):
        """Log camera parameters"""
        self.get_logger().info('=== Kamera Parametreleri ===')
        self.get_logger().info(f'Genişlik: {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}')
        self.get_logger().info(f'Yükseklik: {self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}')
        self.get_logger().info(f'FPS: {self.cap.get(cv2.CAP_PROP_FPS)}')
        self.get_logger().info(f'Parlaklık: {self.cap.get(cv2.CAP_PROP_BRIGHTNESS)}')
        self.get_logger().info(f'Kontrast: {self.cap.get(cv2.CAP_PROP_CONTRAST)}')
        self.get_logger().info(f'Doygunluk: {self.cap.get(cv2.CAP_PROP_SATURATION)}')
        self.get_logger().info('===========================')
    
    def timer_callback(self):
        """Timer callback to capture and publish images"""
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Kameradan görüntü alınamadı')
            return
        
        try:
            # Publish image
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            self.image_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Görüntü yayınlama hatası: {str(e)}')
    
    def destroy_node(self):
        """Clean up resources"""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = USBCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Hata: {str(e)}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
