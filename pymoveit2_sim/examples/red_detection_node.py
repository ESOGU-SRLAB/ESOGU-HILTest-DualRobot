#!/usr/bin/env python3
"""
Red Detection Node - Kamera görüntüsünden kırmızı renk tespiti yapar.
PointCloud2 kullanarak tespit edilen noktaların 3D world koordinatlarını bulur.
"""

from threading import Thread, Lock
import time
import math
import numpy as np
import os
from datetime import datetime
import csv
import struct

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseArray, Pose, PointStamped
from std_msgs.msg import String, Int32, Header

import tf2_ros
from tf2_geometry_msgs import do_transform_point

# OpenCV
try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False
    print("UYARI: OpenCV yüklü değil!")


def imgmsg_to_cv2_manual(img_msg):
    """ROS Image mesajını OpenCV formatına dönüştür"""
    dtype = np.uint8
    if img_msg.encoding == 'rgb8':
        img = np.frombuffer(img_msg.data, dtype=dtype).reshape(img_msg.height, img_msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == 'bgr8':
        return np.frombuffer(img_msg.data, dtype=dtype).reshape(img_msg.height, img_msg.width, 3)
    elif img_msg.encoding == 'mono8':
        return np.frombuffer(img_msg.data, dtype=dtype).reshape(img_msg.height, img_msg.width)
    elif img_msg.encoding == 'rgba8':
        img = np.frombuffer(img_msg.data, dtype=dtype).reshape(img_msg.height, img_msg.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif img_msg.encoding == 'bgra8':
        img = np.frombuffer(img_msg.data, dtype=dtype).reshape(img_msg.height, img_msg.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    else:
        raise ValueError(f"Desteklenmeyen encoding: {img_msg.encoding}")


class RedDetectionNode(Node):
    def __init__(self):
        super().__init__("red_detection_node")
        
        # Parametreler
        self.declare_parameter('image_topic', '/sim/image')
        self.declare_parameter('pointcloud_topic', '/sim/pointcloud')
        self.declare_parameter('camera_frame', 'sim_ur10e_rgb_optical_frame')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('output_dir', '/home/cem/colcon_ws/src/pymoveit2_sim/examples/detection_results')
        self.declare_parameter('min_red_area', 500)  # Minimum kırmızı alan (piksel)
        self.declare_parameter('duplicate_distance', 0.20)  # Aynı nokta kabul mesafesi (metre)
        
        self.image_topic = self.get_parameter('image_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.output_dir = self.get_parameter('output_dir').value
        self.min_red_area = self.get_parameter('min_red_area').value
        self.duplicate_distance = self.get_parameter('duplicate_distance').value
        
        # Çıktı dizinini oluştur
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Callback group
        callback_group = ReentrantCallbackGroup()
        
        # TF2 Buffer ve Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ==================== PUBLISHERS ====================
        self.detected_points_publisher = self.create_publisher(
            PoseArray, '/detected_red_points', 10
        )
        self.status_publisher = self.create_publisher(
            String, '/red_detection/status', 10
        )
        
        # ==================== SUBSCRIBERS ====================
        # Image subscriber
        if CV_AVAILABLE:
            self.image_subscription = self.create_subscription(
                Image, self.image_topic, self.image_callback, 10,
                callback_group=callback_group
            )
        
        # PointCloud2 subscriber
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2, self.pointcloud_topic, self.pointcloud_callback, 10,
            callback_group=callback_group
        )
        
        # Waypoint indeksi dinle
        self.waypoint_subscription = self.create_subscription(
            Int32, '/sensing_robot/current_waypoint', self.waypoint_callback, 10,
            callback_group=callback_group
        )
        
        # Robot durumu dinle
        self.robot_status_subscription = self.create_subscription(
            String, '/sensing_robot/status', self.robot_status_callback, 10,
            callback_group=callback_group
        )
        
        # Progress status dinle - SENSING COMPLETED gelince kapan
        self.progress_subscription = self.create_subscription(
            String, '/progress_status', self.progress_callback, 10,
            callback_group=callback_group
        )
        
        # Kapanma flag'i
        self.shutdown_requested = False
        
        # State değişkenleri
        self.latest_image = None
        self.latest_pointcloud = None
        self.pointcloud_width = 0
        self.pointcloud_height = 0
        self.current_waypoint = 0
        self.robot_status = "UNKNOWN"
        self.is_scanning = False
        self.image_received = False
        self.pointcloud_received = False
        self.total_detections = 0
        
        # Tespit edilen noktalar
        self.detected_points = []  # World frame'deki 3D noktalar
        self.detected_points_lock = Lock()
        
        # CSV dosyası
        self.csv_filename = os.path.join(
            self.output_dir, 
            f"red_points_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        self._init_csv()
        
        # Detection timer
        self.detection_timer = self.create_timer(0.5, self.detection_loop)
        
        # Status timer
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        self.get_logger().info("=== RED DETECTION STARTED ===")
    
    def _init_csv(self):
        """CSV dosyasını başlat"""
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'waypoint_index', 
                'pixel_x', 'pixel_y', 'area',
                'world_x', 'world_y', 'world_z'
            ])
    
    def pointcloud_callback(self, msg: PointCloud2):
        """PointCloud2 mesajını al ve sakla"""
        self.latest_pointcloud = msg
        self.pointcloud_width = msg.width
        self.pointcloud_height = msg.height
        self.pointcloud_received = True
    
    def image_callback(self, msg):
        """Kamera görüntüsünü al"""
        if not CV_AVAILABLE:
            return
        try:
            self.latest_image = imgmsg_to_cv2_manual(msg)
            self.image_received = True
        except Exception as e:
            pass
    
    def waypoint_callback(self, msg):
        """Waypoint indeksini güncelle"""
        self.current_waypoint = msg.data
    
    def robot_status_callback(self, msg: String):
        """Robot durumunu güncelle"""
        self.robot_status = msg.data
        self.is_scanning = "SCANNING" in msg.data or "MOVING" not in msg.data
    
    def progress_callback(self, msg: String):
        """Progress status dinle - SENSING COMPLETED gelince kapan"""
        if msg.data == "SENSING COMPLETED" and not self.shutdown_requested:
            self.shutdown_requested = True
            self.get_logger().info(f"=== RED DETECTION COMPLETED ({len(self.detected_points)} points) ===")
            
            # Final raporu kaydet
            self.save_final_report()
            
            # Timer'ları durdur
            self.detection_timer.cancel()
            self.status_timer.cancel()
            
            # Node'u kapat
            raise SystemExit(0)
    
    def publish_status(self):
        """Durum yayınla"""
        msg = String()
        with self.detected_points_lock:
            point_count = len(self.detected_points)
        msg.data = f"DETECTED:{point_count}|IMAGE:{self.image_received}|PC:{self.pointcloud_received}"
        self.status_publisher.publish(msg)
    
    def detection_loop(self):
        """Periyodik kırmızı renk tespiti - PointCloud2 ile 3D koordinat"""
        # Hem image hem pointcloud gerekli
        if self.latest_image is None or self.latest_pointcloud is None:
            return
        
        # Kırmızı renk tespiti yap
        detected = self.detect_red_color()
        self.total_detections += 1
        
        if not detected:
            return
        
        for point in detected:
            pixel_x, pixel_y = point['pixel']
            area = point['area']
            
            # PointCloud2'den 3D koordinat al (kamera frame'de)
            point_3d_camera = self.get_point_from_pointcloud(pixel_x, pixel_y)
            
            if point_3d_camera is None:
                continue
            
            # Kamera frame'den world frame'e dönüştür
            world_point = self.transform_to_world(point_3d_camera)
            
            if world_point is None:
                continue
            
            # Duplicate kontrolü
            if self.is_duplicate(world_point):
                continue
            
            # Yeni nokta kaydet
            point_data = {
                'timestamp': datetime.now().isoformat(),
                'waypoint_index': self.current_waypoint,
                'pixel': (pixel_x, pixel_y),
                'area': area,
                'world_position': world_point
            }
            
            with self.detected_points_lock:
                self.detected_points.append(point_data)
            
            # CSV'ye yaz
            self._write_to_csv(point_data)
            
            # Yayınla
            self._publish_detected_points()
    
    def get_point_from_pointcloud(self, pixel_x: int, pixel_y: int):
        """
        PointCloud2'den belirli bir pikselin 3D koordinatını al.
        """
        if self.latest_pointcloud is None:
            return None
        
        pc = self.latest_pointcloud
        
        # Sınır kontrolü
        if pixel_x < 0 or pixel_x >= pc.width or pixel_y < 0 or pixel_y >= pc.height:
            return None
        
        # Point indeksini hesapla
        point_index = pixel_y * pc.width + pixel_x
        
        # Field offset'lerini bul
        x_offset = None
        y_offset = None
        z_offset = None
        
        for field in pc.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if x_offset is None or y_offset is None or z_offset is None:
            return None
        
        # Point'in başlangıç byte'ı
        point_start = point_index * pc.point_step
        
        # X, Y, Z değerlerini oku (float32)
        try:
            x = struct.unpack_from('f', pc.data, point_start + x_offset)[0]
            y = struct.unpack_from('f', pc.data, point_start + y_offset)[0]
            z = struct.unpack_from('f', pc.data, point_start + z_offset)[0]
            
            # NaN kontrolü
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                # Çevredeki piksellere bak
                return self._get_nearby_valid_point(pixel_x, pixel_y)
            
            return (x, y, z)
            
        except Exception as e:
            return None
    
    def _get_nearby_valid_point(self, pixel_x: int, pixel_y: int, search_radius: int = 5):
        """NaN olan piksel için çevredeki geçerli noktayı bul"""
        pc = self.latest_pointcloud
        
        for r in range(1, search_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = pixel_x + dx, pixel_y + dy
                    if 0 <= nx < pc.width and 0 <= ny < pc.height:
                        point_index = ny * pc.width + nx
                        point_start = point_index * pc.point_step
                        
                        try:
                            x = struct.unpack_from('f', pc.data, point_start + 0)[0]
                            y = struct.unpack_from('f', pc.data, point_start + 4)[0]
                            z = struct.unpack_from('f', pc.data, point_start + 8)[0]
                            
                            if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                                return (x, y, z)
                        except:
                            continue
        return None
    
    def transform_to_world(self, point_camera):
        """Kamera frame'deki noktayı world frame'e dönüştür"""
        try:
            # PointCloud2'nin frame_id'sini kullan
            source_frame = self.latest_pointcloud.header.frame_id
            
            # TF dönüşümü al
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # PointStamped oluştur
            point_stamped = PointStamped()
            point_stamped.header.frame_id = source_frame
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = float(point_camera[0])
            point_stamped.point.y = float(point_camera[1])
            point_stamped.point.z = float(point_camera[2])
            
            # Dönüştür
            world_point = do_transform_point(point_stamped, transform)
            
            return (
                world_point.point.x,
                world_point.point.y,
                world_point.point.z
            )
            
        except Exception as e:
            return None
    
    def is_duplicate(self, world_point):
        """Bu nokta daha önce tespit edilmiş mi kontrol et"""
        with self.detected_points_lock:
            for existing in self.detected_points:
                existing_world = existing.get('world_position')
                if existing_world is None:
                    continue
                
                # Öklid mesafesi
                dist = math.sqrt(
                    (world_point[0] - existing_world[0])**2 +
                    (world_point[1] - existing_world[1])**2 +
                    (world_point[2] - existing_world[2])**2
                )
                
                if dist < self.duplicate_distance:
                    return True
        return False
    
    def detect_red_color(self):
        """Kırmızı renk tespiti yap"""
        if self.latest_image is None:
            return []
        
        try:
            image = self.latest_image.copy()
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Kırmızı renk maskeleri
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2
            
            # Gürültü azaltma
            kernel = np.ones((7, 7), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Konturları bul
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            detected = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.min_red_area:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        detected.append({
                            'pixel': (cx, cy),
                            'area': area
                        })
            
            return detected
            
        except Exception as e:
            return []
    
    def _write_to_csv(self, point_data):
        """CSV'ye yaz"""
        try:
            with open(self.csv_filename, 'a', newline='') as f:
                writer = csv.writer(f)
                world = point_data.get('world_position', (0, 0, 0))
                writer.writerow([
                    point_data['timestamp'],
                    point_data['waypoint_index'],
                    point_data['pixel'][0],
                    point_data['pixel'][1],
                    point_data['area'],
                    world[0],
                    world[1],
                    world[2]
                ])
        except Exception as e:
            pass
    
    def _publish_detected_points(self):
        """Tespit edilen noktaları PoseArray olarak yayınla"""
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.world_frame
        
        with self.detected_points_lock:
            for point_data in self.detected_points:
                world = point_data.get('world_position')
                if world is None:
                    continue
                    
                pose = Pose()
                pose.position.x = world[0]
                pose.position.y = world[1]
                pose.position.z = world[2]
                pose.orientation.w = 1.0
                
                pose_array.poses.append(pose)
        
        self.detected_points_publisher.publish(pose_array)
    
    def save_final_report(self):
        """Final raporu kaydet"""
        report_file = os.path.join(
            self.output_dir, 
            f"report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        )
        
        with self.detected_points_lock:
            with open(report_file, 'w') as f:
                f.write("="*60 + "\n")
                f.write("KIRMIZI NOKTA TESPİT RAPORU (PointCloud2)\n")
                f.write(f"Tarih: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("="*60 + "\n\n")
                
                f.write(f"Toplam tespit: {len(self.detected_points)}\n")
                f.write(f"Toplam tarama: {self.total_detections}\n\n")
                
                for i, point in enumerate(self.detected_points, 1):
                    world = point.get('world_position', (0, 0, 0))
                    f.write(f"--- Nokta {i} ---\n")
                    f.write(f"Waypoint: {point['waypoint_index']}\n")
                    f.write(f"Piksel: {point['pixel']}\n")
                    f.write(f"Alan: {point['area']}\n")
                    f.write(f"World: ({world[0]:.4f}, {world[1]:.4f}, {world[2]:.4f})\n\n")
        
        return report_file


def main():
    rclpy.init()
    
    node = RedDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.shutdown_requested:
            node.save_final_report()
    except SystemExit:
        pass
    finally:
        with node.detected_points_lock:
            pass
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
