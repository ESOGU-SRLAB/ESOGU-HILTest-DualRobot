#!/usr/bin/env python3
"""
Red detection / mock perception - v4
- SR_MODE sırasında kırmızı tespit eder
- PointCloud2 + TF ile world koordinatı çıkarır
- /harmony/mock_perception/defect (tekil JSON) yayınlar
- duplicate filtre uygular (yayını azaltır)
- /harmony/log üretir
"""

from __future__ import annotations

from threading import Lock
from datetime import datetime
import time
import json
import math
import struct
from typing import Optional, Tuple, Dict, Any, List

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

import tf2_ros
from tf2_geometry_msgs import do_transform_point

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


def _now_ros(node: Node) -> str:
    t = node.get_clock().now().to_msg()
    return f"{t.sec}.{t.nanosec:09d}"


def imgmsg_to_cv2_manual(img_msg):
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
    raise ValueError(f"Unsupported encoding: {img_msg.encoding}")


class HarmonyRedDetectionV4(Node):
    def __init__(self):
        super().__init__("harmony_red_detection")

        self.declare_parameter("image_topic", "/sim/image")
        self.declare_parameter("pointcloud_topic", "/sim/pointcloud")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("min_red_area", 500)
        self.declare_parameter("duplicate_distance", 1.0)  # 60cm parça + tolerans için 1.0m
        self.declare_parameter("publish_rate_limit_hz", 2.0)  # spam engelle
        self.declare_parameter("duplicate_time_window_sec", 120.0)  # aynı nesneye tekrar yayın penceresi
        self.declare_parameter("duplicate_pixel_distance", 25)  # piksel bazlı duplicate filtresi
        self.declare_parameter("detection_ttl_sec", 600.0)  # cache temizliği (SR dışına çıkınca da tut)
        self.declare_parameter("clear_cache_on_exit_sr_mode", False)
        self.declare_parameter("merge_pixel_distance", 100)  # yakın contour'ları birleştirme mesafesi

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.min_red_area = int(self.get_parameter("min_red_area").value)
        self.duplicate_distance = float(self.get_parameter("duplicate_distance").value)
        self.publish_rate_limit_hz = float(self.get_parameter("publish_rate_limit_hz").value)

        self.duplicate_time_window_sec = float(self.get_parameter("duplicate_time_window_sec").value)
        self.duplicate_pixel_distance = int(self.get_parameter("duplicate_pixel_distance").value)
        self.detection_ttl_sec = float(self.get_parameter("detection_ttl_sec").value)
        self.clear_cache_on_exit_sr_mode = bool(self.get_parameter("clear_cache_on_exit_sr_mode").value)
        self.merge_pixel_distance = int(self.get_parameter("merge_pixel_distance").value)

        cbg = ReentrantCallbackGroup()

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.defect_pub = self.create_publisher(String, "/harmony/mock_perception/defect", 10)

        # Subscribers
        if CV_AVAILABLE:
            self.image_sub = self.create_subscription(Image, self.image_topic, self._image_cb, 10, callback_group=cbg)
        else:
            self._log("WARN", "CV_MISSING", "OpenCV not available; red detection disabled", {})

        self.pc_sub = self.create_subscription(PointCloud2, self.pointcloud_topic, self._pc_cb, 10, callback_group=cbg)
        self.robot_status_sub = self.create_subscription(String, "/harmony/robot_status", self._robot_status_cb, 10, callback_group=cbg)

        # State
        self.latest_image = None
        self.latest_pc: Optional[PointCloud2] = None
        self.in_sr_mode = False

        self.lock = Lock()
        self.detected_cache: List[Dict[str, Any]] = []  # {t, px, py, x, y, z}
        self.last_pub_time = 0.0

        self.timer = self.create_timer(0.2, self._tick)

        self.get_logger().info(f"HarmonyRedDetectionV4 initialized, world_frame={self.world_frame}")
        self.get_logger().info(f"  duplicate_distance={self.duplicate_distance}m, duplicate_pixel_distance={self.duplicate_pixel_distance}px")

    def _robot_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            new_in_sr = (str(data.get("state")) == "SR_MODE")
            # SR_MODE dışına çıkınca cache'i otomatik silmek bazen aynı defekti tekrar yayınlatır
            # (SR_MODE -> WAITING -> SR_MODE gibi geçişlerde). İsteğe bağlı tutuyoruz.
            if self.in_sr_mode and (not new_in_sr) and self.clear_cache_on_exit_sr_mode:
                with self.lock:
                    self.detected_cache.clear()
            self.in_sr_mode = new_in_sr
        except Exception:
            self.in_sr_mode = False

    def _pc_cb(self, msg: PointCloud2):
        self.latest_pc = msg

    def _image_cb(self, msg: Image):
        if not CV_AVAILABLE:
            return
        try:
            self.latest_image = imgmsg_to_cv2_manual(msg)
        except Exception:
            pass

    def _tick(self):
        if not self.in_sr_mode:
            return
        if self.latest_image is None or self.latest_pc is None:
            return

        # rate limit
        now = time.time()
        if self.publish_rate_limit_hz > 0:
            min_dt = 1.0 / self.publish_rate_limit_hz
            if now - self.last_pub_time < min_dt:
                return

        detected = self._detect_red(self.latest_image)
        if not detected:
            return

        detected.sort(key=lambda d: d["area"], reverse=True)
        best = detected[0]
        px, py = best["pixel"]

        p_cam = self._pc_point(px, py)
        if p_cam is None:
            return
        p_world = self._to_world(p_cam)
        if p_world is None:
            return

        now_wall = time.time()
        self._purge_cache(now_wall)

        if self._is_duplicate(p_world, (px, py), now_wall):
            return

        with self.lock:
            self.detected_cache.append({
                "t": now_wall,
                "px": int(px),
                "py": int(py),
                "x": float(p_world[0]),
                "y": float(p_world[1]),
                "z": float(p_world[2]),
            })

        defect_payload = {
            "timestamp": datetime.now().isoformat(),
            "defect_id": f"defect_{int(time.time())}",
            "defect_type": "PATCH",
            "severity_color": "RED",
            "frame_id": self.world_frame,
            "position": {"x": float(p_world[0]), "y": float(p_world[1]), "z": float(p_world[2])},
            "pixel": {"x": int(px), "y": int(py)},
            "area": float(best["area"]),
        }

        out = String()
        out.data = json.dumps(defect_payload)
        self.defect_pub.publish(out)
        self.last_pub_time = now

        self.get_logger().info(f"Published defect: {defect_payload['defect_id']}, pos={defect_payload['position']}, area={defect_payload['area']}")

    def _detect_red(self, image_bgr):
        try:
            hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

            kernel = np.ones((7, 7), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            raw_detections = []
            for c in contours:
                area = cv2.contourArea(c)
                if area >= self.min_red_area:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        raw_detections.append({"pixel": (cx, cy), "area": area})
            
            # Sadece EN BÜYÜK alana sahip TEK bir tespit döndür
            if not raw_detections:
                return []
            # En büyük alanı seç
            largest = max(raw_detections, key=lambda d: d["area"])
            return [largest]
        except Exception:
            return []

    def _merge_nearby_detections(self, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Yakın contour'ları birleştir. Her gruptan en büyük alanı seç.
        Sonuç olarak tek parça tespit edilmiş olur.
        """
        if not detections:
            return []
        
        merge_dist = self.merge_pixel_distance
        
        # Simple clustering: assign each detection to a group
        groups: List[List[Dict[str, Any]]] = []
        used = [False] * len(detections)
        
        for i, det in enumerate(detections):
            if used[i]:
                continue
            
            # Start a new group with this detection
            group = [det]
            used[i] = True
            px1, py1 = det["pixel"]
            
            # Find all nearby detections
            for j, other in enumerate(detections):
                if used[j]:
                    continue
                px2, py2 = other["pixel"]
                dist = math.sqrt((px1 - px2) ** 2 + (py1 - py2) ** 2)
                if dist < merge_dist:
                    group.append(other)
                    used[j] = True
            
            groups.append(group)
        
        # Her gruptan en büyük alanı seç
        result = []
        for group in groups:
            # Alana göre sırala, en büyüğü al
            best = max(group, key=lambda d: d["area"])
            result.append(best)
        
        return result

    def _pc_point(self, px: int, py: int) -> Optional[Tuple[float, float, float]]:
        pc = self.latest_pc
        if pc is None:
            return None
        if px < 0 or px >= pc.width or py < 0 or py >= pc.height:
            return None

        idx = py * pc.width + px

        x_off = y_off = z_off = None
        for f in pc.fields:
            if f.name == "x":
                x_off = f.offset
            elif f.name == "y":
                y_off = f.offset
            elif f.name == "z":
                z_off = f.offset
        if x_off is None or y_off is None or z_off is None:
            return None

        start = idx * pc.point_step
        try:
            x = struct.unpack_from("f", pc.data, start + x_off)[0]
            y = struct.unpack_from("f", pc.data, start + y_off)[0]
            z = struct.unpack_from("f", pc.data, start + z_off)[0]
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                return None
            return (x, y, z)
        except Exception:
            return None

    def _to_world(self, p_cam: Tuple[float, float, float]) -> Optional[Tuple[float, float, float]]:
        try:
            pc = self.latest_pc
            assert pc is not None
            source_frame = pc.header.frame_id
            tr = self.tf_buffer.lookup_transform(
                self.world_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            ps = PointStamped()
            ps.header.frame_id = source_frame
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.point.x = float(p_cam[0])
            ps.point.y = float(p_cam[1])
            ps.point.z = float(p_cam[2])

            pw = do_transform_point(ps, tr)
            return (pw.point.x, pw.point.y, pw.point.z)
        except Exception:
            return None

    def _purge_cache(self, now_wall: float):
    # """TTL dolan kayıtları temizle (SR dışına çıkınca da cache tutulduğu için şart)."""
        if self.detection_ttl_sec <= 0:
            return
        with self.lock:
            self.detected_cache = [c for c in self.detected_cache if (now_wall - float(c.get("t", now_wall))) <= self.detection_ttl_sec]

    def _is_duplicate(
        self,
        p_world: Tuple[float, float, float],
        pixel: Tuple[int, int],
        now_wall: float,
    ) -> bool:
        """
        Duplicate filtresi:
        - Dünya koordinatı yakınsa (duplicate_distance)
        - VE/VEYA piksel yakınsa (duplicate_pixel_distance)
        - Üstelik yakın zaman penceresinde (duplicate_time_window_sec)
        """
        px, py = int(pixel[0]), int(pixel[1])

        with self.lock:
            for c in self.detected_cache:
                t = float(c.get("t", 0.0))
                if self.duplicate_time_window_sec > 0 and (now_wall - t) > self.duplicate_time_window_sec:
                    continue

                dx = float(p_world[0]) - float(c.get("x", 0.0))
                dy = float(p_world[1]) - float(c.get("y", 0.0))
                dz = float(p_world[2]) - float(c.get("z", 0.0))
                d = math.sqrt(dx * dx + dy * dy + dz * dz)

                dpix = math.sqrt((px - int(c.get("px", 0))) ** 2 + (py - int(c.get("py", 0))) ** 2)

                if d < self.duplicate_distance:
                    self.get_logger().debug(f"Duplicate found: distance={d:.3f}m < {self.duplicate_distance}m")
                    return True
                if self.duplicate_pixel_distance > 0 and dpix < self.duplicate_pixel_distance:
                    return True
        return False


def main():
    rclpy.init()
    node = HarmonyRedDetectionV4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()