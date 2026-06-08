#!/usr/bin/env python3
"""
Red detection / mock perception - v2
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
from std_msgs.msg import String, Int32

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


class HarmonyRedDetectionV2(Node):
    def __init__(self):
        super().__init__("harmony_red_detection")

        self.declare_parameter("image_topic", "/sim/image")
        self.declare_parameter("pointcloud_topic", "/sim/pointcloud")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("min_red_area", 500)
        self.declare_parameter("duplicate_distance", 0.15)
        self.declare_parameter("publish_rate_limit_hz", 2.0)  # spam engelle

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.min_red_area = int(self.get_parameter("min_red_area").value)
        self.duplicate_distance = float(self.get_parameter("duplicate_distance").value)
        self.publish_rate_limit_hz = float(self.get_parameter("publish_rate_limit_hz").value)

        cbg = ReentrantCallbackGroup()

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.defect_pub = self.create_publisher(String, "/harmony/mock_perception/defect", 10)
        self.log_pub = self.create_publisher(String, "/harmony/log", 50)

        # Subscribers
        if CV_AVAILABLE:
            self.image_sub = self.create_subscription(Image, self.image_topic, self._image_cb, 10, callback_group=cbg)
        else:
            self._log("WARN", "CV_MISSING", "OpenCV not available; red detection disabled", {})

        self.pc_sub = self.create_subscription(PointCloud2, self.pointcloud_topic, self._pc_cb, 10, callback_group=cbg)
        self.robot_status_sub = self.create_subscription(String, "/harmony/robot_status", self._robot_status_cb, 10, callback_group=cbg)
        self.waypoint_sub = self.create_subscription(Int32, "/sensing_robot/current_waypoint", self._wp_cb, 10, callback_group=cbg)

        # State
        self.latest_image = None
        self.latest_pc: Optional[PointCloud2] = None
        self.in_sr_mode = False
        self.current_waypoint = 0

        self.lock = Lock()
        self.detected_world_points: List[Tuple[float, float, float]] = []
        self.last_pub_time = 0.0

        self.timer = self.create_timer(0.2, self._tick)

        self._log("INFO", "NODE_START", "HarmonyRedDetectionV2 initialized", {"world_frame": self.world_frame})

    def _log(self, level: str, event: str, detail: str, ctx: Dict[str, Any]):
        payload = {
            "timestamp": _now_ros(self),
            "node": self.get_name(),
            "level": level,
            "event": event,
            "detail": detail,
            "ctx": ctx,
        }

        if level == "ERROR":
            self.get_logger().error(f"{event}: {detail} | {ctx}")
        elif level == "WARN":
            self.get_logger().warning(f"{event}: {detail} | {ctx}")
        else:
            self.get_logger().info(f"{event}: {detail} | {ctx}")

        msg = String()
        msg.data = json.dumps(payload)
        self.log_pub.publish(msg)

    def _robot_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.in_sr_mode = (str(data.get("state")) == "SR_MODE")
            # SR_MODE dışında duplicate cache temizlenebilir
            if not self.in_sr_mode:
                with self.lock:
                    self.detected_world_points.clear()
        except Exception:
            self.in_sr_mode = False

    def _wp_cb(self, msg: Int32):
        self.current_waypoint = int(msg.data)

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

        if self._is_duplicate(p_world):
            return

        with self.lock:
            self.detected_world_points.append(p_world)

        defect_payload = {
            "timestamp": datetime.now().isoformat(),
            "defect_id": f"defect_{int(time.time())}",
            "defect_type": "PATCH",
            "severity_color": "RED",
            "frame_id": self.world_frame,
            "waypoint_index": self.current_waypoint,
            "position": {"x": float(p_world[0]), "y": float(p_world[1]), "z": float(p_world[2])},
            "pixel": {"x": int(px), "y": int(py)},
            "area": float(best["area"]),
        }

        out = String()
        out.data = json.dumps(defect_payload)
        self.defect_pub.publish(out)
        self.last_pub_time = now

        self._log("INFO", "DEFECT_PUB", "Published defect JSON", {
            "defect_id": defect_payload["defect_id"],
            "waypoint": self.current_waypoint,
            "pos": defect_payload["position"],
            "area": defect_payload["area"],
        })

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
            out = []
            for c in contours:
                area = cv2.contourArea(c)
                if area >= self.min_red_area:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        out.append({"pixel": (cx, cy), "area": area})
            return out
        except Exception:
            return []

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

    def _is_duplicate(self, p_world: Tuple[float, float, float]) -> bool:
        with self.lock:
            for e in self.detected_world_points:
                d = math.sqrt((p_world[0] - e[0]) ** 2 + (p_world[1] - e[1]) ** 2 + (p_world[2] - e[2]) ** 2)
                if d < self.duplicate_distance:
                    return True
        return False


def main():
    rclpy.init()
    node = HarmonyRedDetectionV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
