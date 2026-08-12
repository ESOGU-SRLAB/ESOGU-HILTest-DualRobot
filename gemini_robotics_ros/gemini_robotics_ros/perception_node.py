#!/usr/bin/env python3
"""
Gemini algı düğümü: doğal dil sorgusu -> world frame'inde 3D tespitler + RViz marker.

Robotu hareket ettirmez. Hareketten önce "ER 2 sim görüntüsünde doğru yeri
gösteriyor mu" sorusunu ayrı ayrı doğrulamak için var.

Kullanım:
    ros2 topic pub --once /gemini/query std_msgs/String "{data: 'the red cube'}"
    ros2 topic echo /gemini/detections

RViz'de /gemini/markers (MarkerArray) eklenirse tespitler küre + etiket olarak görünür.
"""

from __future__ import annotations

import json
import threading
from datetime import datetime

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .er_client import make_er_client
from .locator import GeminiLocator


class GeminiPerceptionNode(Node):
    def __init__(self):
        super().__init__("gemini_perception")

        self.declare_parameter("backend", "gemini")
        self.declare_parameter("model", "gemini-robotics-er-2-preview")
        self.declare_parameter("thinking_level", "high")
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("vertex_project", "")
        self.declare_parameter("vertex_location", "us-central1")

        self.declare_parameter("image_topic", "/sim/image")
        self.declare_parameter("cloud_topic", "/sim/pointcloud")
        self.declare_parameter("depth_topic", "/sim/depth/image")
        self.declare_parameter("camera_info_topic", "/sim/camera_info")
        self.declare_parameter("intensity_topic", "/intensity")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("camera_frame_override", "")
        self.declare_parameter("deprojection", "auto")
        self.declare_parameter("sample_window", 5)
        self.declare_parameter("max_items", 10)
        self.declare_parameter("marker_lifetime_sec", 0.0)

        self.declare_parameter("er_image_source", "render")
        self.declare_parameter("render_mode", "normals")
        self.declare_parameter("render_min_m", 0.3)
        self.declare_parameter("render_max_m", 4.0)
        self.declare_parameter("publish_er_image", True)
        self.declare_parameter("er_image_rate_hz", 2.0)
        self.declare_parameter("gripper_type", "vacuum")

        self.declare_parameter("patch_radius_px", 12)
        self.declare_parameter("patch_depth_band", 0.05)
        self.declare_parameter("patch_min_points", 25)

        def get(name):
            return self.get_parameter(name).value

        er_source = str(get("er_image_source"))
        render_mode = str(get("render_mode"))
        gripper_type = str(get("gripper_type"))

        er_client = make_er_client(
            backend=str(get("backend")),
            model=str(get("model")),
            thinking_level=str(get("thinking_level")),
            jpeg_quality=int(get("jpeg_quality")),
            project=str(get("vertex_project")),
            location=str(get("vertex_location")),
            context_key="rgb" if er_source == "rgb" else render_mode,
            tool="vacuum" if gripper_type == "vacuum" else "parallel",
        )

        self.locator = GeminiLocator(
            node=self,
            er_client=er_client,
            image_topic=str(get("image_topic")),
            cloud_topic=str(get("cloud_topic")),
            depth_topic=str(get("depth_topic")),
            camera_info_topic=str(get("camera_info_topic")),
            intensity_topic=str(get("intensity_topic")),
            world_frame=str(get("world_frame")),
            camera_frame_override=str(get("camera_frame_override")),
            deprojection=str(get("deprojection")),
            sample_window=int(get("sample_window")),
            er_image_source=er_source,
            render_mode=render_mode,
            render_min_m=float(get("render_min_m")),
            render_max_m=float(get("render_max_m")),
            publish_er_image=bool(get("publish_er_image")),
            er_image_rate_hz=float(get("er_image_rate_hz")),
            patch_radius_px=int(get("patch_radius_px")),
            patch_depth_band=float(get("patch_depth_band")),
            patch_min_points=int(get("patch_min_points")),
        )

        self.world_frame = str(get("world_frame"))
        self.max_items = int(get("max_items"))
        self.marker_lifetime_sec = float(get("marker_lifetime_sec"))

        self.detections_pub = self.create_publisher(String, "/gemini/detections", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/gemini/markers", 10)
        self.create_subscription(String, "/gemini/query", self._on_query, 10)

        # Aynı anda tek sorgu: ER çağrısı saniyeler sürebiliyor ve topic'e arka
        # arkaya iki mesaj düşerse kota boşa gider.
        self._busy = threading.Lock()

        self.get_logger().info("gemini_perception hazır. Sorgu: /gemini/query (std_msgs/String)")

    def _on_query(self, msg: String) -> None:
        query = msg.data.strip()
        if not query:
            return
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn(f"Önceki sorgu sürüyor, atlandı: {query!r}")
            return
        threading.Thread(target=self._run_query, args=(query,), daemon=True).start()

    def _run_query(self, query: str) -> None:
        try:
            detections = self.locator.locate(query, max_items=self.max_items)

            payload = {
                "timestamp": datetime.now().isoformat(),
                "query": query,
                "frame_id": self.world_frame,
                "count": len(detections),
                "detections": detections,
            }
            out = String()
            out.data = json.dumps(payload, ensure_ascii=False)
            self.detections_pub.publish(out)
            self._publish_markers(detections)
        finally:
            self._busy.release()

    def _publish_markers(self, detections) -> None:
        array = MarkerArray()

        # Önceki marker'ları temizle; aksi halde eski tespitler RViz'de asılı kalır.
        clear = Marker()
        clear.header.frame_id = self.world_frame
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        stamp = self.get_clock().now().to_msg()
        lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime_sec).to_msg()

        for index, detection in enumerate(detections):
            position = detection["position"]
            surface = detection.get("surface")

            sphere = Marker()
            sphere.header.frame_id = self.world_frame
            sphere.header.stamp = stamp
            sphere.ns = "gemini_detections"
            sphere.id = index * 3
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = position["x"]
            sphere.pose.position.y = position["y"]
            sphere.pose.position.z = position["z"]
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.04
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.1, 0.8, 1.0, 0.9
            sphere.lifetime = lifetime
            array.markers.append(sphere)

            label = detection["label"]
            if surface is not None:
                label += f" ({surface['rms_residual'] * 1000:.1f} mm)"

            text = Marker()
            text.header.frame_id = self.world_frame
            text.header.stamp = stamp
            text.ns = "gemini_detections"
            text.id = index * 3 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = position["x"]
            text.pose.position.y = position["y"]
            text.pose.position.z = position["z"] + 0.08
            text.pose.orientation.w = 1.0
            text.scale.z = 0.05
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = label
            text.lifetime = lifetime
            array.markers.append(text)

            # Yüzey normali oku: vakumlu kavramada emme kabının hangi yönden
            # oturacağını gözle doğrulamanın tek pratik yolu.
            if surface is not None:
                centroid, normal = surface["centroid"], surface["normal"]
                arrow = Marker()
                arrow.header.frame_id = self.world_frame
                arrow.header.stamp = stamp
                arrow.ns = "gemini_normals"
                arrow.id = index * 3 + 2
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.orientation.w = 1.0
                start = Point(x=centroid["x"], y=centroid["y"], z=centroid["z"])
                end = Point(
                    x=centroid["x"] + normal["x"] * 0.12,
                    y=centroid["y"] + normal["y"] * 0.12,
                    z=centroid["z"] + normal["z"] * 0.12,
                )
                arrow.points = [start, end]
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.008, 0.016, 0.0
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.6, 0.1, 0.95
                arrow.lifetime = lifetime
                array.markers.append(arrow)

        self.markers_pub.publish(array)


def main():
    rclpy.init()
    node = GeminiPerceptionNode()
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
