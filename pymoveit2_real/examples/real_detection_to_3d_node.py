#!/usr/bin/env python3
"""
detection_to_3d_node (SIMPLIFIED REVERSE PROJECTION)
====================================================

Düzeltmeler (v2):
  1. IMX frame artık TF ağacındaki gerçek isimle eşleşiyor:
       "sim_ur10e_IMX_Camera_frame_link"  (lens = projeksiyon merkezi)
     camera_info frame_id ("sim_ur10e_imx179_camera") TF'de yok — hardcoded TF ismi kullanılıyor.

  2. _find_closest_point_on_ray: projection eşiği 0.05 → 0.0 yapıldı.
     Depth optical frame'inde Z ekseni kameradan uzağa değil, bazı pozisyonlarda
     farklı yönlere bakabilir; küçük pozitif eşik geçerli noktaları eliyordu.

  3. Ray casting: en yakın nokta yerine ray üzerindeki noktaların MEDIANI alınıyor.
     Böylece gürültülü tek noktalara kilitlenme önleniyor.

  4. TF lookup: avg_time yerine her mesajın kendi timestamp'i kullanılıyor.
     Arm hareket ederken ortalama timestamp TF cache'de olmayabilir.

  5. calib_offset_x / calib_offset_z: 0.0 bırakıldı — IMX frame düzeltmesiyle
     artık offset gerekmemeli. İnce ayar için parametre hâlâ mevcut.
"""

from copy import deepcopy
from threading import Lock

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

import message_filters

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sensor_msgs.msg import CameraInfo, PointCloud2
from yolo_msgs.msg import DetectionArray, BoundingBox3D


def pointcloud2_to_xyz_fast(cloud_msg: PointCloud2) -> np.ndarray:
    """sensor_msgs/PointCloud2 → (N, 3) numpy array"""
    field_map = {f.name: f for f in cloud_msg.fields}
    if "x" not in field_map or "y" not in field_map or "z" not in field_map:
        return np.empty((0, 3), dtype=np.float64)

    n_points = cloud_msg.width * cloud_msg.height
    if n_points == 0:
        return np.empty((0, 3), dtype=np.float64)

    point_step = cloud_msg.point_step
    raw = np.frombuffer(cloud_msg.data, dtype=np.uint8)

    ox = field_map["x"].offset
    oy = field_map["y"].offset
    oz = field_map["z"].offset

    raw_points = raw[: n_points * point_step].reshape(n_points, point_step)

    x = raw_points[:, ox : ox + 4].view(np.float32).reshape(-1)
    y = raw_points[:, oy : oy + 4].view(np.float32).reshape(-1)
    z = raw_points[:, oz : oz + 4].view(np.float32).reshape(-1)

    pts = np.column_stack((x, y, z)).astype(np.float64)

    valid = np.isfinite(pts).all(axis=1)
    return pts[valid]


def _tf_to_rt(tf_stamped):
    """TransformStamped → (R [3×3], t [3])"""
    tr = tf_stamped.transform.translation
    rot = tf_stamped.transform.rotation

    t = np.array([tr.x, tr.y, tr.z], dtype=np.float64)

    x, y, z, w = rot.x, rot.y, rot.z, rot.w
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)

    return R, t


class DetectionTo3DNode(Node):
    """Simplified Reverse Projection — v2 (Gerçek robot versiyonu)"""

    # ── Sabitler — gerçek robot TF frame'leri ─────────────────────────────────
    # SICK Visionary-T Mini depth optical frame (URDF'deki frame ismini gir)
    DEPTH_FRAME = "ur10e_depth_optical_frame"

    # Kamera lens frame'i (TF ağacında gerçek isim — URDF'e göre güncelle)
    IMX_FRAME   = "ur10e_rgb_optical_frame_link"

    TARGET_FRAME    = "world"
    MAX_RAY_DISTANCE = 0.08   # m — ray'e en fazla bu kadar uzak noktalar kabul edilir

    def __init__(self):
        super().__init__("real_detection_to_3d_node")

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter("depth_frame",      self.DEPTH_FRAME)
        self.declare_parameter("imx_frame",        self.IMX_FRAME)
        self.declare_parameter("target_frame",     self.TARGET_FRAME)
        self.declare_parameter("max_ray_distance", self.MAX_RAY_DISTANCE)
        self.declare_parameter("world_z_min",      0.75)
        self.declare_parameter("world_z_max",      1.5)
        self.declare_parameter("world_y_max",      0.3)
        self.declare_parameter("calib_offset_x",   0.0)   # ince kalibrasyon (gerekirse)
        self.declare_parameter("calib_offset_z",   0.0)
        self.declare_parameter("bbox3d_size_x",    0.08)
        self.declare_parameter("bbox3d_size_y",    0.08)
        self.declare_parameter("bbox3d_size_z",    0.10)

        self.depth_frame     = self.get_parameter("depth_frame").value
        self.imx_frame       = self.get_parameter("imx_frame").value
        self.target_frame    = self.get_parameter("target_frame").value
        self.max_ray_dist    = self.get_parameter("max_ray_distance").value
        self.world_z_min     = self.get_parameter("world_z_min").value
        self.world_z_max     = self.get_parameter("world_z_max").value
        self.world_y_max     = self.get_parameter("world_y_max").value
        self.calib_offset_x  = self.get_parameter("calib_offset_x").value
        self.calib_offset_z  = self.get_parameter("calib_offset_z").value
        self.bbox3d_size_x   = self.get_parameter("bbox3d_size_x").value
        self.bbox3d_size_y   = self.get_parameter("bbox3d_size_y").value
        self.bbox3d_size_z   = self.get_parameter("bbox3d_size_z").value

        # ── TF ────────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── K matrisi ─────────────────────────────────────────────────────────
        self._K      = None
        self._K_lock = Lock()

        # ── QoS ───────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )

        # ── Subscriber'lar ────────────────────────────────────────────────────
        # Gerçek robot kamera topic'leri (SICK Visionary-T Mini)
        # Farklı bir kamera kullanıyorsan aşağıdaki topic isimlerini değiştir:
        #   camera_info → /camera_info veya /sick/camera_info vb.
        #   pointcloud  → /points veya /sick/points vb.
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("pointcloud_topic",  "/points")
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        pointcloud_topic  = str(self.get_parameter("pointcloud_topic").value)

        self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_cb, sensor_qos
        )
        self._det_sub = message_filters.Subscriber(
            self, DetectionArray, "/yolo/detections", qos_profile=reliable_qos
        )
        self._pc_sub = message_filters.Subscriber(
            self, PointCloud2, pointcloud_topic, qos_profile=sensor_qos
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._det_sub, self._pc_sub], queue_size=10, slop=0.1
        )
        self._sync.registerCallback(self._synced_cb)

        self._pub = self.create_publisher(DetectionArray, "/detections_3d_world", 10)

        self.get_logger().info(
            f"detection_to_3d_node v2 başlatıldı\n"
            f"  imx_frame  = {self.imx_frame}\n"
            f"  depth_frame= {self.depth_frame}\n"
            f"  target     = {self.target_frame}\n"
            f"  max_ray    = {self.max_ray_dist} m"
        )

    # ──────────────────────────────────────────────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo):
        """K matrisini bir kere al, camera_info frame_id'sini KULLANMA (TF'de yok)."""
        with self._K_lock:
            if self._K is None:
                self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
                self.get_logger().info(
                    f"K matrisi alındı (camera_info frame={msg.header.frame_id})\n"
                    f"  fx={self._K[0,0]:.2f}  fy={self._K[1,1]:.2f}\n"
                    f"  cx={self._K[0,2]:.2f}  cy={self._K[1,2]:.2f}\n"
                    f"  NOT: TF lookup için '{self.imx_frame}' kullanılıyor "
                    f"(camera_info frame_id TF'de mevcut değil)"
                )

    # ──────────────────────────────────────────────────────────────────────────
    def _synced_cb(self, det_msg: DetectionArray, pc_msg: PointCloud2):
        with self._K_lock:
            K = self._K
        if K is None:
            self.get_logger().warn("K matrisi henüz alınmadı", throttle_duration_sec=5.0)
            return

        if not det_msg.detections:
            return

        # Pointcloud frame normalizasyonu (namespace kaldır)
        pc_frame = pc_msg.header.frame_id.split("/")[-1]

        # Pointcloud parse
        pts = pointcloud2_to_xyz_fast(pc_msg)
        n_pts = pts.shape[0]
        if n_pts == 0:
            self.get_logger().warn("PointCloud boş!", throttle_duration_sec=5.0)
            return

        pc_min  = pts.min(axis=0)
        pc_max  = pts.max(axis=0)
        pc_mean = pts.mean(axis=0)
        self.get_logger().info(
            f"📦 PC: {n_pts} nokta | frame={pc_frame}\n"
            f"  min=({pc_min[0]:.3f}, {pc_min[1]:.3f}, {pc_min[2]:.3f})\n"
            f"  max=({pc_max[0]:.3f}, {pc_max[1]:.3f}, {pc_max[2]:.3f})\n"
            f"  mean=({pc_mean[0]:.3f}, {pc_mean[1]:.3f}, {pc_mean[2]:.3f})",
            throttle_duration_sec=5.0,
        )

        # DÜZELTİLDİ: avg_time yerine her mesajın kendi zamanı kullanılıyor.
        # Arm hareket ederken ortalama timestamp TF cache'de bulunmayabilir.
        pc_time  = rclpy.time.Time.from_msg(pc_msg.header.stamp)
        det_time = rclpy.time.Time.from_msg(det_msg.header.stamp)
        dt_ms    = abs(pc_time.nanoseconds - det_time.nanoseconds) / 1e6

        # TF lookup için pointcloud zamanını kullan (depth verisiyle senkron)
        lookup_time = pc_time

        self.get_logger().info(
            f"⏱ Sync: {len(det_msg.detections)} det + PC ({n_pts} pts) | dt={dt_ms:.0f}ms",
            throttle_duration_sec=5.0,
        )

        # TF lookup
        try:
            # IMX kamera lens → depth optical frame
            tf_imx_to_depth = self.tf_buffer.lookup_transform(
                pc_frame,           # hedef frame (depth)
                self.imx_frame,     # kaynak frame (IMX lens) — TF'deki gerçek isim
                lookup_time,
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            # depth optical frame → world
            tf_depth_to_world = self.tf_buffer.lookup_transform(
                self.target_frame,
                pc_frame,
                lookup_time,
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup hatası: {e}", throttle_duration_sec=3.0)
            return

        R_imx_to_depth, t_imx_to_depth = _tf_to_rt(tf_imx_to_depth)
        R_depth_to_world, t_depth_to_world = _tf_to_rt(tf_depth_to_world)

        tr_w = tf_depth_to_world.transform.translation
        tr_i = tf_imx_to_depth.transform.translation
        self.get_logger().info(
            f"🌍 TF depth→world: t=({tr_w.x:.3f}, {tr_w.y:.3f}, {tr_w.z:.3f})\n"
            f"   TF imx→depth:   t=({tr_i.x:.3f}, {tr_i.y:.3f}, {tr_i.z:.3f})",
            throttle_duration_sec=5.0,
        )

        # Tespit işleme
        out_msg = DetectionArray()
        out_msg.header = deepcopy(det_msg.header)
        out_msg.header.frame_id = self.target_frame

        total_count    = len(det_msg.detections)
        no_ray_count   = 0
        filtered_count = 0

        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        for det in det_msg.detections:
            u = det.bbox.center.position.x   # piksel koordinatları
            v = det.bbox.center.position.y

            # IMX frame'inde normalize edilmiş ray yönü
            ray_imx = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
            ray_imx /= np.linalg.norm(ray_imx)

            # Ray'i depth frame'ine taşı
            ray_depth    = R_imx_to_depth @ ray_imx
            origin_depth = t_imx_to_depth   # IMX'in depth frame'indeki konumu

            # Pointcloud'dan en yakın noktayı bul
            best_pt, ray_info = self._find_closest_point_on_ray(pts, ray_depth, origin_depth)

            if best_pt is None:
                no_ray_count += 1
                self.get_logger().info(
                    f"❌ [{det.class_name}] bbox=({u:.0f},{v:.0f}) "
                    f"ray FAIL — {ray_info}"
                )
                continue

            self.get_logger().info(
                f"🎯 [{det.class_name}] bbox=({u:.0f},{v:.0f}) "
                f"depth_pt=({best_pt[0]:.3f},{best_pt[1]:.3f},{best_pt[2]:.3f}) "
                f"ray_dist={ray_info['best_dist']:.4f}m "
                f"({ray_info['n_close']}/{ray_info['n_forward']} pts)"
            )

            # Depth → world
            pos_world = R_depth_to_world @ best_pt + t_depth_to_world
            pos_raw   = pos_world.copy()

            # Kalibrasyon offseti (ince ayar gerekirse)
            pos_world[0] += self.calib_offset_x
            pos_world[2] += self.calib_offset_z

            # Sınır filtresi
            reasons = []
            if pos_world[1] > self.world_y_max:
                reasons.append(f"y={pos_world[1]:.3f} > {self.world_y_max}")
            if pos_world[2] < self.world_z_min:
                reasons.append(f"z={pos_world[2]:.3f} < {self.world_z_min}")
            if pos_world[2] > self.world_z_max:
                reasons.append(f"z={pos_world[2]:.3f} > {self.world_z_max}")

            if reasons:
                filtered_count += 1
                self.get_logger().info(
                    f"🚫 [{det.class_name}] FİLTRE: "
                    f"world=({pos_world[0]:.3f},{pos_world[1]:.3f},{pos_world[2]:.3f}) "
                    f"raw=({pos_raw[0]:.3f},{pos_raw[1]:.3f},{pos_raw[2]:.3f}) "
                    f"sebep: {', '.join(reasons)}"
                )
                continue

            # BoundingBox3D oluştur
            bbox3d = BoundingBox3D()
            bbox3d.center.position.x = float(pos_world[0])
            bbox3d.center.position.y = float(pos_world[1])
            bbox3d.center.position.z = float(pos_world[2])
            bbox3d.frame_id          = self.target_frame
            bbox3d.size.x            = self.bbox3d_size_x
            bbox3d.size.y            = self.bbox3d_size_y
            bbox3d.size.z            = self.bbox3d_size_z

            det.bbox3d = bbox3d
            out_msg.detections.append(det)

            self.get_logger().info(
                f"✅ [{det.class_name}] "
                f"({pos_world[0]:.3f}, {pos_world[1]:.3f}, {pos_world[2]:.3f})"
            )

        published = len(out_msg.detections)
        self.get_logger().info(
            f"📊 Özet: {total_count} tespit → {published} yayınlandı | "
            f"{no_ray_count} ray yok | {filtered_count} filtreden düştü"
        )

        if out_msg.detections:
            self._pub.publish(out_msg)

    # ──────────────────────────────────────────────────────────────────────────
    def _find_closest_point_on_ray(self, pts: np.ndarray, ray_dir: np.ndarray,
                                    origin: np.ndarray):
        """
        Pointcloud'daki noktalar arasından ray'e en yakın olanı döndür.

        DÜZELTİLMİŞ:
          - projection eşiği 0.05 → 0.0 (küçük eşik geçerli noktaları eliyordu)
          - Eşik içindeki noktaların medianı alınır (tek gürültülü noktaya kilitlenme önlenir)

        Returns: (nokta [3,], info_dict) veya (None, hata_str)
        """
        vectors     = pts - origin                        # (N, 3)
        projections = np.dot(vectors, ray_dir)            # (N,)

        # Ray yönünde olan noktalar (kameranın önünde)
        forward     = projections > 0.0
        n_forward   = int(forward.sum())
        if n_forward == 0:
            return None, f"{len(pts)} pts, 0 forward"

        vecs_fwd  = vectors[forward]
        proj_fwd  = projections[forward]

        # Ray'e dik uzaklık
        ray_pts   = proj_fwd[:, None] * ray_dir           # ray üzerindeki en yakın nokta
        distances = np.linalg.norm(vecs_fwd - ray_pts, axis=1)

        close   = distances < self.max_ray_dist
        n_close = int(close.sum())
        if n_close == 0:
            min_d = float(distances.min())
            return None, (
                f"{n_forward} forward pts, "
                f"min_dist={min_d:.4f}m > threshold={self.max_ray_dist}m"
            )

        # Eşik içindeki noktaların medianını al (daha kararlı)
        close_pts  = pts[forward][close]                  # (n_close, 3)
        median_pt  = np.median(close_pts, axis=0)
        best_dist  = float(distances[close].min())

        return median_pt, {
            "best_dist": best_dist,
            "n_close":   n_close,
            "n_forward": n_forward,
        }


def main():
    rclpy.init()
    node = DetectionTo3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()