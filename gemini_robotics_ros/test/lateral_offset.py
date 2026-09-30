#!/usr/bin/env python3
"""Kamera yüzeyleri YANLAMASINA nerede görüyor? X/Y sapmasını ölçer.

SALT OKUMA: yalnızca /points ve TF dinler. Hiçbir şey yayınlamaz, robot
hareket etmez.

    python3 test/lateral_offset.py                     # konveyör raylarını bul
    python3 test/lateral_offset.py --dx 0 --dy -0.03   # gözlenen hatayı çevir

NEDEN: surface_offset.py dikey sapmayı ölçer (kol nesnenin üstünde kalır).
Bu script yatay sapmayı ölçer (kap nesnenin yanına düşer). İkisi de aynı
kaynaktan gelir: ur_sick_optical_joint'in origin'i.

  ur_macro.xacro / ur10e_ur_sick_optical_joint
      <origin xyz="0 0 0.035" rpy="0 0 1.04720"/>
  xyz PARENT (depth_optical_frame) eksenlerinde ifade edilir. Kamera dik
  aşağı bakarken parent'ın +Z'si dünya -Z'sidir, +X ve +Y ise dünya yatay
  düzlemindedir ama takım oryantasyonu kadar dönüktür. Bu yüzden "dünyada
  3 cm -Y" düzeltmesi doğrudan joint'in y'sine YAZILAMAZ; önce parent'ın
  X/Y eksenlerine projekte edilmeli. Script bunu yapar.

  DİKKAT: xyz'nin x ve y'si şu an 0. Yani bugüne kadar kameranın yanlamasına
  konumu HİÇ kalibre edilmedi - sadece z (0.035) ve görüş ekseni etrafındaki
  yaw (60 deg) ayarlandı. Z 45 mm sapmış çıkan bir braketin X/Y'de tam isabet
  olması beklenmez.

İKİ MOD:

1) RAY MODU (argümansız). Konveyörün yan korkuluklarını bulutta bulur ve
   URDF'teki collision kutularıyla karşılaştırır. Korkuluklar X boyunca
   ayrık olduğu için bu yalnızca X sapmasını ölçer; konveyör Y boyunca uzun
   olduğundan Y'yi kısıtlamaz.

2) DÜZELTME MODU (--dx/--dy). Gözlenen dünya hatasını (algılanan eksi
   gerçek) joint origin'ine çevirir. Hatayı ölçmek için:
     a. Kolu tarama pozunda bırak, bir tarama çalıştır, algılanan (x, y)'yi
        logdan al  ->  ARM pose -> TOUCH uç=(X, Y, Z)
     b. Freedrive ile kap ağzını nesnenin O NOKTASINA elle getir
     c. python3 test/cup_tip_now.py  ile gerçek (x, y)'yi oku
     d. --dx (algi_x - gercek_x) --dy (algi_y - gercek_y)
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
import tf2_ros

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# my_robot_cell_macro.xacro'daki korkuluk collision kutuları (dünya frame'i).
# (merkez_x, X boyu). Korkuluklar Y boyunca uzanır.
URDF_RAILS = ((0.5045, 0.0284), (0.7945, 0.0284))
URDF_BELT_Z = 0.8450
URDF_RAIL_TOP_Z = 0.9000


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


class Probe(Node):
    def __init__(self, topic):
        super().__init__("lateral_offset_probe")
        self.cloud = None
        self.create_subscription(PointCloud2, topic,
                                 lambda m: setattr(self, "cloud", m), SENSOR_QOS)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/points")
    ap.add_argument("--world", default="world")
    ap.add_argument("--calib-parent", default="ur10e_depth_optical_frame")
    ap.add_argument("--calib-x", type=float, default=0.0)
    ap.add_argument("--calib-y", type=float, default=0.0)
    ap.add_argument("--dx", type=float, default=None,
                    help="dunyada gozlenen X hatasi: algi_x eksi gercek_x (m)")
    ap.add_argument("--dy", type=float, default=None,
                    help="dunyada gozlenen Y hatasi: algi_y eksi gercek_y (m)")
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.topic)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    spinner = threading.Thread(target=ex.spin, daemon=True)
    spinner.start()

    for _ in range(200):
        if node.cloud is not None:
            break
        time.sleep(0.05)
    if node.cloud is None:
        print(f"HATA: {args.topic} gelmedi.")
        return 1

    cloud = node.cloud
    frame = cloud.header.frame_id
    try:
        tf = node.buffer.lookup_transform(
            args.world, frame, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=3.0))
    except Exception as exc:
        print(f"HATA: TF {frame} -> {args.world} yok: {exc}")
        return 1

    t = tf.transform.translation
    r = tf.transform.rotation
    R = quat_to_matrix((r.x, r.y, r.z, r.w))
    T = np.array([t.x, t.y, t.z])

    axes = None
    try:
        tfc = node.buffer.lookup_transform(
            args.world, args.calib_parent, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=3.0))
        rc = tfc.transform.rotation
        axes = quat_to_matrix((rc.x, rc.y, rc.z, rc.w))
    except Exception as exc:
        print(f"UYARI: TF {args.calib_parent} -> {args.world} alinamadi: {exc}")

    raw = np.frombuffer(cloud.data, np.uint8).reshape(
        cloud.height, cloud.width, cloud.point_step)
    off = {f.name: f.offset for f in cloud.fields}
    get = lambda k: raw[:, :, off[k]:off[k]+4].copy().view(np.float32)[:, :, 0]
    pts = np.dstack((get("x"), get("y"), get("z"))).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    world = pts @ R.T + T

    print(f"kamera frame : {frame}")
    print(f"world konumu : {np.round(T, 4)}")
    print(f"gecerli nokta: {len(world)}")
    print(f"dunya x arali: {world[:,0].min():.4f} .. {world[:,0].max():.4f} m")
    print(f"dunya y arali: {world[:,1].min():.4f} .. {world[:,1].max():.4f} m")
    print(f"dunya z arali: {world[:,2].min():.4f} .. {world[:,2].max():.4f} m")

    if axes is not None:
        print()
        print(f"{args.calib_parent} eksenleri dunyada:")
        for i, nm in enumerate("XYZ"):
            print(f"  +{nm} = {np.round(axes[:, i], 4)}")

    # --- RAY MODU: korkulukları bulutta ara ---
    # Korkuluklar banttan yukarı çıkan, Y boyunca uzanan ince şeritler.
    band = world[(world[:, 2] > URDF_BELT_Z + 0.030) &
                 (world[:, 2] < URDF_RAIL_TOP_Z + 0.030)]
    print()
    print(f"KORKULUK BANDI (z {URDF_BELT_Z+0.030:.3f}..{URDF_RAIL_TOP_Z+0.030:.3f}): "
          f"{len(band)} nokta")
    if len(band) > 200:
        hist, edges = np.histogram(band[:, 0], bins=200,
                                   range=(world[:, 0].min(), world[:, 0].max()))
        # X histogramında en kalabalık iki ayrık tepe = iki korkuluk
        order = np.argsort(hist)[::-1]
        peaks = []
        for idx in order:
            c = 0.5 * (edges[idx] + edges[idx + 1])
            if hist[idx] < 20:
                break
            if all(abs(c - p) > 0.05 for p in peaks):
                peaks.append(c)
            if len(peaks) == 2:
                break
        peaks.sort()
        print(f"  bulutta korkuluk X'leri : {[round(p, 4) for p in peaks]}")
        print(f"  URDF korkuluk X'leri    : {[r[0] for r in URDF_RAILS]}")
        if len(peaks) == 2:
            du = [p - r[0] for p, r in zip(peaks, URDF_RAILS)]
            print(f"  X SAPMASI               : "
                  f"{du[0]*1000:+.1f} mm / {du[1]*1000:+.1f} mm  "
                  f"(ort {np.mean(du)*1000:+.1f} mm)")
            print(f"  bulutta korkuluk arasi  : {peaks[1]-peaks[0]:.4f} m")
            print(f"  URDF korkuluk arasi     : "
                  f"{URDF_RAILS[1][0]-URDF_RAILS[0][0]:.4f} m")
            print("  (ikisi ayni yone kayiyorsa OTELEME, ters yone ise OLCEK/aci)")
    else:
        print("  yeterli nokta yok - kol konveyoru goren tarama pozunda mi?")

    # --- DÜZELTME MODU ---
    if args.dx is not None or args.dy is not None:
        dx = args.dx or 0.0
        dy = args.dy or 0.0
        print()
        print(f"GOZLENEN DUNYA HATASI = ({dx*1000:+.1f}, {dy*1000:+.1f}) mm "
              "(algi eksi gercek)")
        if axes is None:
            print("  Parent eksenleri okunamadi; projeksiyon yapilamiyor.")
        else:
            # Joint origin'i (a, b) kadar oynatmak bulutu dunyada
            # a*Xhat + b*Yhat kadar kaydirir. Yatay bilesenlerin (-dx, -dy)
            # olmasini istiyoruz. 2x2 sistemi coz.
            M = np.array([[axes[0, 0], axes[0, 1]],
                          [axes[1, 0], axes[1, 1]]])
            det = np.linalg.det(M)
            print(f"  parent X/Y'nin yatay izdusumu det = {det:+.4f}")
            if abs(det) < 0.2:
                print("  Parent X/Y neredeyse dikey; bu pozdan yatay duzeltme cikmaz.")
            else:
                a, b = np.linalg.solve(M, np.array([-dx, -dy]))
                print(f"  mevcut  xyz = {args.calib_x} {args.calib_y} 0.035")
                print(f"  cozum   dx_joint = {a:+.5f}   dy_joint = {b:+.5f}")
                print(f"  YENI    xyz = {args.calib_x + a:+.5f} "
                      f"{args.calib_y + b:+.5f} 0.035")
                print()
                print("  Yazdiktan sonra NESNEYI BASKA BIR NOKTAYA KOYUP tekrar ol.")
                print("  Hata yeni noktada da ayni cikiyorsa oteleme dogruydu;")
                print("  buyuyup kuculuyorsa sorun yaw (1.04720) veya derinlik")
                print("  olceginde, oteleme ile kapatilamaz.")

    ex.shutdown()
    spinner.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
