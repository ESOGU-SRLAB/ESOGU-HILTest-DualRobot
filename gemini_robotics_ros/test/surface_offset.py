#!/usr/bin/env python3
"""
Kamera yüzeyleri NEREDE görüyor? Sistematik yükseklik sapmasını ölçer.

SALT OKUMA: yalnızca /points ve TF dinler. Hiçbir şey yayınlamaz, robot
hareket etmez.

    python3 test/surface_offset.py                  # baskın düzlemi ölç
    python3 test/surface_offset.py --truth 0.878    # gerçek yüksekliği biliyorsan

NEDEN: "kol nesnenin bir tık üstüne gidiyor" belirtisi, algılanan yüzeyin
gerçeğinden YÜKSEK okunmasıyla birebir aynı şeydir. Kol her zaman algılanan
noktaya gider; nokta 10 mm yukarıdaysa kap 10 mm yukarıda durur.

Zincirdeki sabitlerden ikisi elle ayarlanmıştır ve doğrudan bu sapmaya yazar:

  ur_macro.xacro / ur10e_ur_sick_optical_joint
      <origin xyz="0 0 0.035" rpy="0 0 1.04720"/>
  Bu z, optik eksen (PARENT = depth_optical_frame'in +Z'si) boyunca bir
  ötelemedir; dünya Z'si boyunca DEĞİL. Kamera aşağı bakarken z'yi artırmak
  algılanan noktaları dünyada AŞAĞI indirir. Script bu projeksiyonu kendisi
  yapar: d = -sapma / cos(eksen, dünya Z).

  DİKKAT - --truth'a MESH'TEN türetilmiş bir sayı VERME. Bu iki kez yanılttı:
  conveyorbelt.stl gerçek konveyörü temsil etmiyor (mesh'te yan raylar bandın
  3 mm üstünde, gerçekte 55 mm). Mesh'ten türetilen her "gerçek" değer bu
  hatayı içeri taşır. --truth ŞERİT METRE ölçümü olmalı.

  Konveyör bandı için doğrulanmış değer: 0.845 m (14 Ağu 2026, yerden).
  Dünya z=0'ın yer olduğu da doğrulandı: kol TOUCH pozundayken TF kap ağzını
  0.9995'te gösterdi, metre 1.00-1.01 m okudu.

Kullanım: kolu konveyörü gören taramada bırak, bunu çalıştır, çıkan yüksekliği
konveyörün şerit metreyle ölçülen yüksekliğiyle karşılaştır. Fark = zincirin
toplam sistematik sapması.
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


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


class Probe(Node):
    def __init__(self, topic):
        super().__init__("surface_offset_probe")
        self.cloud = None
        self.create_subscription(PointCloud2, topic,
                                 lambda m: setattr(self, "cloud", m), SENSOR_QOS)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/points")
    ap.add_argument("--world", default="world")
    ap.add_argument("--truth", type=float, default=None,
                    help="yüzeyin ŞERİT METREYLE ölçülen dünya z'si (m)")
    ap.add_argument("--band", type=float, default=0.004,
                    help="düzlem bandı yarı kalınlığı (m)")
    ap.add_argument("--calib-parent", default="ur10e_depth_optical_frame",
                    help="ur_sick_optical_joint'in PARENT frame'i; -0.01 bu "
                         "frame'in Z ekseni boyuncadır")
    ap.add_argument("--calib-z", type=float, default=0.035,
                    help="ur_sick_optical_joint'in mevcut origin z'si (m)")
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

    # ur_sick_optical_joint'in origin'i PARENT frame'de ifade edilir. Joint'in
    # z'sini d kadar oynatmak buluttaki her noktayı dünyada d * z_hat_parent
    # kadar kaydırır -- dünya Z'si boyunca değil. Ekseni ölç.
    axis_world = None
    try:
        tfc = node.buffer.lookup_transform(
            args.world, args.calib_parent, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=3.0))
        rc = tfc.transform.rotation
        axis_world = quat_to_matrix((rc.x, rc.y, rc.z, rc.w))[:, 2]
    except Exception as exc:
        print(f"UYARI: TF {args.calib_parent} -> {args.world} alınamadı: {exc}")
        print("       Düzeltme optik eksene projekte EDİLEMEYECEK.")

    raw = np.frombuffer(cloud.data, np.uint8).reshape(
        cloud.height, cloud.width, cloud.point_step)
    off = {f.name: f.offset for f in cloud.fields}
    get = lambda k: raw[:, :, off[k]:off[k]+4].copy().view(np.float32)[:, :, 0]
    pts = np.dstack((get("x"), get("y"), get("z"))).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    world = pts @ R.T + T

    print(f"kamera frame : {frame}")
    print(f"world konumu : {np.round(T, 4)}")
    print(f"geçerli nokta: {len(world)}")
    print(f"dünya z aralığı: {world[:,2].min():.4f} .. {world[:,2].max():.4f} m")

    # Baskın YATAY yüzey: z histogramının modu
    z = world[:, 2]
    hist, edges = np.histogram(z, bins=400)
    peak = int(hist.argmax())
    coarse = 0.5 * (edges[peak] + edges[peak + 1])
    sel = np.abs(z - coarse) < args.band
    level = float(np.median(z[sel]))

    print()
    print(f"BASKIN YATAY DÜZLEM (dünya z) = {level:.4f} m")
    print(f"  bandındaki nokta sayısı     = {int(sel.sum())}")
    print(f"  band içi std                = {z[sel].std()*1000:.2f} mm")

    print("\n  en kalabalık 5 seviye:")
    for idx in np.argsort(hist)[::-1][:5]:
        centre = 0.5 * (edges[idx] + edges[idx + 1])
        print(f"    z = {centre:.4f} m   {hist[idx]:6d} nokta")

    if axis_world is not None:
        cos = float(axis_world[2])
        print()
        print(f"optik eksen ({args.calib_parent} +Z) dünyada = "
              f"{np.round(axis_world, 4)}")
        print(f"  dünya Z ile hizası (cos) = {cos:+.4f}  "
              f"-> dikeyden {np.degrees(np.arccos(abs(cos))):.1f}° sapma")

    if args.truth is not None:
        delta = level - args.truth
        print()
        print(f"GERÇEK  = {args.truth:.4f} m")
        print(f"SAPMA   = {delta*1000:+.1f} mm  "
              f"({'algı YÜKSEK okuyor -> kol üstte kalır' if delta > 0 else 'algı ALÇAK okuyor -> kol bastırır'})")
        print()
        print("  Düzeltme ur_macro.xacro'daki kalibre frame'e yazılmalı:")
        print("    <joint name=\"${tf_prefix}ur_sick_optical_joint\" ...>")
        print(f"      mevcut  <origin xyz=\"0 0 {args.calib_z}\" rpy=\"0 0 1.04720\"/>")

        if axis_world is None:
            print("    Optik eksen okunamadı; projeksiyon yapılamıyor.")
        elif abs(cos) < 0.2:
            print(f"    Optik eksen neredeyse yatay (cos={cos:+.3f}); bu joint'in z'si")
            print("    bu pozda dünya yüksekliğini neredeyse hiç değiştirmez.")
            print("    Sapmanın kaynağı başka yerde - bu pozdan düzeltme çıkarma.")
        else:
            # Bulut dünyada d * axis_world kadar kayar; z bileşeninin
            # -delta olmasını istiyoruz.
            d = -delta / cos
            print(f"    Joint z'sini d kadar oynatmak bulutu dünyada "
                  f"d*{cos:+.4f} kadar dikey kaydırır.")
            print(f"    d = -({delta:+.4f}) / {cos:+.4f} = {d:+.5f} m")
            print(f"    YENİ: xyz z = {args.calib_z} + {d:+.5f} = {args.calib_z + d:+.5f}")
            if abs(abs(cos) - 1.0) > 0.02:
                print(f"    (kamera dikeyden "
                      f"{np.degrees(np.arccos(abs(cos))):.1f}° sapık, bu yüzden "
                      f"{abs(d)*1000:.1f} mm != {abs(delta)*1000:.1f} mm)")
        print("    Düzeltmeden sonra bu ölçümü TEKRARLA; sapma ~0 olmalı.")

    # Spin thread'i hala donerken yorumlayici kapanirsa rclcpp tarafi
    # "terminate called without an active exception" ile cokuyor. Once
    # executor'u durdur, thread'i bekle, sonra kapat.
    ex.shutdown()
    spinner.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
