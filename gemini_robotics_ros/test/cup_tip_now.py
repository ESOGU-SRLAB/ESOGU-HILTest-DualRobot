#!/usr/bin/env python3
"""Kabın ağzı ŞU ANDA dünyada nerede? TF'in iddiası vs şerit metre.

SALT OKUMA: yalnızca TF dinler. Hiçbir şey yayınlamaz, robot hareket etmez.

    python3 test/cup_tip_now.py

NEDEN VAR: "kol nesnenin 2-3 cm üstünde kalıyor" belirtisinin iki ayrı
sebebi olabilir ve ikisi birbirine benzer görünür:

  (a) ALGI kayık  - hedef noktası yanlış yerde, kol doğru yere gidiyor
  (b) KOL zinciri kayık - hedef doğru, kol oraya gittiğini sanıyor ama değil

Kamera kolun ÜSTÜNDE olduğu için tek bir yüzey ölçümü bu ikisini ayıramaz;
her iki hata da aynı sapmayı üretir. Bu araç kamerayı denklemden tamamen
çıkarır: TF'e göre kap ağzının dünya koordinatını basar. Kolu durdurup
kap ağzının yerden yüksekliğini şerit metreyle ölç ve karşılaştır.

  TF ile ölçüm UYUŞUYORSA  -> kol zinciri sağlam, hata ALGIDA
  TF ile ölçüm UYUŞMUYORSA -> hata KOLDA (FK / TCP / montaj)

Fark doğrudan milimetre cinsinden aradığımız sapmadır.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
import tf2_ros

# config/gemini_params.yaml ile aynı olmalı. Kap ağzının uç eleman frame
# ORİJİNİNE göre yeri; tool_geometry.py mesh'ten 84.24 mm ölçüyor.
DEFAULT_TIP_OFFSET = (0.072920, 0.042100, 0.0)


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


class Probe(Node):
    def __init__(self):
        super().__init__("cup_tip_probe")
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="world")
    ap.add_argument("--link", default="ur10e_suction_cup")
    ap.add_argument("--tip-offset", nargs=3, type=float,
                    default=list(DEFAULT_TIP_OFFSET),
                    help="kap ağzının link frame'indeki yeri (m)")
    ap.add_argument("--truth", type=float, default=None,
                    help="kap ağzının şerit metreyle ölçülen YERDEN yüksekliği (m)")
    args = ap.parse_args()

    rclpy.init()
    node = Probe()
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    spinner = threading.Thread(target=ex.spin, daemon=True)
    spinner.start()

    tf = None
    for _ in range(40):
        try:
            tf = node.buffer.lookup_transform(
                args.world, args.link, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            break
        except Exception:
            time.sleep(0.25)
    if tf is None:
        print(f"HATA: TF {args.link} -> {args.world} alınamadı.")
        ex.shutdown(); spinner.join(timeout=2.0)
        node.destroy_node(); rclpy.shutdown()
        return 1

    t = tf.transform.translation
    r = tf.transform.rotation
    R = quat_to_matrix((r.x, r.y, r.z, r.w))
    origin = np.array([t.x, t.y, t.z])
    offset = np.asarray(args.tip_offset, dtype=np.float64)
    tip = origin + R @ offset

    print(f"link            : {args.link}")
    print(f"  frame orijini : {np.round(origin, 4)}")
    print(f"  tip offset    : {np.round(offset, 6)}  (|.| = "
          f"{np.linalg.norm(offset)*1000:.2f} mm)")
    print()
    print(f"KAP AĞZI (TF'e göre, dünya) = {np.round(tip, 4)}")
    print(f"  yerden yükseklik z        = {tip[2]:.4f} m")

    # Kabın dünyada hangi yöne baktığı: emme ekseni offset yönüdür.
    axis = R @ offset
    n = float(np.linalg.norm(axis))
    if n > 1e-9:
        axis = axis / n
        print(f"  emme ekseni (dünya)       = {np.round(axis, 4)}  "
              f"-> dikeyden {np.degrees(np.arccos(abs(axis[2]))):.1f} derece")

    if args.truth is not None:
        d = tip[2] - args.truth
        print()
        print(f"ŞERİT METRE = {args.truth:.4f} m")
        print(f"FARK        = {d*1000:+.1f} mm")
        print()
        if abs(d) < 0.005:
            print("  Kol zinciri SAĞLAM (< 5 mm). Sapma ALGIDA aranmalı:")
            print("    test/surface_offset.py ile yüzey yüksekliğini ölç.")
        else:
            print("  Kol zinciri KAYIK. TF kabı olduğundan "
                  f"{'YUKARIDA' if d > 0 else 'AŞAĞIDA'} sanıyor.")
            print("  Bakılacak yerler (algı DEĞİL):")
            print("    - takılı kap fiziksel olarak suction_cup.stl ile aynı mı")
            print("    - tool_tip_offset (tool_geometry.py ile ölç)")
            print("    - flange-tool0 / tool0-sickcamera montaj ölçüleri")

    ex.shutdown()
    spinner.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
