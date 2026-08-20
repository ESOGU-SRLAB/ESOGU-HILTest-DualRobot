#!/usr/bin/env python3
"""
replay_publisher.py
===================
Kayıtlı parquet verisini gerçek sürücü gibi 500 Hz'de yayınlar — robot olmadan
`detector` düğümünü uçtan uca sınamak için.

Sürücünün mesaj yapısını taklit eder:
    /joint_states                            sensor_msgs/JointState
        name    = <tf_prefix><eklem>_joint
        effort  = motor AKIMI [A]            (UR sürücüsü buraya actual_current yazar)
    /force_torque_sensor_broadcaster/ft_data geometry_msgs/WrenchStamped
        frame_id = <tf_prefix>tool0          (wrench TCP çerçevesinde)

`--fault` verilirse kaydın ikinci yarısında ölçüm uzayında arıza üretir.

Kullanım
--------
    ros2 run anomaly_detection replay_publisher
    ros2 run anomaly_detection replay_publisher --ros-args -p fault:=carpisma
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

from .features import JOINT_SUFFIX

BASE = "/home/cem/colcon_ws/src/anomaly_detection"


class ReplayPublisher(Node):

    def __init__(self):
        super().__init__("ur10e_replay_publisher")
        p = self.declare_parameter
        p("parquet", f"{BASE}/ur10e_clean.parquet")
        p("current_to_torque", f"{BASE}/current_to_torque.json")
        p("tf_prefix", "ur10e_")
        p("rate", 500.0)
        p("min_run", 3000)
        p("fault", "yok")          # yok | motor_kaymasi | carpisma | gizyazar | sensor_gurultu
        p("loop", True)

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.prefix = str(g("tf_prefix"))
        self.fault = str(g("fault"))
        self.loop = bool(g("loop"))
        self.nm_per_amp = np.array(
            json.loads(Path(str(g("current_to_torque"))).read_text())["nm_per_amp"])

        import pandas as pd
        df = pd.read_parquet(str(g("parquet")))
        run = df["run_id"].to_numpy()
        edges = np.concatenate([[0], np.flatnonzero(np.diff(run) != 0) + 1, [len(run)]])
        segs = [(int(a), int(c - a)) for a, c in zip(edges[:-1], edges[1:])]
        segs = [s for s in segs if s[1] >= int(g("min_run"))]
        if not segs:
            raise RuntimeError(f"{g('min_run')} örnekten uzun koşu yok.")
        a, L = max(segs, key=lambda s: s[1])
        self.Q = df[[f"q_{j}" for j in range(1, 7)]].to_numpy(np.float64)[a:a + L]
        self.QD = df[[f"qd_{j}" for j in range(1, 7)]].to_numpy(np.float64)[a:a + L]
        self.AMP = df[[f"tau_{j}" for j in range(1, 7)]].to_numpy(np.float64)[a:a + L]
        self.WR = df[["fx", "fy", "fz", "tx", "ty", "tz"]].to_numpy(np.float64)[a:a + L]
        del df

        self.n = L
        self.onset = int(L * 0.5)
        self.i = 0
        self.rng = np.random.default_rng(0)
        self.names = [self.prefix + s for s in JOINT_SUFFIX]

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub_js = self.create_publisher(JointState, "/joint_states", qos)
        self.pub_ft = self.create_publisher(
            WrenchStamped, "/force_torque_sensor_broadcaster/ft_data", qos)
        # rclpy zamanlayıcısı Python'da 500 Hz'i tutturamıyor (ölçüldü: ~400 Hz tavan,
        # yürütücü ek yükü yüzünden). Gerçek sürücü C++ ve 500 Hz'i tutar; testin
        # gerçekçi olması için burada ayrı bir iş parçacığı perf_counter ile ilerliyor.
        self.rate = float(g("rate"))
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"{L:,} örnek ({L/self.rate:.0f} s) yayınlanıyor @ {self.rate:.0f} Hz | "
            f"arıza: {self.fault}" + (f" (%50'den sonra)" if self.fault != "yok" else ""))

    def _run(self) -> None:
        period = 1.0 / self.rate
        t_next = time.perf_counter()
        while not self._stop and rclpy.ok():
            self.tick()
            t_next += period
            dt = t_next - time.perf_counter()
            if dt > 0:
                time.sleep(dt)
            else:
                t_next = time.perf_counter()      # geride kaldık, saati sıfırla

    def tick(self) -> None:
        if self.i >= self.n:
            if not self.loop:
                self.get_logger().info("Kayıt bitti."); self._stop = True; return
            self.i = 0
        i = self.i
        q, qd = self.Q[i].copy(), self.QD[i].copy()
        amps, wr = self.AMP[i].copy(), self.WR[i].copy()

        if self.fault != "yok" and i >= self.onset:
            prog = (i - self.onset) / max(self.n - self.onset, 1)
            if self.fault == "motor_kaymasi":
                amps[2] += 15.0 * prog / self.nm_per_amp[2]        # Eklem 3 torkuna rampa
            elif self.fault == "gizyazar":
                q[4] += 1.5                                        # Eklem 5 pozisyonuna basamak
            elif self.fault == "carpisma":
                c = self.onset + 0.15 * (self.n - self.onset)
                wr += 30.0 * np.exp(-0.5 * ((i - c) / (0.04 * self.n)) ** 2)
            elif self.fault == "sensor_gurultu":
                wr += self.rng.normal(0.0, 3.5, 6)

        now = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = now
        js.name = self.names
        js.position = q.tolist()
        js.velocity = qd.tolist()
        js.effort = amps.tolist()                                  # AMPER (sürücü gibi)
        self.pub_js.publish(js)

        ft = WrenchStamped()
        ft.header.stamp = now
        ft.header.frame_id = self.prefix + "tool0"
        ft.wrench.force.x, ft.wrench.force.y, ft.wrench.force.z = wr[:3]
        ft.wrench.torque.x, ft.wrench.torque.y, ft.wrench.torque.z = wr[3:]
        self.pub_ft.publish(ft)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ReplayPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node._stop = True
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
