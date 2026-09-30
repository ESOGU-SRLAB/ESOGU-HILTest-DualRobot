#!/usr/bin/env python3
"""
replay_publisher.py
===================
Kayıtlı bir oturumu gerçek sürücü gibi 500 Hz'de `/joint_states`e yayınlar —
robot olmadan `detector` düğümünü uçtan uca sınamak için. v4: KTS YOK, tek konu.

Sürücünün mesaj yapısını taklit eder:
    /joint_states                sensor_msgs/JointState
        name    = <tf_prefix><eklem>_joint
        effort  = motor AKIMI [A]   (kayıttaki tau [Nm] / nm_per_amp)

Varsayılan veri kaynağı `anomaly_detection_v2/dataset_prep/data/training_features.csv`
(Aşama A-C'den geçmiş, senaryo etiketli). `use_case` parametresiyle 4 senaryodan
biri seçilir; o senaryonun en uzun tek turu yayınlanır.

`--fault` verilirse kaydın ikinci yarısında ölçüm uzayında arıza üretir — aynı
4 tip, `dataset_prep/evaluate_faults.py` ile FİZİKSEL TUTARLI (arıza gerçek
sinyale eklenir, modele özel ayrı genlik yok):
    motor_kaymasi : Eklem 3 torkuna rampa (0 → 15 Nm)
    carpisma      : tüm eklem torklarına Gauss darbe (limitlerin ~%12'si)
    sensor_gurultu: tüm eklem torklarına Gauss gürültü
    gizyazar      : Eklem 5 pozisyonuna basamak (0,3 rad)

Kullanım
--------
    ros2 run anomaly_detection replay_publisher --ros-args -p use_case:=HRC
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

from sensor_msgs.msg import JointState

from .features import JOINT_SUFFIX

USE_CASES = ("HRC", "MULTIROBOT_INSPECTION", "PICKPLACE", "UR10E_INSPECTION")


def _base() -> str:
    """Varsayılan dosyaların kök dizini — kurulu share, yoksa kaynak ağacı."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("anomaly_detection")
    except Exception:
        return str(Path(__file__).resolve().parent.parent)


def _default_dataset() -> str:
    """v2 hattının çıktısı - anomaly_detection'ın kardeş paketinde durur (kurulu
    pakete kopyalanmaz, yalnız kaynak ağacından çalıştırıldığında bulunur)."""
    return str(Path(__file__).resolve().parent.parent.parent
               / "anomaly_detection_v2" / "dataset_prep" / "data" / "training_features.csv")


BASE = _base()


class ReplayPublisher(Node):

    def __init__(self):
        super().__init__("ur10e_replay_publisher")
        p = self.declare_parameter
        p("dataset", _default_dataset())
        p("current_to_torque", f"{BASE}/current_to_torque.json")
        p("use_case", "HRC")
        p("tf_prefix", "ur10e_")
        p("rate", 500.0)
        p("min_run", 3000)
        p("fault", "yok")          # yok | motor_kaymasi | carpisma | gizyazar | sensor_gurultu
        p("loop", True)

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.prefix = str(g("tf_prefix"))
        self.fault = str(g("fault"))
        self.loop = bool(g("loop"))
        use_case = str(g("use_case"))
        if use_case not in USE_CASES:
            raise SystemExit(f"use_case={use_case!r} değil; geçerli: {USE_CASES}")
        self.nm_per_amp = np.array(
            json.loads(Path(str(g("current_to_torque"))).read_text())["nm_per_amp"])

        import pandas as pd
        cols = (["run_id", "use_case"] + [f"q_{j}" for j in range(1, 7)]
                + [f"qd_{j}" for j in range(1, 7)] + [f"tau_{j}" for j in range(1, 7)])
        df = pd.read_csv(str(g("dataset")), usecols=cols)
        df = df[df.use_case == use_case]
        if df.empty:
            raise SystemExit(f"{g('dataset')} içinde use_case={use_case!r} satırı yok.")
        sizes = df.groupby("run_id").size()
        run_id = int(sizes[sizes >= int(g("min_run"))].idxmax()) if (sizes >= int(g("min_run"))).any() \
            else int(sizes.idxmax())
        seg = df[df.run_id == run_id]
        self.Q = seg[[f"q_{j}" for j in range(1, 7)]].to_numpy(np.float64)
        self.QD = seg[[f"qd_{j}" for j in range(1, 7)]].to_numpy(np.float64)
        self.TAU = seg[[f"tau_{j}" for j in range(1, 7)]].to_numpy(np.float64)
        del df, seg

        self.n = len(self.Q)
        self.onset = int(self.n * 0.5)
        self.i = 0
        self.rng = np.random.default_rng(0)
        self.names = [self.prefix + s for s in JOINT_SUFFIX]

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub_js = self.create_publisher(JointState, "/joint_states", qos)
        # rclpy zamanlayıcısı Python'da 500 Hz'i tutturamıyor (ölçüldü: ~400 Hz tavan);
        # ayrı bir iş parçacığı perf_counter ile ilerliyor.
        self.rate = float(g("rate"))
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"senaryo={use_case} (run_id={run_id}) {self.n:,} örnek "
            f"({self.n/self.rate:.0f} s) yayınlanıyor @ {self.rate:.0f} Hz | "
            f"arıza: {self.fault}" + (" (%50'den sonra)" if self.fault != "yok" else ""))

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
        q, qd, tau = self.Q[i].copy(), self.QD[i].copy(), self.TAU[i].copy()

        if self.fault != "yok" and i >= self.onset:
            prog = (i - self.onset) / max(self.n - self.onset, 1)
            if self.fault == "motor_kaymasi":
                tau[2] += 15.0 * prog                              # Eklem 3 torkuna rampa
            elif self.fault == "gizyazar":
                q[4] += 0.3                                        # Eklem 5 pozisyonuna basamak
            elif self.fault == "carpisma":
                c = self.onset + 0.15 * (self.n - self.onset)
                limit = np.array([330, 330, 150, 56, 56, 56], dtype=float)
                tau += 0.12 * limit * np.exp(-0.5 * ((i - c) / (0.04 * self.n)) ** 2)
            elif self.fault == "sensor_gurultu":
                tau += self.rng.normal(0.0, [3, 3, 2, 1, 1, 1])

        amps = tau / self.nm_per_amp                               # sürücü effort'u AMPER yazar

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.names
        js.position = q.tolist()
        js.velocity = qd.tolist()
        js.effort = amps.tolist()
        self.pub_js.publish(js)
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
