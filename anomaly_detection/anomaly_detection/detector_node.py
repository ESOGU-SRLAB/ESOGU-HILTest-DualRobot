#!/usr/bin/env python3
"""
detector_node.py
================
UR10e için çevrimiçi anomali tespiti — iki LSTM özkodlayıcının skor düzeyinde
birleşimi (245.pdf, Denklem 4).

Akış
----
    /joint_states (500 Hz) ─┐
                            ├─→ OnlineFeatureExtractor ─→ 12 kanal kalıntı
    /…/ft_data    (500 Hz) ─┘        (50 ms gecikme)      24 kanal ham
                                                              │
                            her `stride` örnekte (20 Hz) ─────┤
                                                              ▼
              iki ONNX özkodlayıcı → yeniden yapılanma MSE → min–max → ağırlıklı
              ortalama → eşik → ~/detected

Gecikme bütçesi (ölçülmüş)
--------------------------
    örnek başına  : ters dinamik 22 µs + Jacobian ≈ 30 µs   → 500 Hz'in %1,5'i
    karar başına  : 2 ONNX ileri geçişi ≈ 4–12 ms           → 50 ms'in %9–24'ü
    tespit gecikmesi: SG 50 ms + %10 örtüşme 20 ms + karar periyodu 50 ms ≈ 120–150 ms

Bu düğüm ince bir sarmalayıcıdır; tüm hesap `detector.FusionDetector` içinde ve
`replay_detector.py` tarafından gerçek veriyle doğrulanır.
"""

from __future__ import annotations

import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray

from .detector import FusionDetector
from .features import JOINT_SUFFIX

def _default_base() -> str:
    """Model ve kalibrasyon dosyalarının kök dizini.

    Öncelik kurulu `share/anomaly_detection` dizinindedir; böylece depoyu klonlayan
    herkeste mutlak yol düzenlemesi gerekmez. Varlıklar orada yoksa (ör. paket
    kurulmadan doğrudan kaynak ağacından çalıştırılıyorsa) kaynak dizinine düşülür.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("anomaly_detection"))
        if (share / "fusion_v2" / "fusion_config.json").is_file():
            return str(share)
    except Exception:
        pass
    return str(Path(__file__).resolve().parent.parent)


DEFAULT_BASE = _default_base()


class AnomalyDetectorNode(Node):

    def __init__(self):
        super().__init__("ur10e_anomaly_detector")

        p = self.declare_parameter
        p("residual_model_dir", f"{DEFAULT_BASE}/residual_ae_v2")
        p("raw_model_dir", f"{DEFAULT_BASE}/raw_ae_v2")
        p("fusion_config", f"{DEFAULT_BASE}/fusion_v2/fusion_config.json")
        p("current_to_torque", f"{DEFAULT_BASE}/current_to_torque.json")
        p("residual_calibration", f"{DEFAULT_BASE}/residual_calibration_clean.json")
        p("solver_resources", f"{DEFAULT_BASE}/resources")
        p("joint_states_topic", "/joint_states")
        p("wrench_topic", "/force_torque_sensor_broadcaster/ft_data")
        p("tf_prefix", "ur10e_")
        p("stride", 25)
        p("fts_frame", "tool")
        p("consecutive_for_alarm", 2)
        p("max_wrench_age", 0.02)
        # Uyarlanabilir kural: mutlak eşik θ doğrulama setinin EN KÖTÜ uyan
        # koşularından gelir, iyi uyan bir koşuda taban çizgisi çok altta oturur ve
        # oradaki gerçek arıza θ'yı hiç geçemez. tune_adaptive.py ile ölçüldü:
        # yalnız θ ile motor kayması ve gizyazar hiç yakalanmıyor; kural eklenince
        # dört arıza tipi de her koşuda yakalanıyor, yanlış alarm %0,07.
        p("adaptive", True)
        p("adaptive_window", 600)      # 600 karar = 30 s taban çizgisi
        p("adaptive_k", 8.0)           # plato 6–12 ölçüldü; ortası seçildi
        p("adaptive_warmup", 200)

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.tf_prefix = str(g("tf_prefix"))
        self.max_wrench_age = float(g("max_wrench_age"))
        self.need_consecutive = int(g("consecutive_for_alarm"))

        self.get_logger().info("UR10e anomali tespiti başlatılıyor...")

        sys.path.insert(0, str(Path(g("solver_resources")).resolve()))
        import ur10_solver_py                                    # FMU ile aynı çekirdek

        self.det = FusionDetector(
            residual_model_dir=g("residual_model_dir"),
            raw_model_dir=g("raw_model_dir"),
            fusion_config=g("fusion_config"),
            current_to_torque=g("current_to_torque"),
            residual_calibration=g("residual_calibration"),
            solver=ur10_solver_py.InverseDynamicsSolverUR10(),
            stride=int(g("stride")),
            fts_frame=str(g("fts_frame")),
            adaptive=bool(g("adaptive")),
            adaptive_window=int(g("adaptive_window")),
            adaptive_k=float(g("adaptive_k")),
            adaptive_warmup=int(g("adaptive_warmup")),
        )
        d = self.det
        self.get_logger().info(
            f"  kalıntı: {d.ae_res.n_feat} kanal θ={d.ae_res.threshold:.4f} ({d.ae_res.provider})")
        self.get_logger().info(
            f"  ham    : {d.ae_raw.n_feat} kanal θ={d.ae_raw.threshold:.4f} ({d.ae_raw.provider})")
        self.get_logger().info(
            f"  birleşim: w_kal={d.w_res:.2f} w_ham={d.w_raw:.2f} θ_mutlak={d.thr_fused:.5f} "
            f"ölçek={d.norm} (kalıntı z=(S−{d.res_lo:.4g})/{d.res_span:.4g}, "
            f"ham z=(S−{d.raw_lo:.4g})/{d.raw_span:.4g})")
        if d.adaptive:
            self.get_logger().info(
                f"  uyarlanabilir kural açık: medyan + {d.adaptive_k:.0f}·MAD, "
                f"{d.adaptive_window} karar penceresi "
                f"({d.adaptive_window*d.stride*d.extractor.dt:.0f} s)")
        else:
            self.get_logger().warn(
                "Uyarlanabilir kural KAPALI — yalnız mutlak eşikle yavaş kayma ve "
                "iyi uyan koşulardaki arızalar kaçırılır.")
        if not all(d.trusted):
            names = [JOINT_SUFFIX[i].replace("_joint", "")
                     for i, t in enumerate(d.trusted) if not t]
            self.get_logger().warn(
                f"Akım→tork katsayısı ölçülemeyen eklemler: {', '.join(names)}. "
                f"Tork ölçeği bu kanallarda varsayımdır — tespiti etkilemez, "
                f"Nm cinsinden yorumu etkiler.")

        self.consecutive = 0
        self.last_wrench: np.ndarray | None = None
        self.last_wrench_t = 0.0
        self.jidx: list[int] | None = None
        self.n_samples = 0
        self.n_scores = 0
        self.n_stale = 0
        self.infer_ms: deque = deque(maxlen=200)
        self.t_start = time.monotonic()

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(WrenchStamped, str(g("wrench_topic")), self.on_wrench, qos)
        self.create_subscription(JointState, str(g("joint_states_topic")),
                                 self.on_joint_states, qos)
        self.pub_score = self.create_publisher(Float32, "~/score", 10)
        self.pub_det = self.create_publisher(Bool, "~/detected", 10)
        self.pub_detail = self.create_publisher(Float32MultiArray, "~/detail", 10)
        self.create_timer(10.0, self.on_health)

        self.get_logger().info(
            f"Hazır. Tespit gecikmesi ≈ {d.lag_seconds*1000:.0f} ms "
            f"(SG {d.extractor.lag_seconds*1000:.0f} ms + karar periyodu "
            f"{d.stride*d.extractor.dt*1000:.0f} ms).")

    # ── geri çağırmalar ──
    def on_wrench(self, msg: WrenchStamped) -> None:
        w = msg.wrench
        self.last_wrench = np.array([w.force.x, w.force.y, w.force.z,
                                     w.torque.x, w.torque.y, w.torque.z], dtype=np.float64)
        self.last_wrench_t = time.monotonic()

    def on_joint_states(self, msg: JointState) -> None:
        if self.last_wrench is None:
            return
        if time.monotonic() - self.last_wrench_t > self.max_wrench_age:
            self.n_stale += 1                      # bayat wrench ile skor üretme
            return
        if not msg.velocity or not msg.effort:
            return

        if self.jidx is None:
            want = [self.tf_prefix + s for s in JOINT_SUFFIX]
            try:
                self.jidx = [list(msg.name).index(n) for n in want]
            except ValueError:
                self.get_logger().error(
                    f"Eklem adları bulunamadı. Beklenen: {want}\n"
                    f"  Gelen: {list(msg.name)}\n  'tf_prefix' parametresini düzelt.")
                return
            self.get_logger().info(f"Eklem eşlemesi kuruldu: {self.jidx}")

        i = self.jidx
        q = np.array([msg.position[k] for k in i], dtype=np.float64)
        qd = np.array([msg.velocity[k] for k in i], dtype=np.float64)
        amps = np.array([msg.effort[k] for k in i], dtype=np.float64)

        self.n_samples += 1
        t0 = time.perf_counter()
        r = self.det.push(q, qd, amps, self.last_wrench)
        if r is None:
            return
        self.infer_ms.append((time.perf_counter() - t0) * 1e3)
        self.n_scores += 1

        self.consecutive = self.consecutive + 1 if r["detected"] else 0
        alarm = self.consecutive >= self.need_consecutive

        self.pub_score.publish(Float32(data=float(r["fused"])))
        self.pub_det.publish(Bool(data=bool(alarm)))
        det = Float32MultiArray()
        thr_ad = r["adaptive_threshold"]
        det.data = [float(r["s_residual"]), float(r["s_raw"]),
                    float(r["z_residual"]), float(r["z_raw"]),
                    float(r["fused"]), float(r["threshold"]),
                    float(thr_ad if np.isfinite(thr_ad) else -1.0),
                    float(r["hit_absolute"]), float(r["hit_adaptive"]),
                    float(r["hit_residual"]), float(r["hit_raw"])]
        self.pub_detail.publish(det)

        if alarm and self.consecutive == self.need_consecutive:
            which = "+".join([n for n, h in (("kalıntı", r["hit_residual"]),
                                             ("ham", r["hit_raw"])) if h]) or "yalnız birleşim"
            rule = "+".join([n for n, h in (("mutlak", r["hit_absolute"]),
                                             ("uyarlanabilir", r["hit_adaptive"])) if h])
            lim = (r["threshold"] if r["hit_absolute"] else r["adaptive_threshold"])
            self.get_logger().warn(
                f"ANOMALİ  birleşik={r['fused']:.4f} > {lim:.4f} ({rule})  "
                f"(kalıntı {r['s_residual']:.3f}/θ{self.det.ae_res.threshold:.3f}, "
                f"ham {r['s_raw']:.3f}/θ{self.det.ae_raw.threshold:.3f})  tetikleyen: {which}")

    def on_health(self) -> None:
        el = max(time.monotonic() - self.t_start, 1e-6)
        rate = self.n_samples / el
        ms = float(np.mean(self.infer_ms)) if self.infer_ms else 0.0
        budget = self.det.stride * self.det.extractor.dt * 1e3
        text = (f"örnek {rate:6.0f} Hz | karar {self.n_scores/el:5.1f} Hz | "
                f"çıkarım {ms:5.2f} ms / {budget:.0f} ms bütçe (%{100*ms/budget:.0f})"
                + (f" | bayat wrench {self.n_stale:,}" if self.n_stale else ""))
        # NOT: rclpy günlükçüsü şiddeti ÇAĞRI YERİNE göre önbelleğe alıyor; aynı satırdan
        # bir kez info bir kez warn çağırmak "Logger severity cannot be changed between
        # calls" ile düğümü düşürüyor. Bu yüzden iki ayrı çağrı yeri.
        if rate > 400:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
            if rate > 0:
                self.get_logger().warn(
                    "Örnek hızı 500 Hz'in altında — q̈ türevi sabit dt varsayıyor, "
                    "düşük hızda kalıntı bozulur.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AnomalyDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
