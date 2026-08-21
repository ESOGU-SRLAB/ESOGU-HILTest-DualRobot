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

import json
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
        p("wrench_topic", "/force_torque_sensor_broadcaster/wrench")
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
        # Bu iki değer 2026-08-21 gerçek robot kaydından geldi; ayrıntılı gerekçe
        # detector.py içinde. Kısaca: sınırsız dondurma kalıcı kilit üretiyordu ve
        # dururken öğrenilen taban çizgisi hareketli çalışma için ×400 hatalıydı.
        p("freeze_timeout", 3.0)      # s; bundan uzun alarm = rejim değişimi
        # ÖLÇÜM (2026-08-21, iki gerçek koşu): kapı açıkken taban çizgisine kararların
        # yalnız %13'ü giriyor ve eşik 21,45'e çıkıyor -- verinin tamamının üstünde,
        # yani uyarlanabilir kural ölüyor. Varsayım ("dururken skor düşük") yanlış
        # çıktı: skor poza bağlı, yerçekimi yüklü duruşta 4,30, harekette 1,58.
        # Bu yüzden varsayılan KAPALI. Kilidi kıran dondurma sınırıydı, kapı değil.
        p("motion_qd_min", -1.0)      # rad/s; negatif = kapı kapalı
        # fusion_config.json'daki θ FMU verisinden geliyor ve gerçek robotta 7 kat
        # düşük (temiz koşu medyanı 4,27, θ 0,6436 -> kararların %97'si alarm).
        # Pozitif verilirse config'deki değerin yerine geçer.
        p("threshold_override", 0.0)  # <=0: fusion_config.json'daki θ kullanılır
        # Kayıt: boş dize verilirse tamamen kapanır. Merdivenin 0-2. adımları
        # (taban çizgisi toplama + yeniden kalibrasyon + pasif uyarı) bu iki
        # dosyayla yürür; robota hiçbir komut gitmez.
        p("log_dir", str(Path.home() / "anomali_kayit"))
        p("log_scores", True)

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
            freeze_timeout=float(g("freeze_timeout")),
            motion_qd_min=float(g("motion_qd_min")),
        )
        d = self.det
        ovr = float(g("threshold_override"))
        if ovr > 0.0:
            self.get_logger().warn(
                f"θ_mutlak elle geçersiz kılındı: {d.thr_fused:.5f} -> {ovr:.5f} "
                f"(fusion_config.json değiştirilmedi)")
            d.thr_fused = ovr
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
                f"({d.adaptive_window*d.decision_period:.0f} s); taban çizgisi yalnız "
                f"|q̇|>{d.motion_qd_min} rad/s iken beslenir, dondurma en fazla "
                f"{d.freeze_timeout:.0f} s ({int(d.freeze_timeout/d.decision_period)} karar)")
        else:
            self.get_logger().info(
                "Uyarlanabilir kural kapalı (gerçek robotta ölçülen varsayılan). "
                "Karşılığı: çok yavaş kayan bozulmalar mutlak eşiğe ulaşana kadar "
                "görünmez kalır.")
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
        # İsim kümesi -> eklem indeksleri. None = bu yayıncı UR değil.
        self.jidx_cache: dict[tuple, list[int] | None] = {}
        self.n_samples = 0
        self.n_scores = 0
        self.n_stale = 0
        self.n_foreign = 0
        self.n_short = 0
        self.n_alarms = 0
        self.health_state: str | None = None
        self.t_health = time.monotonic()
        self.n_samples_health = 0
        self.n_scores_health = 0
        self.alarm_active = False
        self.alarm_t0 = 0.0
        self.alarm_peak = 0.0
        self.f_events = None
        self.f_scores = None
        self.t_flush = time.monotonic()
        self._open_logs(str(g("log_dir")).strip(), bool(g("log_scores")))
        self.infer_ms: deque = deque(maxlen=200)

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

    # ── kayıt ──
    def _open_logs(self, log_dir: str, want_scores: bool) -> None:
        """İki dosya açar: olaylar_*.jsonl (yalnız alarmlar) ve skorlar_*.csv
        (her karar). CSV, merdivenin 1. adımında θ ile k'yı gerçek robot verisiyle
        yeniden ölçmek için gereken ham girdidir; 20 Hz'de ~170 MB/gün eder."""
        if not log_dir:
            self.get_logger().warn("Kayıt kapalı (log_dir boş).")
            return
        d = Path(log_dir).expanduser()
        d.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.f_events = open(d / f"olaylar_{ts}.jsonl", "w", buffering=1)
        if want_scores:
            self.f_scores = open(d / f"skorlar_{ts}.csv", "w")
            self.f_scores.write(
                "t_ros,s_kal,s_ham,z_kal,z_ham,birlesik,thr_mutlak,thr_uyarlanabilir,"
                "hit_mutlak,hit_uyarlanabilir,hit_kal,hit_ham,hareket,qd_tepe,"
                "taban_n,donmus,alarm\n")
        self.get_logger().info(
            f"Kayıt: {d}/olaylar_{ts}.jsonl"
            + (f" + skorlar_{ts}.csv" if want_scores else " (skor CSV kapalı)"))

    def _event(self, kind: str, **kw) -> None:
        if self.f_events is None:
            return
        rec = {"zaman": time.strftime("%Y-%m-%dT%H:%M:%S"), "olay": kind}
        rec.update(kw)
        self.f_events.write(json.dumps(rec, ensure_ascii=False) + "\n")

    def _resolve(self, names: tuple) -> list[int] | None:
        """İsim kümesini UR eklem indekslerine çevirir; UR'a ait değilse None."""
        want = [self.tf_prefix + s for s in JOINT_SUFFIX]
        lst = list(names)
        try:
            idx = [lst.index(n) for n in want]
        except ValueError:
            self.get_logger().warn(
                f"UR dışı yayıncı yok sayılıyor ({len(lst)} eklem): {lst}")
            return None
        self.get_logger().info(f"Eklem eşlemesi kuruldu {idx} ← {lst}")
        return idx

    def on_joint_states(self, msg: JointState) -> None:
        if not msg.velocity or not msg.effort:
            return

        # Aynı /joint_states üzerinde birden çok yayıncı olabilir (UR
        # joint_state_broadcaster + AGV köprüsü). Bu yüzden eşleme MESAJ BAŞINA,
        # isim kümesine göre çözülür ve isim kümesi başına önbelleklenir. Tek sefer
        # kurulup sonraki mesajlara körlemesine uygulanırsa — eski davranış — iki
        # yayıncının eklem sırası farklı olduğunda sessizce yanlış eklemler okunur.
        key = tuple(msg.name)
        try:
            i = self.jidx_cache[key]
        except KeyError:
            i = self.jidx_cache[key] = self._resolve(key)
        if i is None:
            self.n_foreign += 1
            return
        if max(i) >= min(len(msg.position), len(msg.velocity), len(msg.effort)):
            self.n_short += 1                    # alan uzunlukları isimlerle uyumsuz
            return

        if self.last_wrench is None:
            return
        if time.monotonic() - self.last_wrench_t > self.max_wrench_age:
            self.n_stale += 1                    # bayat wrench ile skor üretme
            return

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
                    float(r["hit_residual"]), float(r["hit_raw"]),
                    float(r["moving"]), float(r["qd_peak"]),
                    float(r["baseline_n"]), float(r["frozen"])]
        self.pub_detail.publish(det)

        t_ros = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.f_scores is not None:
            self.f_scores.write(
                f"{t_ros:.4f}," + ",".join(f"{v:.6g}" for v in det.data)
                + f",{int(alarm)}\n")
            now = time.monotonic()
            if now - self.t_flush > 2.0:           # 2 s'de bir diske indir
                self.f_scores.flush()
                self.t_flush = now

        if alarm and not self.alarm_active:
            self.alarm_active = True
            self.alarm_t0 = time.monotonic()
            self.alarm_peak = float(r["fused"])
            self.n_alarms += 1
            which = "+".join([n for n, h in (("kalıntı", r["hit_residual"]),
                                             ("ham", r["hit_raw"])) if h]) or "yalnız birleşim"
            rule = "+".join([n for n, h in (("mutlak", r["hit_absolute"]),
                                             ("uyarlanabilir", r["hit_adaptive"])) if h])
            lim = (r["threshold"] if r["hit_absolute"] else r["adaptive_threshold"])
            self.get_logger().warn(
                f"ANOMALİ  birleşik={r['fused']:.4f} > {lim:.4f} ({rule})  "
                f"(kalıntı {r['s_residual']:.3f}/θ{self.det.ae_res.threshold:.3f}, "
                f"ham {r['s_raw']:.3f}/θ{self.det.ae_raw.threshold:.3f})  tetikleyen: {which}")
            self._event("anomali_basladi", t_ros=round(t_ros, 4), sira=self.n_alarms,
                        birlesik=round(float(r["fused"]), 5), esik=round(float(lim), 5),
                        kural=rule, tetikleyen=which,
                        s_kal=round(float(r["s_residual"]), 5),
                        s_ham=round(float(r["s_raw"]), 5),
                        hareket=bool(r["moving"]), qd_tepe=round(float(r["qd_peak"]), 4),
                        taban_n=int(r["baseline_n"]),
                        q=[round(float(v), 5) for v in q],
                        qd=[round(float(v), 5) for v in qd],
                        akim=[round(float(v), 4) for v in amps])
        elif alarm:
            self.alarm_peak = max(self.alarm_peak, float(r["fused"]))
        elif self.alarm_active:
            self.alarm_active = False
            sure = time.monotonic() - self.alarm_t0
            self._event("anomali_bitti", t_ros=round(t_ros, 4), sira=self.n_alarms,
                        sure_s=round(sure, 3), tepe=round(self.alarm_peak, 5))
            self.get_logger().info(
                f"Anomali sona erdi (#{self.n_alarms}, {sure:.2f} s, "
                f"tepe {self.alarm_peak:.4f})")

    def on_health(self) -> None:
        """Sağlık raporu. Düğüm büyük bir launch içinde ortak terminale yazdığı için
        periyodik satır basmaz: durum DEĞİŞTİĞİNDE bir satır yazar. Pratikte bu,
        açılışta bir satır ve sonrasında sessizlik demektir.

        Hız pencereli ölçülür. Kümülatif ortalama, düğüm daha yeni başlarken düşük
        çıkıp yavaşça tırmanıyor (ölçülen: 365 → 495 Hz) ve ilk raporu yanıltıyordu.
        """
        now = time.monotonic()
        el = max(now - self.t_health, 1e-6)
        rate = (self.n_samples - self.n_samples_health) / el
        dec = (self.n_scores - self.n_scores_health) / el
        self.t_health, self.n_samples_health = now, self.n_samples
        self.n_scores_health = self.n_scores

        ms = float(np.mean(self.infer_ms)) if self.infer_ms else 0.0
        budget = self.det.stride * self.det.extractor.dt * 1e3
        text = (f"örnek {rate:6.0f} Hz | karar {dec:5.1f} Hz | "
                f"çıkarım {ms:5.2f} ms / {budget:.0f} ms bütçe (%{100*ms/budget:.0f})"
                + (f" | bayat wrench {self.n_stale:,}" if self.n_stale else "")
                + (f" | UR dışı {self.n_foreign:,}" if self.n_foreign else "")
                + (f" | kısa mesaj {self.n_short:,}" if self.n_short else "")
                + (f" | alarm {self.n_alarms}" if self.n_alarms else ""))

        durum = "veri_yok" if rate <= 0 else ("iyi" if rate > 400 else "dusuk")
        if durum == self.health_state:
            return                              # durum değişmedi -> sessiz kal
        self.health_state = durum

        # NOT: rclpy günlükçüsü şiddeti ÇAĞRI YERİNE göre önbelleğe alıyor; aynı satırdan
        # bir kez info bir kez warn çağırmak "Logger severity cannot be changed between
        # calls" ile düğümü düşürüyor. Bu yüzden ayrı çağrı yerleri.
        if durum == "iyi":
            self.get_logger().info(text)
        elif durum == "dusuk":
            self.get_logger().warn(
                text + "  —  örnek hızı 500 Hz'in altında; q̈ türevi sabit dt "
                "varsayıyor, düşük hızda kalıntı bozulur.")
        else:
            self.get_logger().warn("Veri gelmiyor — /joint_states ve wrench konularını "
                                   "kontrol et.")


    def close_logs(self) -> None:
        for f in (self.f_scores, self.f_events):
            if f is not None:
                f.flush()
                f.close()


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
            node.close_logs()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
