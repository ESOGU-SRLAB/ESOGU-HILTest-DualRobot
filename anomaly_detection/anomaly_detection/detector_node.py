#!/usr/bin/env python3
"""
detector_node.py
================
UR10e için çevrimiçi anomali tespiti — iki LSTM özkodlayıcının skor düzeyinde
birleşimi. v4: KTS YOK (yalnız /joint_states dinlenir), kalıntı hesabı
SENARYOya (use_case) bağlı.

Akış
----
    /joint_states (500 Hz) ─→ OnlineFeatureExtractor ─→ 6 kanal kalıntı (r_tot)
                                    (50 ms gecikme)      16 kanal ham (q_2..5,q̇,τ)
                                                              │
                            her `stride` örnekte (20 Hz) ─────┤
                                                              ▼
              iki ONNX özkodlayıcı → log-normalize → ağırlıklı ortalama →
              rejim-koşullu eşik (+ uyarlanabilir kural) → ~/detected

Bu düğüm ince bir sarmalayıcıdır; tüm hesap `detector.FusionDetector` içinde.
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
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

# user_interface/app.py'nin yayınladığı, hangi senaryonun koştuğunu söyleyen
# latch'li topic (bkz. UseCaseBroadcaster). Bu düğüm BUNA GÖRE kendi use_case'ini
# DEĞİŞTİRMEZ (fizik modeli kuruluşta sabitlenir, canlı yeniden kurmak riskli) -
# yalnız uyuşmazlığı YÜKSEK SESLE bildirir. 30.09.2026'da tam olarak bu uyuşmazlık
# (launch hep varsayılan UR10E_INSPECTION ile başlamış, arayüz HRC/PICKPLACE
# koşturmuş) 309 yanlış alarma yol açmıştı.
TESTBED_USE_CASE_TOPIC = "/testbed/use_case"

from .detector import FusionDetector
from .features import JOINT_SUFFIX

USE_CASES = ("HRC", "MULTIROBOT_INSPECTION", "PICKPLACE", "UR10E_INSPECTION")


def _default_base() -> str:
    """Model ve kalibrasyon dosyalarının kök dizini - v4 varlıklarını (fusion_v4/)
    yoklar. Kurulu share bayatsa (yalnız eski fusion_v3/ varsa) gürültülü uyarır -
    26.08.2026'da tam olarak bu boşluktan ölçüm eski modellerle alınmıştı."""
    src = str(Path(__file__).resolve().parent.parent)
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("anomaly_detection"))
        if (share / "fusion_v4" / "fusion_config.json").is_file():
            return str(share)
        print(
            "\n" + "!" * 72 +
            f"\nBAYAT KURULUM: {share}\n"
            "  Kurulu paket v4 varlıklarını (fusion_v4/) taşımıyor.\n"
            "  `colcon build --packages-select anomaly_detection` çalıştırın,\n"
            "  yoksa ölçüm ESKİ modellerle alınır.\n"
            f"  Şimdilik kaynak ağacına düşülüyor: {src}\n"
            + "!" * 72 + "\n", file=sys.stderr)
    except Exception:
        pass
    return src


DEFAULT_BASE = _default_base()


class AnomalyDetectorNode(Node):

    def __init__(self):
        super().__init__("ur10e_anomaly_detector")

        p = self.declare_parameter
        p("residual_model_dir", f"{DEFAULT_BASE}/residual_ae_v4")
        p("raw_model_dir", f"{DEFAULT_BASE}/raw_ae_v4")
        p("fusion_config", f"{DEFAULT_BASE}/fusion_v4/fusion_config.json")
        p("current_to_torque", f"{DEFAULT_BASE}/current_to_torque.json")
        p("residual_calibration", f"{DEFAULT_BASE}/residual_calibration.json")
        # ZORUNLU niyetinde: yük/ofset düzeltmesi buna göre seçilir. Yanlış
        # senaryo verilirse kalıntı yüzlerce Nm sapabilir (residual_calibration.json'daki
        # senaryolar arası fark - bkz. dosyanın b_by_use_case/payload_by_use_case alanları).
        p("use_case", "UR10E_INSPECTION")
        # Eşik persentili: fusion_config.json dört tanesini de taşıyor (p97..p99.99).
        # p99.9 varsayılan - offline ölçümde p97 saatte binlerce yanlış alarma denk
        # geliyordu (dataset_prep/evaluate_faults.py). Gerçek robotta yeniden ölçülecek.
        p("quantile", "p99.9")
        p("regime_threshold", "auto")
        p("solver_resources", f"{DEFAULT_BASE}/resources")
        p("joint_states_topic", "/joint_states")
        p("tf_prefix", "ur10e_")
        p("stride", 25)
        p("consecutive_for_alarm", 2)
        # Uyarlanabilir kural: v3'te gerçek robotta (21.08.2026) kalıntının POZA
        # bağlı olduğu ölçüldüğü için kapatılmıştı. v4'ün senaryo-başına yük
        # düzeltmesi bu bağımlılığı azaltmış olabilir - Pazartesi testinde
        # değerlendirilip karar verilecek, o yüzden varsayılan hâlâ KAPALI.
        p("adaptive", False)
        p("adaptive_window", 600)
        p("adaptive_k", 8.0)
        p("adaptive_warmup", 200)
        p("freeze_timeout", 3.0)
        p("motion_qd_min", -1.0)       # -1 = fusion_config.json'daki (0,02)
        p("log_dir", str(Path.home() / "anomali_kayit"))
        p("log_scores", True)

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.tf_prefix = str(g("tf_prefix"))
        self.need_consecutive = int(g("consecutive_for_alarm"))

        use_case = str(g("use_case"))
        if use_case not in USE_CASES:
            self.get_logger().warn(
                f"use_case={use_case!r} bilinen 4 senaryodan biri değil "
                f"({USE_CASES}) - residual_calibration.json'da bu isim bulunamazsa "
                f"düğüm hemen hata verip duracak.")

        self.get_logger().info(
            f"UR10e anomali tespiti başlatılıyor (senaryo={use_case})...")

        sys.path.insert(0, str(Path(g("solver_resources")).resolve()))
        import ur10_solver_py                                    # FMU ile aynı çekirdek

        self.det = FusionDetector(
            residual_model_dir=g("residual_model_dir"),
            raw_model_dir=g("raw_model_dir"),
            fusion_config=g("fusion_config"),
            current_to_torque=g("current_to_torque"),
            residual_calibration=g("residual_calibration"),
            use_case=use_case,
            solver=ur10_solver_py.InverseDynamicsSolverUR10(),
            stride=int(g("stride")),
            quantile=str(g("quantile")),
            regime_threshold=({"auto": None, "true": True, "false": False}
                              .get(str(g("regime_threshold")).lower(), None)),
            adaptive=bool(g("adaptive")),
            adaptive_window=int(g("adaptive_window")),
            adaptive_k=float(g("adaptive_k")),
            adaptive_warmup=int(g("adaptive_warmup")),
            freeze_timeout=float(g("freeze_timeout")),
            motion_qd_min=(None if float(g("motion_qd_min")) < 0
                          else float(g("motion_qd_min"))),
        )
        d = self.det
        self.get_logger().info(
            f"  kalıntı: {d.ae_res.n_feat} kanal θ={d.ae_res.threshold:.4f} ({d.ae_res.provider})")
        self.get_logger().info(
            f"  ham    : {d.ae_raw.n_feat} kanal θ={d.ae_raw.threshold:.4f} ({d.ae_raw.provider})")
        self.get_logger().info(
            f"  birleşim: w_kal={d.w_res:.2f} w_ham={d.w_raw:.2f} eşik={d.quantile} "
            f"θ_duran={d.thr_regime['static']:.4f} θ_hareketli={d.thr_regime['moving']:.4f} "
            f"(|q̇|>{d.motion_qd_min:g} rad/s ile seçilir)")
        if d.adaptive:
            self.get_logger().info(
                f"  uyarlanabilir kural açık: medyan + {d.adaptive_k:.0f}·MAD, "
                f"{d.adaptive_window} karar penceresi "
                f"({d.adaptive_window*d.decision_period:.0f} s), dondurma en fazla "
                f"{d.freeze_timeout:.0f} s ({int(d.freeze_timeout/d.decision_period)} karar)")
        else:
            self.get_logger().info("Uyarlanabilir kural kapalı (v3'te gerçek robotta ölçülen "
                                   "varsayılan - v4'te henüz doğrulanmadı).")
        if not all(d.trusted):
            names = [JOINT_SUFFIX[i].replace("_joint", "")
                     for i, t in enumerate(d.trusted) if not t]
            self.get_logger().warn(
                f"Akım→tork katsayısı ölçülemeyen eklemler: {', '.join(names)}. "
                f"Tork ölçeği bu kanallarda varsayımdır (aile katsayısı) — tespiti "
                f"etkilemez, Nm cinsinden yorumu etkiler.")

        self.consecutive = 0
        self.jidx_cache: dict[tuple, list[int] | None] = {}
        self.n_samples = 0
        self.n_scores = 0
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
        self.create_subscription(JointState, str(g("joint_states_topic")),
                                 self.on_joint_states, qos)

        # Arayüzün yayınladığı aktif senaryoyla uyuşmazlığı canlı yakala (bkz.
        # dosya başındaki TESTBED_USE_CASE_TOPIC notu). TRANSIENT_LOCAL ŞART -
        # yayıncı (UseCaseBroadcaster) latch'li, bu düğüm ondan SONRA başlasa
        # bile son değeri bu sayede kaçırmaz.
        tb_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._last_testbed_use_case: str | None = None
        self.create_subscription(String, TESTBED_USE_CASE_TOPIC,
                                 self.on_testbed_use_case, tb_qos)

        self.pub_score = self.create_publisher(Float32, "~/score", 10)
        self.pub_det = self.create_publisher(Bool, "~/detected", 10)
        self.pub_detail = self.create_publisher(Float32MultiArray, "~/detail", 10)
        self.create_timer(10.0, self.on_health)

        self.get_logger().info(
            f"Hazır. Tespit gecikmesi ≈ {d.lag_seconds*1000:.0f} ms "
            f"(SG {d.extractor.lag_seconds*1000:.0f} ms + karar periyodu "
            f"{d.stride*d.extractor.dt*1000:.0f} ms).")

    # ── kayıt ──
    def _open_logs(self, log_dir: str, want_scores: bool) -> None:
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
                "taban_n,donmus,thr_kal,thr_ham,alarm\n")
        self._write_run_meta(d, ts)
        self.get_logger().info(
            f"Kayıt: {d}/olaylar_{ts}.jsonl"
            + (f" + skorlar_{ts}.csv" if want_scores else " (skor CSV kapalı)"))

    def _write_run_meta(self, d: Path, ts: str) -> None:
        """Oturumun KÖKENİNİ yazar (hangi modeller/eşikler/senaryo) - yol değil,
        içerik ve karma. Diskte birden çok model kuşağı durabilir; bir kaydın
        hangisiyle alındığını altı ay sonra yalnız buradan güvenle çözebiliriz."""
        import hashlib
        import subprocess

        def digest(path) -> dict:
            try:
                p_ = Path(path)
                if p_.is_dir():
                    p_ = p_ / "model.onnx"
                h = hashlib.sha256(p_.read_bytes()).hexdigest()[:16]
                return {"path": str(path), "sha256_16": h, "bytes": p_.stat().st_size}
            except Exception as e:
                return {"path": str(path), "error": str(e)}

        det = self.det
        g = lambda k: self.get_parameter(k).value       # noqa: E731
        meta = {
            "timestamp": ts,
            "node": "detector_node",
            "use_case": det.use_case,
            "residual_model": digest(g("residual_model_dir")),
            "raw_model": digest(g("raw_model_dir")),
            "fusion_config_path": str(g("fusion_config")),
            "current_to_torque": digest(g("current_to_torque")),
            "residual_calibration": digest(g("residual_calibration")),
            "w_res": det.w_res, "w_raw": det.w_raw,
            "regime_threshold": det.regime_threshold,
            "quantile": det.quantile,
            "threshold_by_regime": det.thr_regime,
            "threshold_by_regime_source": det.thr_regime_source,
            "motion_qd_min": det.motion_qd_min,
            "residual_theta": det.ae_res.threshold,
            "raw_theta": det.ae_raw.threshold,
            "stride": det.stride,
            "adaptive": det.adaptive, "adaptive_k": det.adaptive_k,
            "adaptive_window": det.adaptive_window,
            "freeze_timeout": det.freeze_timeout,
            "providers": [det.ae_res.provider, det.ae_raw.provider],
            "joint_states_topic": str(g("joint_states_topic")),
        }
        try:
            meta["fusion_config"] = json.loads(
                Path(str(g("fusion_config"))).read_text(encoding="utf-8"))
        except Exception as e:
            meta["fusion_config"] = {"error": str(e)}
        try:
            meta["git_commit"] = subprocess.run(
                ["git", "-C", str(Path(__file__).resolve().parent), "rev-parse", "HEAD"],
                capture_output=True, text=True, timeout=5).stdout.strip() or None
        except Exception:
            meta["git_commit"] = None
        try:
            (d / f"kosu_{ts}.json").write_text(
                json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
            self.get_logger().info(
                f"Köken: {d}/kosu_{ts}.json  "
                f"(senaryo={det.use_case}, w={det.w_res:.2f}, "
                f"eşik={'rejim' if det.regime_threshold else 'global'}/{det.quantile})")
        except Exception as e:
            self.get_logger().error(f"kosu_{ts}.json yazılamadı: {e}")

    def _event(self, kind: str, **kw) -> None:
        if self.f_events is None:
            return
        rec = {"zaman": time.strftime("%Y-%m-%dT%H:%M:%S"), "olay": kind}
        rec.update(kw)
        self.f_events.write(json.dumps(rec, ensure_ascii=False) + "\n")

    def on_testbed_use_case(self, msg: String) -> None:
        """Arayüzün (user_interface) o an koşturduğu senaryoyla bu düğümün
        LAUNCH ANINDA sabitlenmiş use_case'i uyuşuyor mu. Canlı olarak fizik
        modelini DEĞİŞTİRMEZ (riskli - baseline/buffer sıfırlanır), yalnız
        uyuşmazlığı hem terminal logunda hem olaylar_*.jsonl'de (dolayısıyla
        arayüzün kendi anomali panelinde) görünür kılar. 30.09.2026: bu kontrol
        olsaydı 9 koşunun 8'inde ilk saniyede fark edilirdi, saatler sonra değil."""
        gelen = str(msg.data).strip()
        if gelen == self._last_testbed_use_case:
            return                                   # aynı değer tekrar geldi, sessiz kal
        self._last_testbed_use_case = gelen
        if gelen in ("", "IDLE") or gelen == self.det.use_case:
            return
        self.get_logger().error(
            f"USE_CASE UYUŞMAZLIĞI: bu düğüm '{self.det.use_case}' ile başlatıldı "
            f"ama arayüz şu an '{gelen}' senaryosunu koşturuyor. Kalıntı skoru "
            f"YANLIŞ senaryonun yük/sürtünme düzeltmesiyle hesaplanıyor demektir "
            f"- CTRL+C yapıp 'use_case:={gelen}' ile yeniden başlatın.")
        self._event("use_case_uyusmazligi",
                    baslatilan=self.det.use_case, arayuzdeki=gelen)

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
        # joint_state_broadcaster + AGV köprüsü). Eşleme MESAJ BAŞINA, isim
        # kümesine göre çözülür ve isim kümesi başına önbelleklenir.
        key = tuple(msg.name)
        try:
            i = self.jidx_cache[key]
        except KeyError:
            i = self.jidx_cache[key] = self._resolve(key)
        if i is None:
            self.n_foreign += 1
            return
        if max(i) >= min(len(msg.position), len(msg.velocity), len(msg.effort)):
            self.n_short += 1
            return

        q = np.array([msg.position[k] for k in i], dtype=np.float64)
        qd = np.array([msg.velocity[k] for k in i], dtype=np.float64)
        amps = np.array([msg.effort[k] for k in i], dtype=np.float64)

        self.n_samples += 1
        t0 = time.perf_counter()
        r = self.det.push(q, qd, amps)
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
                    float(r["baseline_n"]), float(r["frozen"]),
                    # Her modelin KENDİ eşiği (indeks 15/16, EK — arayüz artık
                    # bunları hard-code etmek yerine buradan okuyor; bkz.
                    # user_interface/app.py AnomalyCollector).
                    float(r["threshold_residual"]), float(r["threshold_raw"])]
        self.pub_detail.publish(det)

        t_ros = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.f_scores is not None:
            self.f_scores.write(
                f"{t_ros:.4f}," + ",".join(f"{v:.6g}" for v in det.data)
                + f",{int(alarm)}\n")
            now = time.monotonic()
            if now - self.t_flush > 2.0:
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
        """Sağlık raporu - yalnız durum DEĞİŞTİĞİNDE bir satır yazar."""
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
                + (f" | UR dışı {self.n_foreign:,}" if self.n_foreign else "")
                + (f" | kısa mesaj {self.n_short:,}" if self.n_short else "")
                + (f" | alarm {self.n_alarms}" if self.n_alarms else ""))

        durum = "veri_yok" if rate <= 0 else ("iyi" if rate > 400 else "dusuk")
        if durum == self.health_state:
            return
        self.health_state = durum

        if durum == "iyi":
            self.get_logger().info(text)
        elif durum == "dusuk":
            self.get_logger().warn(
                text + "  —  örnek hızı 500 Hz'in altında; q̈ türevi sabit dt "
                "varsayıyor, düşük hızda kalıntı bozulur.")
        else:
            self.get_logger().warn("Veri gelmiyor — /joint_states konusunu kontrol et.")

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
