"""
detector.py
===========
Anomali tespitinin ROS'tan bağımsız çekirdeği: öznitelik motoru + iki ONNX
özkodlayıcı + skor düzeyinde birleşim + eşik.

ROS düğümü (`detector_node.py`) bunun ince bir sarmalayıcısıdır. Ayrı tutulmasının
sebebi test edilebilirlik: `replay_detector.py` gerçek parquet verisini bu SINIFIN
kendisinden geçirir, yani doğrulanan şey düğümün gerçekten çalıştırdığı koddur —
teste özel bir kopyası değil.
"""

from __future__ import annotations

import json
from collections import deque
from pathlib import Path

import numpy as np

from .features import OnlineFeatureExtractor

# Nedensel (geleceği görmeyen) normalleştirme ölçekleri. İkisi de bildirinin
# Denk. 4 yapısını korur; yalnız min–max sınırlarının hangi kümeden alındığı
# değişir. evaluate_fusion.py ikisini yarıştırıp kazananı config'e yazar.
CAUSAL_NORMS = {"theta", "minmax_val"}

RESIDUAL_COLS = [f"r_int_{j}" for j in range(1, 7)] + [f"r_ext_{j}" for j in range(1, 7)]
RAW_COLS = ([f"q_{j}" for j in range(1, 7)] + [f"qd_{j}" for j in range(1, 7)]
            + [f"tau_{j}" for j in range(1, 7)] + ["fx", "fy", "fz", "tx", "ty", "tz"])


class OnnxAE:
    """Tek özkodlayıcı: ONNX oturumu + normalizasyon istatistikleri + eşik."""

    def __init__(self, model_dir: str | Path, expect_cols: list[str],
                 providers: list[str] | None = None, threads: int = 2):
        import onnxruntime as ort

        model_dir = Path(model_dir)
        meta = json.loads((model_dir / "metadata.json").read_text(encoding="utf-8"))
        cols = list(meta["feature_cols"])
        if cols != expect_cols:
            raise ValueError(
                f"{model_dir.name}: kanal sırası beklenenden farklı.\n"
                f"  metadata : {cols}\n  beklenen : {expect_cols}\n"
                f"  Bu sessizce yanlış skor üretir; eğitim ve çıkarım aynı sırayı kullanmalı.")

        if providers is None:
            avail = ort.get_available_providers()
            providers = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider")
                         if p in avail] or ["CPUExecutionProvider"]
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = threads     # 500 Hz döngüsünü aç gözlülükten koru
        self.sess = ort.InferenceSession(str(model_dir / "model.onnx"),
                                         sess_options=opts, providers=providers)
        self.input_name = self.sess.get_inputs()[0].name
        self.mean = np.asarray(meta["mean"], dtype=np.float32)
        self.std = np.asarray(meta["std"], dtype=np.float32)
        self.window = int(meta["window_size"])
        self.n_feat = int(meta["features"])
        self.threshold = float(meta["threshold"])
        self.name = model_dir.name
        self.provider = self.sess.get_providers()[0]

    def score(self, window: np.ndarray) -> float:
        """(T, D) pencere → yeniden yapılanma MSE (eğitimdeki kayıpla aynı tanım)."""
        x = ((window - self.mean) / self.std).astype(np.float32)[None, ...]
        recon = self.sess.run(None, {self.input_name: x})[0]
        return float(np.mean((recon - x) ** 2))


class FusionDetector:
    """
    500 Hz örnek alır, `stride` örnekte bir birleşik anomali skoru üretir.

    Birleşim (245.pdf Denk. 4):  S = w_kal·S̄_kal + w_ham·S̄_ham
    Normalleştirme sınırları akış üzerinden hesaplanamaz (gelecek bilinmez), bu
    yüzden `evaluate_fusion.py`'nin TEMİZ DOĞRULAMA kümesinden donduruduğu
    `fusion_config.json` kullanılır; ölçek orada affin (lo, span) olarak verilir.
    """

    def __init__(self, residual_model_dir, raw_model_dir, fusion_config,
                 current_to_torque, residual_calibration, solver,
                 stride: int = 25, fts_frame: str = "tool",
                 providers: list[str] | None = None,
                 adaptive: bool = True, adaptive_window: int = 600,
                 adaptive_k: float = 8.0, adaptive_warmup: int = 200):
        ctt = json.loads(Path(current_to_torque).read_text(encoding="utf-8"))
        rc = json.loads(Path(residual_calibration).read_text(encoding="utf-8"))
        self.trusted = list(ctt["trusted"])
        self.extractor = OnlineFeatureExtractor(
            solver, ctt["nm_per_amp"], offset_b=rc["b"], fts_frame=fts_frame)

        self.ae_res = OnnxAE(residual_model_dir, RESIDUAL_COLS, providers)
        self.ae_raw = OnnxAE(raw_model_dir, RAW_COLS, providers)
        if self.ae_res.window != self.ae_raw.window:
            raise ValueError("İki modelin pencere boyutu farklı — birleşim yapılamaz.")
        self.window = self.ae_res.window
        self.stride = int(stride)

        fc = json.loads(Path(fusion_config).read_text(encoding="utf-8"))
        self.norm = str(fc.get("norm", ""))
        if self.norm not in CAUSAL_NORMS:
            raise ValueError(
                f"{fusion_config}: 'norm' alanı {sorted(CAUSAL_NORMS)} içinde değil "
                f"(okunan: {self.norm!r}).\n"
                f"  Bildirinin min–max normalleştirmesi sınırlarını arıza enjekte edilmiş\n"
                f"  test kümesinden alır; canlıda o maksimumlar görülmediği için birleşik\n"
                f"  eşik her iki tekil eşikten de katı olur ve tespit çöker.\n"
                f"  evaluate_fusion.py'yi yeniden çalıştırıp güncel config'i üret.")
        self.w_res, self.w_raw = float(fc["w_kal"]), float(fc["w_ham"])
        self.thr_fused = float(fc["fused_threshold"])
        self.expected = fc.get("expected", {})
        # Her nedensel ölçek bir AFFİN dönüşüm:  z = (S − lo)/span.
        #   theta      : lo = 0,   span = θ           → z = 1 "modelin kendi eşiğinde"
        #   minmax_val : lo = min, span = max − min   → bildirinin formülü, sınırlar
        #                TEMİZ DOĞRULAMA pencerelerinden (geleceği görmez)
        # Eski (scale'siz) config'ler theta varsayılarak okunur.
        sc = fc.get("scale") or {"residual": {"lo": 0.0, "span": fc["residual"]["threshold"]},
                                 "raw": {"lo": 0.0, "span": fc["raw"]["threshold"]}}
        self.res_lo, self.res_span = float(sc["residual"]["lo"]), float(sc["residual"]["span"])
        self.raw_lo, self.raw_span = float(sc["raw"]["lo"]), float(sc["raw"]["span"])
        if not (self.res_span > 0 and self.raw_span > 0):
            raise ValueError(f"{fusion_config}: ölçek genişliği (span) pozitif değil.")
        # Tekil modellerin kendi P97 eşikleri — yalnız "hangi model tetikledi"
        # raporlaması için; birleşim kararına girmezler.
        self.res_theta = float(fc["residual"]["threshold"])
        self.raw_theta = float(fc["raw"]["threshold"])

        self.res_buf: deque = deque(maxlen=self.window)
        self.raw_buf: deque = deque(maxlen=self.window)
        self._since = 0

        # ── Uyarlanabilir kural ────────────────────────────────────────────
        # Mutlak eşik θ, doğrulama setinin EN KÖTÜ uyan koşularından gelir. İyi uyan
        # bir koşuda taban çizgisi θ'nın 200 katı altında oturur; oradaki gerçek bir
        # arıza skoru 120 kat yükseltse bile mutlak eşiği geçemez (ölçüldü: gizyazar
        # hatasında birleşik skor 0,003 → 0,462, θ = 0,978).
        #
        # Çözüm: mutlak eşiğin YANINA, son `adaptive_window` kararın kendi dağılımına
        # göre çalışan sağlam bir kural. Alarm = mutlak VEYA uyarlanabilir. Alarm
        # sürerken taban çizgisi dondurulur, yoksa uzun süren bir arıza yeni "normal"
        # olur ve alarm sessizce söner.
        self.adaptive = bool(adaptive)
        self.adaptive_k = float(adaptive_k)
        self.adaptive_warmup = int(adaptive_warmup)
        self._hist: deque = deque(maxlen=int(adaptive_window))

    @property
    def adaptive_window(self) -> int:
        """Uyarlanabilir taban çizgisinin kaç karar geriye baktığı."""
        return self._hist.maxlen or 0

    @property
    def lag_seconds(self) -> float:
        """Tespit gecikmesinin deterministik kısmı: SG gecikmesi + karar periyodu."""
        return self.extractor.lag_seconds + self.stride * self.extractor.dt

    def reset(self) -> None:
        """Kesintiden sonra temiz başlangıç. Uyarlanabilir taban çizgisi de silinir:
        başka bir koşunun skor dağılımı bu koşu için geçerli değildir."""
        self.extractor.reset()
        self.res_buf.clear()
        self.raw_buf.clear()
        self._hist.clear()
        self._since = 0

    def push(self, q, qd, effort_amps, wrench) -> dict | None:
        """Bir örnek ekler. Karar anı gelmediyse None, geldiyse skor sözlüğü."""
        f = self.extractor.push(q, qd, effort_amps, wrench)
        if f is None:
            return None
        self.res_buf.append(f["residual_vec"])
        self.raw_buf.append(f["raw_vec"])
        self._since += 1
        if len(self.res_buf) < self.window or self._since < self.stride:
            return None
        self._since = 0

        s_res = self.ae_res.score(np.stack(self.res_buf))
        s_raw = self.ae_raw.score(np.stack(self.raw_buf))
        z_res = (s_res - self.res_lo) / self.res_span
        z_raw = (s_raw - self.raw_lo) / self.raw_span
        fused = self.w_res * z_res + self.w_raw * z_raw

        hit_abs = bool(fused > self.thr_fused)
        thr_ad = float("inf")
        hit_ad = False
        if self.adaptive and len(self._hist) >= self.adaptive_warmup:
            h = np.fromiter(self._hist, dtype=np.float64)
            med = float(np.median(h))
            mad = float(np.median(np.abs(h - med)))
            # 1.4826·MAD ≈ σ (Gauss'ta). Taban çizgisi neredeyse sabitse MAD sıfıra
            # gider ve kural aşırı hassaslaşır; medyanın küçük bir oranıyla tabanlanıyor.
            scale = max(1.4826 * mad, 0.05 * med, 1e-9)
            thr_ad = med + self.adaptive_k * scale
            hit_ad = bool(fused > thr_ad)

        detected = hit_abs or hit_ad
        if not detected:                      # alarm sürerken taban çizgisi donar
            self._hist.append(fused)

        return {
            "s_residual": s_res, "s_raw": s_raw,
            "z_residual": z_res, "z_raw": z_raw,
            "fused": fused, "threshold": self.thr_fused,
            "adaptive_threshold": thr_ad,
            "detected": detected,
            "hit_absolute": hit_abs, "hit_adaptive": hit_ad,
            "hit_residual": bool(s_res > self.ae_res.threshold),
            "hit_raw": bool(s_raw > self.ae_raw.threshold),
        }
