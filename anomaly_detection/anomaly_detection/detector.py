"""
detector.py
===========
Anomali tespitinin ROS'tan bağımsız çekirdeği: KTS'siz öznitelik motoru + iki
ONNX özkodlayıcı + skor düzeyinde birleşim + rejim-koşullu eşik + uyarlanabilir
kural. ROS düğümü (`detector_node.py`) bunun ince bir sarmalayıcısıdır.

v3 -> v4 (bkz. features.py docstring'i, KOKEN madde madde):
  - KTS çıkarıldı: kalıntı 12→6 kanal (yalnız r_tot), ham 24→16 kanal (wrench yok,
    q_1/q_6 yok).
  - Birleşim normalizasyonu affine (S−lo)/span YERİNE LOG: z = (log10(S+eps)−μ)/σ,
    μ/σ VAL kümesinin log-skorlarından. Sebep: skor dağılımı çok ağır kuyruklu
    (bkz. `dataset_prep/analyze_errors.py` çıktıları); log bunu normale yaklaştırıp
    birleşimi tek bir aşırı uç pencereye karşı sağlamlaştırıyor.
  - Birleşim ağırlığı (w_kal) artık ÖLÇÜLMÜŞ (evaluate_faults.py, fiziksel-tutarlı
    arıza enjeksiyonu ile), önsel/bildiriden alınmış değil.
  - Eşik REJİM başına 4 persentil taşıyor (p97/p99/p99.9/p99.99) — hangisinin
    kullanılacağı `quantile` parametresiyle seçilir, sabit tek sayı değil.
  - Kalıntı hesabı artık SENARYO'ya (use_case) bağlı — bu yeni, ZORUNLU bir girdi.
"""

from __future__ import annotations

import json
from collections import deque
from pathlib import Path

import numpy as np

from .features import OnlineFeatureExtractor

RESIDUAL_COLS = [f"r_tot_{j}" for j in range(1, 7)]
RAW_COLS = ([f"q_{j}" for j in (2, 3, 4, 5)] + [f"qd_{j}" for j in range(1, 7)]
            + [f"tau_{j}" for j in range(1, 7)])
QUANTILES = ("p97", "p99", "p99.9", "p99.99")


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
        self.meta = meta
        self.provider = self.sess.get_providers()[0]

    def score(self, window: np.ndarray) -> float:
        """(T, D) pencere → yeniden yapılanma MSE (eğitimdeki kayıpla aynı tanım)."""
        x = ((window - self.mean) / self.std).astype(np.float32)[None, ...]
        recon = self.sess.run(None, {self.input_name: x})[0]
        return float(np.mean((recon - x) ** 2))


class FusionDetector:
    """
    500 Hz örnek alır, `stride` örnekte bir birleşik anomali skoru üretir.

    Birleşim: S_bir = w_kal·z_kal + w_ham·z_ham, z = (log10(S+eps) − μ_val) / σ_val.
    μ/σ ve w, `fusion_config.json`'dan (VAL kümesinden ölçülmüş, bkz. dosyanın
    kendi `PROVISIONAL` uyarısı — eşikler gerçek robotta yeniden kalibre edilmeli).
    """

    def __init__(self, residual_model_dir, raw_model_dir, fusion_config,
                 current_to_torque, residual_calibration, use_case: str, solver,
                 stride: int = 25, providers: list[str] | None = None,
                 quantile: str = "p99.9",
                 regime_threshold: bool | None = None,
                 adaptive: bool = False, adaptive_window: int = 600,
                 adaptive_k: float = 8.0, adaptive_warmup: int = 200,
                 freeze_timeout: float = 3.0, motion_qd_min: float | None = None):
        ctt = json.loads(Path(current_to_torque).read_text(encoding="utf-8"))
        cal = json.loads(Path(residual_calibration).read_text(encoding="utf-8"))
        self.trusted = list(ctt["trusted"])
        self.use_case = use_case
        self.extractor = OnlineFeatureExtractor(
            solver, ctt["nm_per_amp"], cal, use_case)

        self.ae_res = OnnxAE(residual_model_dir, RESIDUAL_COLS, providers)
        self.ae_raw = OnnxAE(raw_model_dir, RAW_COLS, providers)
        if self.ae_res.window != self.ae_raw.window:
            raise ValueError("İki modelin pencere boyutu farklı — birleşim yapılamaz.")
        self.window = self.ae_res.window
        self.stride = int(stride)

        fc = json.loads(Path(fusion_config).read_text(encoding="utf-8"))
        if fc.get("PROVISIONAL"):
            import warnings
            warnings.warn(
                f"{fusion_config}: PROVISIONAL eşikler kullanılıyor - "
                f"{fc.get('provisional_note', '')}", stacklevel=2)
        self.norm = str(fc.get("norm", ""))
        if self.norm != "log":
            raise ValueError(f"{fusion_config}: yalnız 'log' normalizasyonu destekleniyor "
                             f"(okunan: {self.norm!r}).")
        ln = fc["log_norm"]
        self.log_eps = float(ln["eps"])
        self.log_res = (float(ln["residual"]["mean"]), float(ln["residual"]["std"]))
        self.log_raw = (float(ln["raw"]["mean"]), float(ln["raw"]["std"]))
        self.w_res, self.w_raw = float(fc["w_kal"]), float(fc["w_ham"])

        if quantile not in QUANTILES:
            raise ValueError(f"quantile {quantile!r} değil; geçerli: {QUANTILES}")
        self.quantile = quantile
        # use_case'e özel eşik varsa onu kullan (gerçek hücreden ölçülmüş olabilir,
        # bkz. HRC girdisi: temas pozundaki beklenen reaksiyon torku jenerik eşikte
        # yok); yoksa jenerik (offline, PROVISIONAL) threshold_by_regime'e düş.
        regime_tbl = fc.get("threshold_by_regime_by_use_case", {}).get(
            use_case, fc["threshold_by_regime"])
        self.thr_regime = {r: float(regime_tbl[r][quantile]) for r in ("static", "moving")}
        self.thr_regime_source = "use_case" if use_case in fc.get(
            "threshold_by_regime_by_use_case", {}) else "generic"
        # 01.10.2026: ikinci, DAHA DÜŞÜK bir "uyarı" eşiği - her zaman p99.9,
        # `quantile` parametresinden (duruş eşiği için p99.99 olabilir) BAĞIMSIZ.
        # UI'da sarı pop-up bunu kullanır; robotu durdurmaz, yalnız operatöre
        # haber verir. `quantile` yanlışlıkla "p99.9" seçilirse iki eşik aynı
        # sayıya denk gelir - bu durumda uyarı==duruş, pop-up hiç görünmeden
        # direkt dururuz (bilinçli davranış, hata değil).
        self.thr_warn_regime = {r: float(regime_tbl[r]["p99.9"]) for r in ("static", "moving")}
        self.regime_threshold = bool(fc.get("regime_threshold", True) if regime_threshold is None
                                     else regime_threshold)
        self.motion_qd_min = float(motion_qd_min if motion_qd_min is not None
                                   else fc.get("motion_qd_min", 0.02))
        self.res_theta = float(fc["residual"]["threshold"])
        self.raw_theta = float(fc["raw"]["threshold"])

        self.res_buf: deque = deque(maxlen=self.window)
        self.raw_buf: deque = deque(maxlen=self.window)
        self._since = 0

        # ── Uyarlanabilir kural (v3'ten aynen taşındı - normalizasyon değişse de
        # medyan/MAD tabanlı kural ölçekten bağımsız çalışır). Gerçek robotta
        # (21.08.2026) kalıntının POZA bağlı olduğu ölçüldüğü için varsayılan KAPALI;
        # v4'ün senaryo-başına yük düzeltmesi bu bağımlılığı azaltmış olabilir ama
        # HENÜZ gerçek robotta doğrulanmadı - Pazartesi testinde değerlendirilecek.
        self.adaptive = bool(adaptive)
        self.adaptive_k = float(adaptive_k)
        self.adaptive_warmup = int(adaptive_warmup)
        self.freeze_timeout = float(freeze_timeout)
        self._hist: deque = deque(maxlen=int(adaptive_window))
        self._alarm_run = 0
        self._qd_peak = 0.0

    @property
    def adaptive_window(self) -> int:
        return self._hist.maxlen or 0

    @property
    def decision_period(self) -> float:
        return self.stride * self.extractor.dt

    @property
    def lag_seconds(self) -> float:
        return self.extractor.lag_seconds + self.stride * self.extractor.dt

    def reset(self) -> None:
        self.extractor.reset()
        self.res_buf.clear()
        self.raw_buf.clear()
        self._hist.clear()
        self._since = 0
        self._alarm_run = 0
        self._qd_peak = 0.0

    def _z(self, score: float, log_mu_sigma: tuple[float, float]) -> float:
        mu, sigma = log_mu_sigma
        return (np.log10(score + self.log_eps) - mu) / sigma

    def push(self, q, qd, effort_amps) -> dict | None:
        """Bir örnek ekler. Karar anı gelmediyse None, geldiyse skor sözlüğü."""
        self._qd_peak = max(self._qd_peak, float(np.max(np.abs(qd))))
        f = self.extractor.push(q, qd, effort_amps)
        if f is None:
            return None
        self.res_buf.append(f["residual_vec"])
        self.raw_buf.append(f["raw_vec"])
        self._since += 1
        if len(self.res_buf) < self.window or self._since < self.stride:
            return None
        self._since = 0
        qd_peak, self._qd_peak = self._qd_peak, 0.0
        moving = qd_peak > self.motion_qd_min

        s_res = self.ae_res.score(np.stack(self.res_buf))
        s_raw = self.ae_raw.score(np.stack(self.raw_buf))
        z_res = self._z(s_res, self.log_res)
        z_raw = self._z(s_raw, self.log_raw)
        fused = self.w_res * z_res + self.w_raw * z_raw

        thr_abs = self.thr_regime["moving"] if (self.regime_threshold and moving) else (
            self.thr_regime["static"] if self.regime_threshold else self.thr_regime["moving"])
        hit_abs = bool(fused > thr_abs)
        # Uyarı katmanı (p99.9) - duruş katmanından (quantile, p99.99) bağımsız,
        # her zaman hesaplanır. Yalnız bilgilendirme; robotu BU değer durdurmaz.
        thr_warn = self.thr_warn_regime["moving"] if (self.regime_threshold and moving) else (
            self.thr_warn_regime["static"] if self.regime_threshold else self.thr_warn_regime["moving"])
        hit_warn = bool(fused > thr_warn)
        thr_ad = float("inf")
        hit_ad = False
        if self.adaptive and len(self._hist) >= self.adaptive_warmup:
            h = np.fromiter(self._hist, dtype=np.float64)
            med = float(np.median(h))
            mad = float(np.median(np.abs(h - med)))
            scale = max(1.4826 * mad, 0.05 * abs(med), 1e-9)
            thr_ad = med + self.adaptive_k * scale
            hit_ad = bool(fused > thr_ad)

        detected = hit_abs or hit_ad
        self._alarm_run = self._alarm_run + 1 if detected else 0
        frozen = detected and self._alarm_run * self.decision_period <= self.freeze_timeout
        if moving and not frozen:
            self._hist.append(fused)

        return {
            "moving": moving, "qd_peak": qd_peak, "frozen": frozen,
            "baseline_n": len(self._hist),
            "s_residual": s_res, "s_raw": s_raw,
            "z_residual": z_res, "z_raw": z_raw,
            "fused": fused, "threshold": thr_abs, "quantile": self.quantile,
            "threshold_regime": ("moving" if moving else "static") if self.regime_threshold else "global",
            "adaptive_threshold": thr_ad,
            "detected": detected,
            "hit_absolute": hit_abs, "hit_adaptive": hit_ad,
            "hit_residual": bool(s_res > self.ae_res.threshold),
            "hit_raw": bool(s_raw > self.ae_raw.threshold),
            # Her modelin KENDİ eşiği (metadata.json'dan, P97, quantile
            # parametresinden bağımsız). Arayüz bunları hard-code etmemeli —
            # bir sonraki yeniden eğitimde sessizce bayatlar (bkz. detector_node.py
            # ~/detail dokümantasyonu, THR_FUSED'in daha önce başına geleni).
            "threshold_residual": self.ae_res.threshold,
            "threshold_raw": self.ae_raw.threshold,
            "threshold_warn": thr_warn, "hit_warn": hit_warn,
        }
