#!/usr/bin/env python3
"""
evaluate_ae.py  (v2)
====================
Eğitilmiş modeli doğrulama böleninde uçtan uca değerlendirir.

Üretilenler
-----------
  evaluation.json      tüm sayısal sonuçlar (AUC, F1, P, R, gecikme, CUSUM, sınıflandırma)
  evaluation.png       ROC · PR · skor dağılımı
  timeline.png         anomali skoru zaman çizelgesi + tespit edilen bölgeler + CUSUM
  clustering.png       kümelenme süresi dağılımı + en büyük kümeler
  classification.png   internal / external / mixed dağılımı (yalnız hybrid)
  eval_scores.npz      pencere skorları, etiketler, kanal hataları, CUSUM izi

Sentetik etiketleme (her modun kendi çalışmasındaki gibi)
  raw    : pencere-içi FTS varyansı > P90  VEYA  eklem hızı varyansı > P90
  hybrid : pencere-içi r_ext varyansı > P90 VEYA r_int varyansı > P90

Kullanım
--------
    python evaluate_ae.py --mode raw    --model-dir raw_ae_model
    python evaluate_ae.py --mode hybrid --model-dir hybrid_ae_model
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
import torch
from torch.utils.data import DataLoader
from sklearn.metrics import (roc_auc_score, roc_curve, precision_recall_curve,
                             average_precision_score, precision_recall_fscore_support)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from models import PRESETS, WindowDataset, build  # noqa: E402

LABEL_GROUPS = {
    "raw": {"FTS": ["fx", "fy", "fz", "tx", "ty", "tz"],
            "eklem hızı": [f"qd_{j}" for j in range(1, 7)]},
    "hybrid": {"r_ext": [f"r_ext_{j}" for j in range(1, 7)],
               "r_int": [f"r_int_{j}" for j in range(1, 7)]},
}


# ───────────────────────── yardımcılar ─────────────────────────

def cusum(e: np.ndarray, mu_ref: float, delta: float, h: float):
    """Tek taraflı CUSUM. S_t = max(0, S_{t-1} + e_t - mu_ref - delta); alarm S_t > h."""
    S = np.zeros(len(e))
    s = 0.0
    for i, x in enumerate(e):
        s = max(0.0, s + x - mu_ref - delta)
        S[i] = s
    return S, S > h


def cluster(mask: np.ndarray, max_gap: int = 2):
    """Ardışık True bölgelerini, aralarında <=max_gap boşluk varsa birleştirerek kümele."""
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    out, start, prev = [], idx[0], idx[0]
    for i in idx[1:]:
        if i - prev > max_gap:
            out.append((start, prev))
            start = i
        prev = i
    out.append((start, prev))
    return out


def bench_latency(model, meta, dev, batches=(1, 8, 64), iters=200, warmup=30):
    """Pencere başına çıkarım gecikmesi. 500 Hz gereksinimiyle karşılaştırmak için."""
    res = {}
    model.eval()
    for B in batches:
        x = torch.randn(B, meta["window_size"], meta["features"], device=dev)
        with torch.no_grad():
            for _ in range(warmup):
                model(x)
            if dev.type == "cuda":
                torch.cuda.synchronize()
            ts = []
            for _ in range(iters):
                t0 = time.perf_counter()
                model(x)
                if dev.type == "cuda":
                    torch.cuda.synchronize()
                ts.append((time.perf_counter() - t0) * 1000.0)
        ts = np.array(ts)
        res[f"batch{B}"] = {
            "ms_per_call_p50": round(float(np.percentile(ts, 50)), 3),
            "ms_per_call_p95": round(float(np.percentile(ts, 95)), 3),
            "ms_per_call_p99": round(float(np.percentile(ts, 99)), 3),
            "ms_per_window_p50": round(float(np.percentile(ts, 50)) / B, 4),
            "windows_per_sec": round(1000.0 * B / float(np.percentile(ts, 50)), 1),
        }
    return res


# ───────────────────────────── ana ─────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", required=True, choices=["raw", "hybrid"])
    ap.add_argument("--model-dir", default=None)
    ap.add_argument("--parquet", default=None)
    ap.add_argument("--device", default=None)
    ap.add_argument("--label-percentile", type=float, default=90.0)
    ap.add_argument("--batch-size", type=int, default=512)
    ap.add_argument("--cusum-delta", type=float, default=0.5, help="σ cinsinden tolerans")
    ap.add_argument("--cusum-h", type=float, default=5.0, help="σ cinsinden alarm eşiği")
    ap.add_argument("--cluster-gap", type=int, default=2, help="pencere cinsinden birleştirme boşluğu")
    ap.add_argument("--no-latency", action="store_true")
    args = ap.parse_args()

    preset = PRESETS[args.mode]
    model_dir = Path(args.model_dir or preset["default_model_dir"])
    meta = json.loads((model_dir / "metadata.json").read_text(encoding="utf-8"))
    parquet = args.parquet or meta.get("parquet") or preset["default_parquet"]
    dev = torch.device(args.device) if args.device else torch.device(
        "cuda" if torch.cuda.is_available() else "cpu")

    cols = list(meta["feature_cols"])
    W, S = meta["window_size"], meta["stride"]
    mean = np.array(meta["mean"], dtype=np.float32)
    std = np.array(meta["std"], dtype=np.float32)

    print("=" * 70)
    print(f"DEĞERLENDİRME — {args.mode.upper()}   ({model_dir})")
    print("=" * 70)
    print(f"  parquet: {parquet}")
    print(f"  cihaz  : {dev}" + (f" ({torch.cuda.get_device_name(0)})" if dev.type == "cuda" else ""))

    # ── veri ──
    want = cols + (["t"] if "t" not in cols else [])
    try:
        df = pd.read_parquet(parquet, columns=want)
        tcol = df["t"].to_numpy(dtype=np.float64)
    except Exception:
        df = pd.read_parquet(parquet, columns=cols)
        tcol = None
    data = df[cols].to_numpy(dtype=np.float32)
    ok = np.isfinite(data).all(axis=1)
    data, tcol = data[ok], (tcol[ok] if tcol is not None else None)
    n = len(data)
    split = int(n * meta.get("train_ratio", 0.8))
    val_raw = data[split:]
    t_val = tcol[split:] if tcol is not None else np.arange(len(val_raw)) * 0.002
    val_d = (val_raw - mean) / std
    ds = WindowDataset(val_d, W, S)
    starts = np.arange(len(ds)) * S
    t_win = t_val[np.minimum(starts + W - 1, len(t_val) - 1)]
    t_win = t_win - t_win[0]
    print(f"  doğrulama: {n-split:,} örnek → {len(ds):,} pencere (W={W}, stride={S})")

    # ── skorlar ──
    model = build(meta).to(dev)
    model.load_state_dict(torch.load(model_dir / "best_model.pt", map_location=dev,
                                     weights_only=True))
    model.eval()
    ld = DataLoader(ds, batch_size=args.batch_size, shuffle=False)
    sc, ce = [], []
    with torch.no_grad():
        for b in ld:
            b = b.to(dev)
            se = (model(b) - b) ** 2
            sc.append(se.mean(dim=(1, 2)).float().cpu().numpy())
            ce.append(se.mean(dim=1).float().cpu().numpy())
    scores = np.concatenate(sc)
    chan_err = np.concatenate(ce)

    # ── sentetik etiketler ──
    groups = LABEL_GROUPS[args.mode]
    lab, gvar = {}, {}
    for g, cs in groups.items():
        ii = [cols.index(c) for c in cs]
        sub = val_raw[:, ii]
        v = np.array([sub[s:s + W].var(axis=0).mean() for s in starts])
        thr = np.percentile(v, args.label_percentile)
        lab[g] = v > thr
        gvar[g] = v
        print(f"  etiket {g:12s}: varyans P{args.label_percentile:g}={thr:.5f} → "
              f"{lab[g].sum():,} pencere ({100*lab[g].mean():.1f}%)")
    y = np.zeros(len(ds), dtype=bool)
    for m in lab.values():
        y |= m
    print(f"  {'BİRLEŞİK':19s}: {y.sum():,} anomali penceresi ({100*y.mean():.1f}%)")
    if y.all() or not y.any():
        print("HATA: etiketler tek sınıf.", file=sys.stderr)
        return 2

    # ── metrikler ──
    auc = float(roc_auc_score(y, scores))
    apr = float(average_precision_score(y, scores))
    prec, rec, pr_thr = precision_recall_curve(y, scores)
    f1c = 2 * prec * rec / (prec + rec + 1e-12)
    bi = int(np.nanargmax(f1c))
    best_thr = float(pr_thr[min(bi, len(pr_thr) - 1)])
    tthr = float(meta["threshold"])
    det = scores > tthr
    p_t, r_t, f_t, _ = precision_recall_fscore_support(y, det, average="binary", zero_division=0)

    print(f"\n  {'AUC (ROC)':30s}: {auc:.4f}")
    print(f"  {'Average Precision':30s}: {apr:.4f}")
    print(f"  {'En iyi F1 (optimal eşik)':30s}: {f1c[bi]:.4f}  (P={prec[bi]:.3f} R={rec[bi]:.3f} eşik={best_thr:.5f})")
    print(f"  {'Eğitilmiş eşikte F1':30s}: {f_t:.4f}  (P={p_t:.3f} R={r_t:.3f} eşik={tthr:.5f})")
    print(f"  {'Alarm oranı (eğit. eşik)':30s}: {100*det.mean():.2f}%")

    out = {
        "mode": args.mode, "model_dir": str(model_dir), "parquet": str(parquet),
        "device": str(dev), "n_val_windows": int(len(ds)),
        "window_size": W, "stride": S, "features": meta["features"],
        "hidden_dim": meta["hidden_dim"], "total_params": meta.get("total_params"),
        "best_val_loss": meta.get("best_val_loss"), "epochs_trained": meta.get("epochs_trained"),
        "anomaly_ratio": float(y.mean()),
        "auc": auc, "average_precision": apr,
        "best_f1": float(f1c[bi]), "best_f1_precision": float(prec[bi]),
        "best_f1_recall": float(rec[bi]), "best_f1_threshold": best_thr,
        "trained_threshold": tthr, "f1_at_trained_threshold": float(f_t),
        "precision_at_trained_threshold": float(p_t),
        "recall_at_trained_threshold": float(r_t),
        "alarm_rate_at_trained_threshold": float(det.mean()),
    }

    # ── CUSUM (yavaş sürüklenme katmanı) ──
    mu = float(scores.mean()); sg = float(scores.std())
    Sc, cu_alarm = cusum(scores, mu, args.cusum_delta * sg, args.cusum_h * sg)
    comb = det | cu_alarm
    p_c, r_c, f_c, _ = precision_recall_fscore_support(y, comb, average="binary", zero_division=0)
    extra = int((cu_alarm & ~det).sum())
    print(f"\n  CUSUM (δ={args.cusum_delta}σ, h={args.cusum_h}σ):")
    print(f"    {'CUSUM alarmı':28s}: {cu_alarm.sum():,} pencere  (AE'nin kaçırdığı +{extra:,})")
    print(f"    {'AE + CUSUM birleşik F1':28s}: {f_c:.4f}  (P={p_c:.3f} R={r_c:.3f})")
    out["cusum"] = {"delta_sigma": args.cusum_delta, "h_sigma": args.cusum_h,
                    "mu_ref": mu, "sigma": sg,
                    "n_alarm": int(cu_alarm.sum()), "n_extra_over_ae": extra,
                    "f1_combined": float(f_c), "precision_combined": float(p_c),
                    "recall_combined": float(r_c)}

    # ── kümeleme ──
    cl = cluster(det, max_gap=args.cluster_gap)
    dt_win = float(np.median(np.diff(t_win))) if len(t_win) > 1 else S * 0.002
    durs = np.array([(b - a + 1) * dt_win for a, b in cl]) if cl else np.array([])
    print(f"\n  Tespit kümesi: {len(cl):,}   "
          f"ortalama süre {1000*durs.mean() if len(durs) else 0:.0f} ms   "
          f"en uzun {1000*durs.max() if len(durs) else 0:.0f} ms")
    out["clusters"] = {"n": len(cl), "gap_windows": args.cluster_gap,
                       "mean_duration_ms": float(1000 * durs.mean()) if len(durs) else 0.0,
                       "max_duration_ms": float(1000 * durs.max()) if len(durs) else 0.0}

    # ── anomali sınıflandırma ──
    labels_cls = None
    if args.mode == "hybrid":
        ii_int = [cols.index(f"r_int_{j}") for j in range(1, 7)]
        ii_ext = [cols.index(f"r_ext_{j}") for j in range(1, 7)]
        nint = np.array([np.linalg.norm(val_raw[s:s + W, ii_int], axis=1).mean() for s in starts])
        next_ = np.array([np.linalg.norm(val_raw[s:s + W, ii_ext], axis=1).mean() for s in starts])
        labels_cls = np.full(len(ds), "normal", dtype=object)
        a = det & (nint > 2 * next_)
        b = det & (next_ > 2 * nint)
        labels_cls[det] = "mixed"
        labels_cls[a] = "internal"
        labels_cls[b] = "external"
        cnt = {k: int((labels_cls == k).sum()) for k in ("internal", "external", "mixed")}
        print(f"\n  Anomali sınıflandırma (‖r_int‖ vs 2‖r_ext‖):")
        for k, v in cnt.items():
            print(f"    {k:12s}: {v:,} ({100*v/max(det.sum(),1):.1f}% tespitlerin)")
        out["classification"] = cnt
    else:
        # raw modda ayrıştırma yok → kanal-grubu baskınlığı ile vekil sınıflandırma
        gi = {g: [cols.index(c) for c in cs] for g, cs in groups.items()}
        ga = chan_err[:, gi["FTS"]].mean(1)
        gb = chan_err[:, gi["eklem hızı"]].mean(1)
        labels_cls = np.full(len(ds), "normal", dtype=object)
        labels_cls[det] = "mixed"
        labels_cls[det & (ga > 2 * gb)] = "FTS-baskin"
        labels_cls[det & (gb > 2 * ga)] = "hiz-baskin"
        cnt = {k: int((labels_cls == k).sum()) for k in ("FTS-baskin", "hiz-baskin", "mixed")}
        print(f"\n  Vekil sınıflandırma (kanal grubu baskınlığı):")
        for k, v in cnt.items():
            print(f"    {k:12s}: {v:,} ({100*v/max(det.sum(),1):.1f}%)")
        out["classification"] = cnt

    # ── kanal katkıları ──
    top = chan_err[scores > np.percentile(scores, 99)].mean(axis=0)
    order = np.argsort(top)[::-1][:8]
    print(f"\n  En yüksek skorlu %1 penceresinde kanal katkıları:")
    for i in order:
        print(f"    {cols[i]:12s} {top[i]:.5f}")
    out["channel_contrib_top1pct"] = {cols[i]: float(top[i]) for i in order}

    # ── gecikme ──
    if not args.no_latency:
        print(f"\n  Çıkarım gecikmesi ({dev}):")
        lat = bench_latency(model, meta, dev)
        for k, v in lat.items():
            print(f"    {k:8s}: p50 {v['ms_per_call_p50']:7.3f} ms/çağrı  "
                  f"{v['ms_per_window_p50']:7.4f} ms/pencere  "
                  f"{v['windows_per_sec']:9.1f} pencere/s")
        need = 500.0 / S     # 500 Hz akışta stride S ile saniyede gereken pencere
        head = lat["batch1"]["windows_per_sec"] / need
        print(f"    500 Hz akışta stride={S} → {need:.0f} pencere/s gerekli; "
              f"batch=1 kapasitesi {head:.1f}× " + ("✅" if head >= 2 else "⚠ dar"))
        out["latency"] = lat
        out["required_windows_per_sec_at_500hz"] = need
        out["headroom_batch1"] = round(float(head), 2)

    (model_dir / "evaluation.json").write_text(json.dumps(out, indent=2, ensure_ascii=False),
                                               encoding="utf-8")
    np.savez_compressed(model_dir / "eval_scores.npz", scores=scores, labels=y,
                        channel_err=chan_err.astype(np.float32), cusum=Sc.astype(np.float32),
                        t_win=t_win, cls=labels_cls.astype(str))

    # ── figürler ──
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fpr, tpr, _ = roc_curve(y, scores)
        fig, ax = plt.subplots(1, 3, figsize=(16, 4.4))
        ax[0].plot(fpr, tpr, lw=2); ax[0].plot([0, 1], [0, 1], "k--", lw=.8)
        ax[0].set_xlabel("FPR"); ax[0].set_ylabel("TPR")
        ax[0].set_title(f"ROC — AUC = {auc:.4f}"); ax[0].grid(alpha=.3)
        ax[1].plot(rec, prec, lw=2)
        ax[1].scatter([rec[bi]], [prec[bi]], c="r", zorder=5, label=f"en iyi F1={f1c[bi]:.3f}")
        ax[1].set_xlabel("Recall"); ax[1].set_ylabel("Precision")
        ax[1].set_title(f"PR — AP = {apr:.4f}"); ax[1].legend(); ax[1].grid(alpha=.3)
        lo, hi = np.percentile(scores, [0.1, 99.9])
        bins = np.linspace(lo, hi, 80)
        ax[2].hist(scores[~y], bins=bins, alpha=.6, label="normal", density=True)
        ax[2].hist(scores[y], bins=bins, alpha=.6, label="anomali", density=True)
        ax[2].axvline(tthr, color="r", ls="--", label=f"eşik {tthr:.3f}")
        ax[2].set_xlabel("rekonstrüksiyon hatası"); ax[2].set_yscale("log")
        ax[2].set_title("Skor dağılımı"); ax[2].legend(); ax[2].grid(alpha=.3)
        fig.suptitle(f"{args.mode.upper()} — doğrulama böleni", fontsize=12)
        fig.tight_layout(); fig.savefig(model_dir / "evaluation.png", dpi=130); plt.close(fig)

        fig, ax = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
        ax[0].plot(t_win, scores, lw=.4, color="steelblue")
        ax[0].axhline(tthr, color="r", ls="--", lw=1.2, label=f"eşik {tthr:.3f}")
        if cl:
            for a_, b_ in cl[:400]:
                ax[0].axvspan(t_win[a_], t_win[b_], color="red", alpha=.18, lw=0)
        ax[0].set_ylabel("anomali skoru"); ax[0].legend(); ax[0].grid(alpha=.3)
        ax[0].set_title(f"{args.mode.upper()} — anomali zaman çizelgesi ({len(cl)} küme)")
        ax[1].plot(t_win, Sc, lw=.6, color="darkorange")
        ax[1].axhline(args.cusum_h * sg, color="r", ls="--", lw=1.2,
                      label=f"CUSUM h = {args.cusum_h}σ")
        ax[1].set_ylabel("CUSUM S(t)"); ax[1].set_xlabel("t [s]")
        ax[1].legend(); ax[1].grid(alpha=.3)
        fig.tight_layout(); fig.savefig(model_dir / "timeline.png", dpi=130); plt.close(fig)

        if len(durs):
            fig, ax = plt.subplots(1, 2, figsize=(12, 4))
            ax[0].hist(1000 * durs, bins=40, color="teal")
            ax[0].set_xlabel("küme süresi [ms]"); ax[0].set_ylabel("adet")
            ax[0].set_yscale("log"); ax[0].set_title(f"Küme süresi dağılımı (n={len(cl)})")
            ax[0].grid(alpha=.3)
            k = np.argsort(durs)[::-1][:5]
            ax[1].barh([f"#{i+1}" for i in range(len(k))][::-1], (1000 * durs[k])[::-1],
                       color="indianred")
            ax[1].set_xlabel("süre [ms]"); ax[1].set_title("En uzun 5 küme"); ax[1].grid(alpha=.3)
            fig.tight_layout(); fig.savefig(model_dir / "clustering.png", dpi=130); plt.close(fig)

        keys = [k for k in out["classification"] if out["classification"][k] > 0]
        if keys:
            fig, ax = plt.subplots(figsize=(6, 4))
            ax.bar(keys, [out["classification"][k] for k in keys],
                   color=["indianred", "steelblue", "mediumpurple"][:len(keys)])
            ax.set_ylabel("pencere"); ax.set_title(f"{args.mode.upper()} — anomali tipi dağılımı")
            ax.grid(alpha=.3, axis="y")
            fig.tight_layout(); fig.savefig(model_dir / "classification.png", dpi=130); plt.close(fig)

        print(f"\n→ {model_dir}/evaluation.png · timeline.png · clustering.png · classification.png")
    except Exception as e:
        print(f"  (grafik atlandı: {e})")

    print(f"→ {model_dir/'evaluation.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
