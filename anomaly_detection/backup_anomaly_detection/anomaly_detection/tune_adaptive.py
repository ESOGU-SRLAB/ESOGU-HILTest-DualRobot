#!/usr/bin/env python3
"""
tune_adaptive.py
================
Uyarlanabilir alarm kuralının `k` katsayısını tarayarak seçer.

Mutlak eşik θ, doğrulama setinin en kötü uyan koşularından gelir; iyi uyan bir
koşuda taban çizgisi θ'nın çok altında oturur ve oradaki gerçek bir arıza mutlak
eşiği hiç geçemez (ölçüldü: gizyazar hatasında birleşik skor 0,003 → 0,462 iken
θ = 0,978). Uyarlanabilir kural bunu kapatır:

    alarm  =  birleşik > θ_mutlak   VEYA   birleşik > medyan + k·1,4826·MAD

Bu betik skor izlerini BİR KEZ hesaplar, sonra `k`'yı ücretsiz süpürür: temiz
veride yanlış alarm oranına ve dört senaryodaki tespit oranı/gecikmesine bakar.

Kullanım
--------
    python3 tune_adaptive.py
    python3 tune_adaptive.py --runs 4 --max-len 8000
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import deque
from pathlib import Path

import numpy as np
import pandas as pd

PKG = Path("/home/cem/colcon_ws/src/anomaly_detection")   # paket 20.08.2026da yeniden adlandirildi
BASE = Path("/home/cem/colcon_ws/src/anomaly_detection")

FAULTS = {
    "yok":            {"tr": "Arıza yok (temiz)", "kind": None},
    "motor_kaymasi":  {"tr": "Motor kayması", "kind": "ramp", "joint": 2, "amp_nm": 15.0},
    "carpisma":       {"tr": "Çarpışma", "kind": "wrench_pulse", "amp_n": 30.0},
    "gizyazar":       {"tr": "Gizyazar hatası", "kind": "q_step", "joint": 4, "amp_rad": 1.5},
    "sensor_gurultu": {"tr": "Sensör gürültüsü", "kind": "wrench_noise", "amp_n": 3.5},
}


def apply_rule(trace: np.ndarray, thr_abs: float, k: float,
               window: int, warmup: int) -> np.ndarray:
    """Skor izine mutlak VEYA uyarlanabilir kuralı uygular → alarm maskesi."""
    hist: deque = deque(maxlen=window)
    out = np.zeros(len(trace), dtype=bool)
    for i, v in enumerate(trace):
        hit = v > thr_abs
        if not hit and len(hist) >= warmup:
            h = np.fromiter(hist, dtype=np.float64)
            med = float(np.median(h))
            mad = float(np.median(np.abs(h - med)))
            hit = v > med + k * max(1.4826 * mad, 0.05 * med, 1e-9)
        out[i] = hit
        if not hit:
            hist.append(v)      # alarm sürerken taban çizgisi donar
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean", default=str(BASE / "ur10e_clean.parquet"))
    ap.add_argument("--runs", type=int, default=4)
    ap.add_argument("--max-len", type=int, default=9000)
    ap.add_argument("--min-run", type=int, default=3000)
    ap.add_argument("--onset", type=float, default=0.5)
    ap.add_argument("--window", type=int, default=600)
    ap.add_argument("--warmup", type=int, default=200)
    ap.add_argument("--ks", type=float, nargs="*",
                    default=[4, 6, 8, 12, 20, 40, 80])
    args = ap.parse_args()

    sys.path.insert(0, str(PKG))
    sys.path.insert(0, str(BASE / "resources"))
    from anomaly_detection.detector import FusionDetector
    import ur10_solver_py

    nm_per_amp = np.array(json.loads((BASE / "current_to_torque.json").read_text())["nm_per_amp"])
    det = FusionDetector(
        BASE / "residual_ae_v2", BASE / "raw_ae_v2",
        BASE / "fusion_v2" / "fusion_config.json",
        BASE / "current_to_torque.json", BASE / "residual_calibration_clean.json",
        ur10_solver_py.InverseDynamicsSolverUR10(),
        adaptive=False,           # ham izi üret; kuralı sonra uygula
    )

    df = pd.read_parquet(args.clean)
    run = df["run_id"].to_numpy()
    e = np.concatenate([[0], np.flatnonzero(np.diff(run) != 0) + 1, [len(run)]])
    segs = sorted([(int(a), int(c - a)) for a, c in zip(e[:-1], e[1:])
                   if int(c - a) >= args.min_run], key=lambda s: -s[1])[:args.runs]
    Q = df[[f"q_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    QD = df[[f"qd_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    AMP = df[[f"tau_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    WR = df[["fx", "fy", "fz", "tx", "ty", "tz"]].to_numpy(np.float64)
    del df

    print("=" * 78)
    print("UYARLANABİLİR ALARM KURALI — k TARAMASI")
    print("=" * 78)
    print(f"  θ_mutlak = {det.thr_fused:.4f}   pencere {args.window} karar "
          f"({args.window*det.stride*det.extractor.dt:.0f} s)   ısınma {args.warmup}")
    print(f"  {len(segs)} koşu × {len(FAULTS)} senaryo\n  skor izleri hesaplanıyor...")

    rng = np.random.default_rng(0)
    traces: dict[str, list[tuple[np.ndarray, int]]] = {}
    for fname, cfg in FAULTS.items():
        traces[fname] = []
        for a, L0 in segs:
            L = min(L0, args.max_len)
            onset = int(L * args.onset)
            det.reset()
            tr, dec_idx = [], []
            for k in range(L):
                i = a + k
                q, qd = Q[i].copy(), QD[i].copy()
                amps, wr = AMP[i].copy(), WR[i].copy()
                if cfg["kind"] and k >= onset:
                    prog = (k - onset) / max(L - onset, 1)
                    if cfg["kind"] == "ramp":
                        amps[cfg["joint"]] += cfg["amp_nm"] * prog / nm_per_amp[cfg["joint"]]
                    elif cfg["kind"] == "q_step":
                        q[cfg["joint"]] += cfg["amp_rad"]
                    elif cfg["kind"] == "wrench_pulse":
                        c = onset + 0.15 * (L - onset)
                        wr += cfg["amp_n"] * np.exp(-0.5 * ((k - c) / (0.04 * L)) ** 2)
                    elif cfg["kind"] == "wrench_noise":
                        wr += rng.normal(0.0, cfg["amp_n"], 6)
                r = det.push(q, qd, amps, wr)
                if r is not None:
                    tr.append(r["fused"]); dec_idx.append(k)
            traces[fname].append((np.array(tr), int(np.searchsorted(dec_idx, onset))))
        print(f"    {cfg['tr']:<22} ✓")

    print(f"\n  {'k':>5}{'yanlış alarm':>14}", end="")
    for f in list(FAULTS)[1:]:
        print(f"{FAULTS[f]['tr'][:13]:>15}", end="")
    print()
    print("  " + "-" * 76)
    rows = []
    for k in args.ks:
        line = f"  {k:>5.0f}"
        fa = []
        for tr, on in traces["yok"]:
            d = apply_rule(tr, det.thr_fused, k, args.window, args.warmup)
            fa.append(d.mean())
        fa_rate = float(np.mean(fa))
        line += f"{100*fa_rate:>13.2f}%"
        row = {"k": k, "false_alarm": fa_rate, "faults": {}}
        for fname in list(FAULTS)[1:]:
            hits, lags = 0, []
            for tr, on in traces[fname]:
                d = apply_rule(tr, det.thr_fused, k, args.window, args.warmup)
                post = d[on:]
                if post.any():
                    hits += 1
                    lags.append(int(np.argmax(post)) * det.stride * det.extractor.dt * 1000)
            txt = (f"{hits}/{len(segs)} {np.median(lags):.0f}ms" if lags
                   else f"{hits}/{len(segs)}  —")
            line += f"{txt:>15}"
            row["faults"][fname] = {"runs_detected": hits, "runs": len(segs),
                                    "median_lag_ms": float(np.median(lags)) if lags else None}
        rows.append(row)
        print(line)

    # Plato ortası seçilir: kenardaki bir k, taban çizgisi istatistiği azıcık
    # kayınca kuralı bozar. Ortadaki değer her iki yöne de pay bırakır.
    ok = [r for r in rows if r["false_alarm"] <= 0.03]
    best = plateau = None
    if ok:
        score = lambda r: sum(f["runs_detected"] for f in r["faults"].values())  # noqa: E731
        top = max(score(r) for r in ok)
        plateau = [r for r in ok if score(r) == top]
        # Eşit tespit başarısında önce yanlış alarm, sonra gecikme ayırt eder:
        # büyük k tespit sayısını düşürmese de yavaş arızada gecikmeyi büyütüyor.
        lag = lambda r: sum(f["median_lag_ms"] or 0 for f in r["faults"].values())  # noqa: E731
        lo = min(r["false_alarm"] for r in plateau)
        cand = [r for r in plateau if r["false_alarm"] <= lo + 1e-9]
        lmin = min(lag(r) for r in cand)
        cand = [r for r in cand if lag(r) <= lmin + 1e-9]
        plateau = cand                      # gerçekten denk olan k'lar
        best = cand[len(cand) // 2]         # ortası: her iki yöne de pay bırakır
    print()
    if best:
        print(f"  Platolar: k ∈ [{plateau[0]['k']:.0f}, {plateau[-1]['k']:.0f}] "
              f"aynı tespit başarısını veriyor ({score(best)}/{4*len(segs)} koşu)")
        print(f"  Seçim: k = {best['k']:.0f} (plato ortası)  "
              f"yanlış alarm %{100*best['false_alarm']:.2f} ≤ %3 bütçesi")
    else:
        print("  ⚠ Hiçbir k %3 yanlış alarm bütçesini tutturamadı.")
    (BASE / "fusion_v2" / "adaptive_tuning.json").write_text(
        json.dumps({"theta_abs": det.thr_fused, "window": args.window,
                    "warmup": args.warmup, "sweep": rows,
                    "selected_k": best["k"] if best else None,
                    "plateau_k": [plateau[0]["k"], plateau[-1]["k"]] if plateau else None},
                   indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"  → fusion_v2/adaptive_tuning.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
