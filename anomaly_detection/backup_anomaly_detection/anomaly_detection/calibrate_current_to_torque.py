#!/usr/bin/env python3
"""
calibrate_current_to_torque.py
==============================
Motor akımı [A] → eklem torku [Nm] katsayılarını YERÇEKİMİNDEN türetir.

Neden yeni bir betik (denetim bulgusu F4)
-----------------------------------------
`convert_effort_to_nm.py`'nin gömülü katsayıları τ_model'e regresyonla bulunmuştu
ve doğrulaması "generate_residuals yeniden çalıştırıldığında a ≈ 1,0 çıkmalı"
şeklinde tanımlanmıştı. Bu döngüsel: katsayı zaten o oranı 1 yapacak şekilde
seçildiği için sağlama her koşulda geçer. Üstelik ölçümü fiziğe değil, q̈ tahmini
bozuk olan modele kalibre ediyordu.

Buradaki ölçüt bağımsız: yalnızca quasi-statik örneklerde (|q̇| küçük, |q̈| küçük)
dinamik terimler kaybolur ve

    τ(q) ≈ g(q)          [Nm]        ← FMU çözücüsünün yerçekimi vektörü
    i                    [A]         ← ölçülen motor akımı

kalır. g(q), q̈ tahminine de atalet matrisine de bağlı DEĞİL; hattaki hiçbir
hatadan etkilenmez. Eklem başına  g ≈ k·(i − i₀)  uydurulur; k [Nm/A].

Ölçülebilirlik
--------------
Yöntem yalnızca yerçekimi yükü TAŞIYAN eklemlerde çalışır:

    shoulder_pan   g ≡ 0 (dikey eksen)          → ölçülemez
    shoulder_lift  g std ≈ 39 Nm                → ölçülür  ✅
    elbow          g std ≈ 17 Nm                → ölçülür  ✅
    wrist_1/2/3    g std ≈ 0,3 / 0,02 / 0,03 Nm → ölçülemez

Ölçülemeyen eklemlerde aynı eklem modülü ailesinden (--family) katsayı
kopyalanır ve JSON'da `trusted: false` olarak işaretlenir. ÖNEMLİ: bu seçim
tespit başarımını etkilemez — özkodlayıcı kanal başına z-skor normalize ettiği
için sabit çarpan soğurulur. Etkilediği tek şey fiziksel yorumlanabilirlik ve
Nm cinsinden verilen arıza genliklerinin anlamı.

Kullanım
--------
    python3 calibrate_current_to_torque.py
    python3 calibrate_current_to_torque.py --raw ur10e_clean.parquet --qd-max 0.005
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

JN = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
TAU_MAX = [330.0, 330.0, 150.0, 56.0, 56.0, 56.0]      # UR10e eklem tork sınırları [Nm]
# Aynı eklem modülü ailesi: taban+omuz aynı büyük modül, dirsek orta, bilekler küçük.
FAMILY = {0: "buyuk", 1: "buyuk", 2: "orta", 3: "kucuk", 4: "kucuk", 5: "kucuk"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default="ur10e_clean.parquet")
    ap.add_argument("--resources", default="resources")
    ap.add_argument("--out", default="current_to_torque.json")
    ap.add_argument("--qd-max", type=float, default=0.005,
                    help="quasi-statik eşiği [rad/s]")
    ap.add_argument("--min-r2", type=float, default=0.30,
                    help="bu R²'nin altında kalan eklem 'ölçülemez' sayılır")
    ap.add_argument("--max-samples", type=int, default=60_000,
                    help="regresyona girecek en fazla örnek (alt örnekleme)")
    args = ap.parse_args()

    sys.path.insert(0, str(Path(args.resources).resolve()))
    try:
        import ur10_solver_py
    except ImportError as e:
        print(f"HATA: ur10_solver_py import edilemedi: {e}", file=sys.stderr)
        return 2
    solver = ur10_solver_py.InverseDynamicsSolverUR10()

    print("=" * 70)
    print("AKIM → TORK KALİBRASYONU (quasi-statik yerçekimi)")
    print("=" * 70)
    print(f"  girdi : {args.raw}")
    print(f"  çıktı : {args.out}")

    cols = [f"q_{j}" for j in range(1, 7)] + [f"qd_{j}" for j in range(1, 7)] + \
           [f"tau_{j}" for j in range(1, 7)]
    df = pd.read_parquet(args.raw, columns=cols)
    Q = df[[f"q_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    QD = df[[f"qd_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    I = df[[f"tau_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    del df

    qs = np.abs(QD).max(axis=1) < args.qd_max
    n_qs = int(qs.sum())
    step = max(1, n_qs // args.max_samples)
    Qs, Is = Q[qs][::step], I[qs][::step]
    print(f"\n  toplam {len(Q):,} örnek → quasi-statik {n_qs:,} (%{100*qs.mean():.1f}) "
          f"→ regresyon {len(Qs):,} (adım {step})")

    G = np.array([np.asarray(solver.getGravityVector(list(q))).ravel() for q in Qs])

    print(f"\n  {'eklem':<15}{'Nm/A':>9}{'i₀ [A]':>9}{'R²':>8}{'korel':>8}"
          f"{'g std':>9}{'i std':>8}   durum")
    print("  " + "-" * 78)
    res = []
    for j in range(6):
        g, i = G[:, j], Is[:, j]
        if g.std() < 1e-6:
            res.append({"joint": JN[j], "nm_per_amp": None, "offset_a": 0.0,
                        "r2": 0.0, "corr": 0.0, "measurable": False,
                        "reason": "yerçekimi torku sıfır (dikey eksen)"})
            print(f"  {JN[j]:<15}{'—':>9}{'—':>9}{'—':>8}{'—':>8}"
                  f"{g.std():>9.4f}{i.std():>8.4f}   ölçülemez (g ≡ 0)")
            continue
        M = np.vstack([i, np.ones(len(i))]).T
        sol, *_ = np.linalg.lstsq(M, g, rcond=None)
        k, c = float(sol[0]), float(sol[1])
        r2 = float(1 - (g - M @ sol).var() / g.var())
        cr = float(np.corrcoef(i, g)[0, 1])
        okm = r2 >= args.min_r2
        res.append({"joint": JN[j], "nm_per_amp": k, "offset_a": -c / k if abs(k) > 1e-9 else 0.0,
                    "r2": r2, "corr": cr, "measurable": bool(okm),
                    "reason": "" if okm else f"R² {r2:.3f} < {args.min_r2}"})
        print(f"  {JN[j]:<15}{k:>9.3f}{-c/k if abs(k)>1e-9 else 0:>9.3f}{r2:>8.3f}{cr:>8.3f}"
              f"{g.std():>9.4f}{i.std():>8.4f}   {'ölçüldü ✅' if okm else 'ölçülemez'}")

    # ── ölçülemeyen eklemler: aynı modül ailesinden kopyala ──
    print()
    for j in range(6):
        if res[j]["measurable"]:
            res[j]["trusted"] = True
            res[j]["source"] = "quasi-statik yerçekimi regresyonu"
            continue
        fam = FAMILY[j]
        donors = [k for k in range(6) if FAMILY[k] == fam and res[k]["measurable"]]
        if donors:
            best = max(donors, key=lambda k: res[k]["r2"])
            res[j]["nm_per_amp"] = res[best]["nm_per_amp"]
            res[j]["source"] = f"'{JN[best]}' ekleminden kopyalandı (aynı '{fam}' modül ailesi)"
        else:
            # aileden de veri yoksa tork sınırı oranıyla ölçekle — en zayıf varsayım
            best = max(range(6), key=lambda k: res[k]["r2"] if res[k]["measurable"] else -1)
            res[j]["nm_per_amp"] = res[best]["nm_per_amp"] * TAU_MAX[j] / TAU_MAX[best]
            res[j]["source"] = (f"'{JN[best]}' katsayısının tork sınırı oranıyla ölçeklenmesi "
                                f"({TAU_MAX[j]}/{TAU_MAX[best]}) — ZAYIF varsayım")
        res[j]["trusted"] = False
        print(f"  ⚠ {res[j]['joint']:<14} ölçülemedi ({res[j]['reason']}) → "
              f"{res[j]['nm_per_amp']:.3f} Nm/A, kaynak: {res[j]['source']}")

    # ── fiziksel makullük ──
    print(f"\n  Fiziksel makullük — tepe akım × katsayı, eklem sınırına karşı:")
    print(f"  {'eklem':<15}{'tepe |i| A':>12}{'→ Nm':>9}{'sınır Nm':>10}{'':>4}")
    print("  " + "-" * 52)
    for j in range(6):
        pk = float(np.abs(I[:, j]).max())
        nm = pk * res[j]["nm_per_amp"]
        flag = "✅" if nm <= TAU_MAX[j] else "⚠ sınır aşımı"
        print(f"  {JN[j]:<15}{pk:>12.2f}{nm:>9.1f}{TAU_MAX[j]:>10.0f}   {flag}")

    payload = {
        "method": "quasi-static gravity regression",
        "source_parquet": str(args.raw),
        "qd_max": args.qd_max, "min_r2": args.min_r2,
        "n_quasi_static": n_qs, "n_regression": len(Qs),
        "nm_per_amp": [r["nm_per_amp"] for r in res],
        # convert_effort_to_nm.py ile uyum: o betik tau/a yapıyor, yani a = A/Nm
        "a": [1.0 / r["nm_per_amp"] for r in res],
        "trusted": [r["trusted"] for r in res],
        "r2": [r["r2"] for r in res],
        "per_joint": res,
        "note": ("nm_per_amp: tork [Nm] = nm_per_amp * akim [A]. trusted=false olan "
                 "eklemlerde katsayi olculemedi, ayni modul ailesinden kopyalandi; "
                 "tespit basarimini etkilemez (AE z-skor normalize ediyor), yalnizca "
                 "fiziksel yorumlanabilirligi ve Nm cinsinden arizanin anlamini etkiler."),
    }
    Path(args.out).write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    ntr = sum(r["trusted"] for r in res)
    print(f"\n  → {args.out}   ({ntr}/6 eklem ölçüldü, {6-ntr}/6 varsayım)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
