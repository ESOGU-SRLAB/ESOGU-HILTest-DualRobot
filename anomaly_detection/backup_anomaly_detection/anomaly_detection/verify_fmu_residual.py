#!/usr/bin/env python3
"""
verify_fmu_residual.py
======================
`ur10e_hybrid_residual.parquet`'in gerçekten FMU ters dinamik çıktısı içerdiğini
doğrular.

Neden gerekli
-------------
`resources/model.py` içindeki FMU arka ucu, C++ solver yüklenemezse hatayı yutup
sessizce geri dönüyor (`if not self.solver: return`). Bu durumda τ_model sıfır kalır,
`r_total = τ_ölçülen − 0 = τ_ölçülen` olur ve parquet yine üretilir — ama içi
tamamen yanlış olur. Eğitim sırasında FMU hiç çalışmadığı için bu hata eğitim
çıktısında görünmez.

Yapılan kontroller
------------------
  1. İki parquet'in satır sayısı ve `t` kolonu hizalı mı?
  2. Tanımsal özdeşlik: r_int + r_ext == r_total ?
  3. τ_model = τ_ölçülen − r_total  → sıfır mı? (sıfırsa FMU çalışmamış)
  4. τ_model'in eklem başına istatistikleri makul mü?
  5. FİZİKSEL TEST: quasi-statik anlarda yerçekimi torku, tabanı dik olan robotta
     Eklem 1 (shoulder_pan, ekseni düşey) etrafında SIFIR olmalıdır. τ_model'in
     1. ekleminin küçük, 2. ekleminin (shoulder_lift) büyük olması beklenir.
     Bu, sıfır olmayan ama uydurma bir τ_model'i de yakalar.

Kullanım
--------
    python verify_fmu_residual.py
    python verify_fmu_residual.py --residual ur10e_hybrid_residual.parquet ^
                                  --raw ur10e_raw_features.parquet
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import pandas as pd

JN = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--residual", default="ur10e_hybrid_residual.parquet")
    ap.add_argument("--raw", default="ur10e_raw_features.parquet")
    ap.add_argument("--slow", type=float, default=0.02,
                    help="quasi-statik eşiği [rad/s]")
    args = ap.parse_args()

    print("=" * 70)
    print("FMU ARTIK DOĞRULAMASI")
    print("=" * 70)

    rc = ["t"] + [f"r_total_{j}" for j in range(1, 7)] + \
         [f"r_ext_{j}" for j in range(1, 7)] + [f"r_int_{j}" for j in range(1, 7)]
    wc = ["t"] + [f"tau_{j}" for j in range(1, 7)] + [f"qd_{j}" for j in range(1, 7)]

    R = pd.read_parquet(args.residual, columns=rc)
    W = pd.read_parquet(args.raw, columns=wc)
    print(f"  kalıntı parquet: {len(R):,} satır")
    print(f"  ham parquet    : {len(W):,} satır")

    ok_all = True

    # ── 1) hizalama ──
    print("\n[1] Satır hizalaması")
    if len(R) != len(W):
        print(f"  ❌ satır sayıları farklı — karşılaştırma geçersiz")
        return 2
    dt = np.abs(R["t"].to_numpy() - W["t"].to_numpy())
    aligned = dt.max() < 1e-6
    print(f"  |Δt| maks = {dt.max():.3e} s   → {'HİZALI ✅' if aligned else 'HİZASIZ ❌'}")
    if not aligned:
        print(f"  ⚠ satırlar eşleşmiyor; aşağıdaki testler güvenilmez.")
        ok_all = False

    r_tot = R[[f"r_total_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    r_ext = R[[f"r_ext_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    r_int = R[[f"r_int_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    tau = W[[f"tau_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    qd = W[[f"qd_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    del R, W

    # ── 2) tanımsal özdeşlik ──
    print("\n[2] Tanımsal özdeşlik  r_int + r_ext == r_total")
    d = np.abs(r_int + r_ext - r_tot).max()
    print(f"  maks sapma = {d:.3e}   → {'TUTUYOR ✅' if d < 1e-3 else 'TUTMUYOR ❌'}")
    ok_all &= d < 1e-3

    # ── 3) τ_model sıfır mı? ──
    print("\n[3] τ_model = τ_ölçülen − r_total   (FMU gerçekten çalışmış mı?)")
    tau_model = tau - r_tot
    amax = np.abs(tau_model).max()
    frac_zero = float((np.abs(tau_model) < 1e-9).all(axis=1).mean())
    print(f"  |τ_model| maks           = {amax:.4f} Nm")
    print(f"  tamamen sıfır satır oranı = %{100*frac_zero:.2f}")
    if amax < 1e-6:
        print("  ❌ τ_model TAMAMEN SIFIR → FMU hiç çalışmamış, parquet geçersiz.")
        ok_all = False
    elif frac_zero > 0.05:
        print(f"  ⚠ satırların %{100*frac_zero:.1f}'inde τ_model sıfır — FMU aralıklı çalışmış olabilir.")
        ok_all = False
    else:
        print("  ✅ τ_model sıfırdan farklı → FMU çıktı üretmiş.")

    # ── 4) eklem bazında istatistikler ──
    print("\n[4] Eklem bazında büyüklükler [Nm]")
    print(f"  {'eklem':<16}{'τ_ölçülen std':>15}{'τ_model std':>13}{'r_total std':>13}{'korel':>8}")
    print("  " + "-" * 65)
    for j in range(6):
        c = np.corrcoef(tau[:, j], tau_model[:, j])[0, 1]
        print(f"  {JN[j]:<16}{tau[:, j].std():>15.3f}{tau_model[:, j].std():>13.3f}"
              f"{r_tot[:, j].std():>13.3f}{c:>8.3f}")

    # ── 5) fiziksel test ──
    print(f"\n[5] Fiziksel test — quasi-statik yerçekimi imzası (|q̇| < {args.slow} rad/s)")
    slow = np.linalg.norm(qd, axis=1) < args.slow
    ns = int(slow.sum())
    print(f"  quasi-statik örnek: {ns:,} (%{100*slow.mean():.1f})")
    if ns < 1000:
        print("  ⚠ yeterli quasi-statik örnek yok, test atlandı.")
    else:
        tm = tau_model[slow]
        rms = np.sqrt((tm ** 2).mean(axis=0))
        print(f"  {'eklem':<16}{'τ_model RMS':>13}")
        print("  " + "-" * 30)
        for j in range(6):
            print(f"  {JN[j]:<16}{rms[j]:>13.3f}")
        # Tabanı dik robotta yerçekimi, düşey eksenli Eklem 1 etrafında tork üretmez.
        ratio = rms[0] / (rms[1] + 1e-9)
        print(f"\n  Eklem1 / Eklem2 RMS oranı = {ratio:.3f}")
        if rms.max() < 1e-9:
            # 0/0 durumunda oran 0 çıkıp testi yanlışlıkla geçiriyordu.
            print("  ❌ τ_model tamamen sıfır — fiziksel test uygulanamaz, model çalışmamış.")
            ok_all = False
        elif ratio < 0.25:
            print("  ✅ Beklenen imza: shoulder_pan ≈ 0, shoulder_lift baskın.")
            print("     τ_model gerçek bir yerçekimi/dinamik modelden geliyor.")
        else:
            print("  ⚠ shoulder_pan torku beklenenden büyük. Ya taban dik değil,")
            print("     ya q̈ gürültüsü atalet terimini şişiriyor, ya da model şüpheli.")
            ok_all = False

    # ── özet ──
    print("\n" + "=" * 70)
    if ok_all:
        print("SONUÇ: ✅ Parquet gerçek FMU ters dinamik çıktısı içeriyor.")
    else:
        print("SONUÇ: ⚠ Yukarıdaki işaretli maddelere bak — parquet şüpheli.")
    print("=" * 70)
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
