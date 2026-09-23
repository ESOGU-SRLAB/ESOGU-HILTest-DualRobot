#!/usr/bin/env python3
"""
convert_effort_to_nm.py
=======================
`ur10e_raw_features.parquet` içindeki τ (effort) kanallarını AMPER'den Nm'ye çevirir.

Neden
-----
UR sürücüsü `effort` alanına motor akımını yazıyor (Nm değil):

    "The effort field contains the currents reported by the joints and not the
     actual efforts in a physical sense."   — ur_robot_driver / Controllers

FMU ise Nm üretiyor, `J(q)ᵀ·F_KTS` de Nm. Ölçümü de Nm'ye çekince hattın tamamı
tek birimde olur; bildirinin Nm cinsinden verdiği arıza genlikleri doğrudan
uygulanabilir ve aşağı akıştaki ölçek dönüşümlerine hiç gerek kalmaz.

Ne YAPMAZ
---------
* Kaynak CSV'ye dokunmaz.
* Girdi parquet'ini değiştirmez; yeni bir dosya yazar.
* Model doğruluğunu değiştirmez — AE zaten kanal başına z-skor normalize ediyor,
  sabit bir çarpan normalizasyonda tamamen soğurulur. Kazanç yorumlanabilirlik.

Dönüşüm
-------
    τ_Nm = i / a_j

`a_j` yalnızca ÖLÇEK; akım ofseti (b) bilerek uygulanmıyor — onu `generate_residuals`
kendi afin kalibrasyonunda zaten buluyor.

Katsayılar
----------
Öntanımlı değerler, senin 1.124.432 örneklik veri kümende τ_ölç ile τ_model
regresyonundan ölçüldü:

    eklem            a        1/a (Nm/A)   R²      kaynak
    shoulder_pan   0.0923       10.8     0.002    ← shoulder_lift'ten (aynı 330 Nm ailesi)
    shoulder_lift  0.0923       10.8     0.918    doğrudan fit
    elbow          0.1099        9.1     0.899    doğrudan fit
    wrist_1        0.1347        7.4     0.175    doğrudan fit
    wrist_2        0.1347        7.4     0.003    ← wrist_1'den (aynı 56 Nm ailesi)
    wrist_3        0.1347        7.4     0.000    ← wrist_1'den (aynı 56 Nm ailesi)

Yerçekimi yükü taşımayan eklemlerde regresyon `a`'yı gürültüden uyduruyor
(wrist_3'te işareti bile ters çıkıyor), o yüzden oralarda aynı tork sınırına
sahip güvenilir eklemin katsayısı kullanılıyor.

`--calibration residual_calibration.json` verilirse katsayılar oradan okunur.

Doğrulama
---------
Dönüşümden sonra `generate_residuals.py --units nm` çalıştırıldığında, kendi
kalibrasyonu **a ≈ 1,0** bulmalı. Bulmuyorsa katsayılar yanlış demektir.

Kullanım
--------
    python3 convert_effort_to_nm.py
    python3 convert_effort_to_nm.py --calibration residual_calibration.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

JN = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

# Ölçülen varsayılan katsayılar (bkz. modül başlığı)
DEFAULT_A = [0.0923, 0.0923, 0.1099, 0.1347, 0.1347, 0.1347]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default="ur10e_raw_features.parquet")
    ap.add_argument("--out", default="ur10e_raw_features_nm.parquet")
    ap.add_argument("--calibration", default=None,
                    help="generate_residuals.py'nin yazdığı residual_calibration.json")
    ap.add_argument("--a", nargs=6, type=float, default=None,
                    help="katsayıları elle ver (6 adet)")
    args = ap.parse_args()

    if args.a:
        A = np.array(args.a, dtype=float); src = "komut satırı"
    elif args.calibration and Path(args.calibration).exists():
        cj = json.loads(Path(args.calibration).read_text(encoding="utf-8"))
        A = np.array(cj["a"], dtype=float); src = args.calibration
    else:
        A = np.array(DEFAULT_A, dtype=float); src = "gömülü varsayılan (ölçülmüş)"

    if np.any(np.abs(A) < 1e-6):
        print(f"HATA: sıfıra yakın katsayı var: {A.tolist()}", file=sys.stderr)
        return 2

    print("=" * 68)
    print("EFFORT (AMPER) → TORK (Nm) DÖNÜŞÜMÜ")
    print("=" * 68)
    print(f"  girdi      : {args.raw}")
    print(f"  çıktı      : {args.out}")
    print(f"  katsayılar : {src}\n")

    df = pd.read_parquet(args.raw)
    tau_cols = [f"tau_{j}" for j in range(1, 7)]
    missing = [c for c in tau_cols if c not in df.columns]
    if missing:
        print(f"HATA: eksik kolon: {missing}", file=sys.stderr)
        return 2

    print(f"  {'eklem':<15}{'a':>9}{'1/a Nm/A':>11}{'std önce (A)':>14}{'std sonra (Nm)':>16}")
    print("  " + "-" * 66)
    for j, c in enumerate(tau_cols):
        before = float(df[c].std())
        df[c] = df[c].to_numpy(np.float64) / A[j]
        print(f"  {JN[j]:<15}{A[j]:>9.4f}{1/A[j]:>11.1f}{before:>14.3f}{df[c].std():>16.3f}")

    df.to_parquet(args.out, index=False)
    sz = Path(args.out).stat().st_size / 1e6
    print(f"\n  {len(df):,} satır yazıldı ({sz:.1f} MB)")

    # Fiziksel sağlama
    peak = float(np.abs(df["tau_2"]).quantile(0.999))
    print(f"\n  Fiziksel sağlama — shoulder_lift torku:")
    print(f"    p99,9 tepe = {peak:.1f} Nm")
    print(f"    kol yatayken beklenen yerçekimi torku ≈ 121 Nm (ur_description kütleleri)")
    if 60 <= peak <= 250:
        print(f"    ✅ makul aralıkta")
    else:
        print(f"    ⚠ beklenenden uzak — katsayıları kontrol et")

    print(f"\n  Sonraki adım:")
    print(f"    python3 generate_residuals.py --raw {args.out} --units nm \\")
    print(f"            --out ur10e_hybrid_residual.parquet --backend so")
    print(f"    → kalibrasyon tablosunda a ≈ 1,0 çıkmalı (dönüşümün sağlaması)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
