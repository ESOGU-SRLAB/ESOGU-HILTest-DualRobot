#!/usr/bin/env python3
"""
verify_online_features.py
=========================
Çevrimiçi öznitelik motorunu ÇEVRİMDIŞI hatta karşı sayısal olarak doğrular.

Anomali tespitinde en sinsi hata, eğitim verisini üreten hesabın canlı hesaptan
milimetrik farklı olmasıdır: model o farkı anomali sanar, eşik anlamını yitirir ve
hata hiçbir yerde hata gibi görünmez. Bu betik o riski kapatır — `ur10e_clean.parquet`
(amper) örneklerini tek tek `OnlineFeatureExtractor`'a besler ve çıktıyı
`ur10e_features_fric.parquet`in (çevrimdışı) aynı satırlarıyla karşılaştırır.

Kabul kriteri: maks mutlak fark < 1e-9.

Kullanım
--------
    python3 verify_online_features.py
    python3 verify_online_features.py --runs 20
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd

def _find_package(env: str = "AD_PACKAGE") -> Path:
    """
    ROS paketinin kökünü bulur (içinde anomaly_detection/features.py olan dizin).

    Bu betik paketin dışına taşınabilir — nitekim taşındı: hat dosyaları büyük
    parquet ve model ağırlıkları içerdiği için depoya girmiyor ve paketin yanında
    durmuyor. Yolu `__file__`'a göre saymak bu yüzden kırılgan; sırayla ortam
    değişkenine, bilinen konumlara ve üst dizinlere bakılır.
    """
    import os
    cands = []
    if os.environ.get(env):
        cands.append(Path(os.environ[env]))
    cands += [Path.home() / "colcon_ws" / "src" / "anomaly_detection"]
    cands += list(Path(__file__).resolve().parents)
    for c in cands:
        if (c / "anomaly_detection" / "features.py").exists():
            return c
    raise SystemExit(
        "anomaly_detection paketi bulunamadi. "
        f"{env} ortam degiskeniyle verin, or.:  "
        f"export {env}=~/colcon_ws/src/anomaly_detection")


PKG = _find_package()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean", default="ur10e_clean.parquet", help="amper cinsinden girdi")
    ap.add_argument("--features", default="ur10e_features_fric.parquet", help="çevrimdışı çıktı")
    ap.add_argument("--calib", default="current_to_torque.json")
    ap.add_argument("--residual-calib", default="residual_calibration_clean.json")
    ap.add_argument("--resources", default=(os.environ.get("AD_PACKAGE") or "/home/cem/colcon_ws/src/anomaly_detection") + "/resources",
                    help="FMU çözücüsünün (.so) bulunduğu dizin. Hat dosyaları paketin dışında durabildiği için göreli varsayılan güvenilmez; AD_PACKAGE ortam değişkeni varsa ona, yoksa bilinen paket yoluna bakılır.")
    ap.add_argument("--friction-model", default="friction_model.json",
                    help="çevrimdışı hat sürtünme çıkarıyorsa AYNI dosya; "
                         "boş verilirse terim uygulanmaz")
    ap.add_argument("--runs", type=int, default=10, help="kaç koşu denensin")
    ap.add_argument("--tol", type=float, default=1e-9)
    args = ap.parse_args()

    sys.path.insert(0, str(PKG))
    sys.path.insert(0, str(Path(args.resources).resolve()))
    from anomaly_detection.features import OnlineFeatureExtractor
    import ur10_solver_py

    nm_per_amp = json.loads(Path(args.calib).read_text())["nm_per_amp"]
    b = json.loads(Path(args.residual_calib).read_text())["b"]

    src = pd.read_parquet(args.clean)
    off = pd.read_parquet(args.features)
    if len(src) != len(off):
        print(f"HATA: satır sayıları farklı ({len(src):,} vs {len(off):,})", file=sys.stderr)
        return 2

    run = src["run_id"].to_numpy()
    Q = src[[f"q_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    QD = src[[f"qd_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    AMP = src[[f"tau_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    W6 = src[["fx", "fy", "fz", "tx", "ty", "tz"]].to_numpy(np.float64)
    del src

    OFF = {k: off[[f"{k}_{j}" for j in range(1, 7)]].to_numpy(np.float64)
           for k in ("r_int", "r_ext", "r_total", "tau_model")}
    OFF["tau"] = off[[f"tau_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    VALID = off["valid"].to_numpy(bool)
    del off

    fric = None
    if args.friction_model:
        fp = Path(args.friction_model)
        if fp.exists():
            fric = json.loads(fp.read_text(encoding="utf-8"))
    ex = OnlineFeatureExtractor(ur10_solver_py.InverseDynamicsSolverUR10(),
                                nm_per_amp, offset_b=b, friction=fric)
    print("=" * 66)
    print("ÇEVRİMİÇİ ↔ ÇEVRİMDIŞI ÖZNİTELİK DOĞRULAMASI")
    print("=" * 66)
    print(f"  SG gecikmesi: {ex.lag_samples} örnek ({ex.lag_seconds*1000:.0f} ms)")
    print(f"  sürtünme     : {'UYGULANIYOR (' + str(args.friction_model) + ')' if fric else 'yok'}")

    edges = np.concatenate([[0], np.flatnonzero(np.diff(run) != 0) + 1, [len(run)]])
    segs = [(int(a), int(c - a)) for a, c in zip(edges[:-1], edges[1:])]
    segs = [s for s in segs if s[1] >= ex.sg_window][:args.runs]

    diffs = {k: 0.0 for k in ("tau", "tau_model", "r_total", "r_ext", "r_int")}
    n_cmp = 0
    n_valid_mismatch = 0
    for a, L in segs:
        ex.reset()
        for i in range(a, a + L):
            f = ex.push(Q[i], QD[i], AMP[i], W6[i])
            if f is None:
                continue
            j = i - ex.lag_samples                      # merkez örneğin indeksi
            if not VALID[j]:
                n_valid_mismatch += 1
                continue
            for k in diffs:
                key = "tau" if k == "tau" else k
                diffs[k] = max(diffs[k], float(np.abs(
                    (f[key] if k != "tau" else f["tau"]) - OFF[key][j]).max()))
            n_cmp += 1

    print(f"  karşılaştırılan örnek: {n_cmp:,}  ({len(segs)} koşu)")
    if n_valid_mismatch:
        print(f"  ⚠ geçerlilik maskesi dışında kalan {n_valid_mismatch:,} örnek atlandı")
    print(f"\n  {'kanal':<14}{'maks mutlak fark':>20}")
    print("  " + "-" * 34)
    ok = True
    for k, v in diffs.items():
        good = v < args.tol
        ok &= good
        print(f"  {k:<14}{v:>20.3e}   {'✅' if good else '❌'}")
    print("\n" + ("  SONUÇ: ✅ Çevrimiçi motor çevrimdışı hatla BİREBİR aynı."
                  if ok else
                  f"  SONUÇ: ❌ Fark {args.tol:.0e} toleransını aşıyor — düğüm kullanılmamalı."))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
