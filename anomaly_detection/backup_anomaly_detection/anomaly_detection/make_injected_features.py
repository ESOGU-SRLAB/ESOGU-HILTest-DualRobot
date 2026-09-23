#!/usr/bin/env python3
"""
make_injected_features.py
=========================
ÖLÇÜM UZAYINDA arıza enjekte edip kalıntıyı YENİDEN HESAPLAR.

Neden
-----
Bildiriden miras protokol arızayı iki temsil uzayına AYRI AYRI ve farklı
genliklerle enjekte ediyor: ham modele `tau_3 += 15 Nm`, kalıntı modeline
`r_int_3 += 25 Nm`. Yani iki model aynı fiziksel arızayı görmüyor, birbirinden
bağımsız iki bozulma görüyor ve kalıntı tarafındaki genlik elle seçilmiş bir
sayı. Makale bunu "injected in measurement space" diye anlatıyor; değil.

Doğrusu: arızayı yalnız ÖLÇÜLEN kanallara (τ, q, KTS) koymak ve kalıntıyı
hattın kendisiyle yeniden üretmek. O zaman kalıntıdaki bozulma bir varsayım
değil, ters dinamiğin ve Jacobian aktarımının sonucudur:

    q_5 basamağı  → τ̂_model değişir  → r_ic değişir      (fiziksel yayılım)
    τ_3 rampası   → r_top doğrudan kayar
    KTS darbesi   → r_dis = J(q)ᵀF üzerinden yayılır

Sürtünme terimi q̇'nin fonksiyonu ve hiçbir senaryo q̇'ye dokunmuyor, yani
düzeltme bir arıza imzasını soğuramaz — bu betik onu ayrıca doğrular.

Katsayılar YENİDEN UYDURULMAZ: --friction file ile eğitim koşularından gelen
dosya kullanılır. Aksi hâlde arızalı veri sürtünme katsayısına sızardı.

Kullanım
--------
    python3 make_injected_features.py --split test --out-dir injected_test
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from inject_faults import FAULTS, inject  # noqa: E402

HERE = Path(__file__).resolve().parent


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean", default="ur10e_clean_nm.parquet",
                    help="ÖLÇÜM uzayı (Nm) parquet — arıza buraya girer")
    ap.add_argument("--splits", default="splits.json")
    ap.add_argument("--split", default="test", choices=["val", "test"])
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--resources", default=(os.environ.get("AD_PACKAGE") or "/home/cem/colcon_ws/src/anomaly_detection") + "/resources",
                    help="FMU çözücüsünün (.so) bulunduğu dizin. Hat dosyaları paketin dışında durabildiği için göreli varsayılan güvenilmez; AD_PACKAGE ortam değişkeni varsa ona, yoksa bilinen paket yoluna bakılır.")
    ap.add_argument("--calib-in", default="residual_calibration_fric.json",
                    help="TEMİZ hattın kalibrasyonu; enjekte edilmiş veride "
                         "yeniden uydurulmaz (yoksa arıza b'yi kaydırır ve "
                         "dokunulmamış koşuların kalıntısı da değişir)")
    ap.add_argument("--friction-model", default="friction_model.json")
    ap.add_argument("--no-friction", action="store_true",
                    help="sürtünme ablasyonu için: kalıntıyı sürtünme terimi OLMADAN "
                         "üret. Ablasyonun kontrollü olması için enjeksiyon ve "
                         "kalıntı hattı, karşılaştırılan modelinkiyle aynı olmalı.")
    ap.add_argument("--keep-measurement", action="store_true",
                    help="ara (enjekte edilmiş ölçüm) parquet'lerini silme")
    args = ap.parse_args()

    out_dir = Path(args.out_dir or f"injected_{args.split}")
    out_dir.mkdir(parents=True, exist_ok=True)
    SP = json.loads(Path(args.splits).read_text(encoding="utf-8"))
    runs = SP[args.split]
    min_len = SP["min_fault_len"]

    df = pd.read_parquet(args.clean)
    rid = df["run_id"].to_numpy()
    edges = np.concatenate([[0], np.flatnonzero(np.diff(rid) != 0) + 1, [len(rid)]])
    sl = {int(rid[a]): (int(a), int(b - a)) for a, b in zip(edges[:-1], edges[1:])}
    segs = [sl[r] for r in runs if sl[r][1] >= min_len]

    print("=" * 74)
    print(f"ÖLÇÜM UZAYINDA ENJEKSİYON — bölme '{args.split}'")
    print("=" * 74)
    print(f"  {len(df):,} örnek · {len(runs)} koşu · arıza taşıyıcı {len(segs)} koşu")

    # Enjeksiyon YALNIZ ölçülen kanallara. `raw` spesifikasyonu zaten ölçüm
    # kanallarını hedefliyor (tau_j [Nm], q_j [rad], fx..tz [N/Nm]); kalıntı
    # spesifikasyonu artık KULLANILMIYOR — kalıntıdaki bozulma türetiliyor.
    cols = list(df.columns)
    meta = {"split": args.split, "n_fault_runs": len(segs),
            "segments": [[int(a), int(L)] for a, L in segs],
            "note": "faults injected in MEASUREMENT space; residuals recomputed"}
    for name, cfg in FAULTS.items():
        tgt = [c for c in cfg["raw"]["cols"] if c in cols]
        missing = [c for c in cfg["raw"]["cols"] if c not in cols]
        if missing:
            print(f"HATA: {name}: {missing} ölçüm parquet'inde yok.", file=sys.stderr)
            return 2
        arr = df[cols].to_numpy(dtype=np.float64)
        # scale=None: genlikler zaten fiziksel birimde (Nm / rad / N)
        arr = inject(arr, cols, cfg, "raw", rng=np.random.default_rng(0),
                     scale=None, segments=segs)
        mpath = out_dir / f"meas_{name}.parquet"
        pd.DataFrame(arr, columns=cols).astype(
            {"run_id": df["run_id"].dtype}).to_parquet(mpath, index=False)

        fpath = out_dir / f"features_{name}.parquet"
        cmd = [sys.executable, str(HERE / "generate_residuals.py"),
               "--raw", str(mpath), "--out", str(fpath),
               "--calib-out", str(out_dir / f"calib_{name}.json"),
               "--calibration-in", args.calib_in,
               "--resources", args.resources, "--backend", "so",
               "--units", "nm", "--calibrate", "offset", "--fts-frame", "tool",
               *(["--friction", "none"] if args.no_friction
                 else ["--friction", "file", "--friction-model", args.friction_model])]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout[-2000:]); print(r.stderr[-2000:], file=sys.stderr)
            return 2
        if not args.keep_measurement:
            mpath.unlink()
        print(f"  {cfg['en']:<16} {', '.join(tgt):<28} → {fpath.name}")
        meta[name] = {"channels": tgt, "amplitude": cfg["raw"]["amp"]}

    (out_dir / "manifest.json").write_text(
        json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n  → {out_dir}/  ({len(FAULTS)} senaryo + manifest.json)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
