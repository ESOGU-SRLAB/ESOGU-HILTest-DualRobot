#!/usr/bin/env python3
"""
prepare_dataset.py
==================
ur10e_raw_features.parquet  →  ur10e_clean.parquet   (kesintisiz 500 Hz koşular)

Neden gerekli (denetim bulgusu F1)
----------------------------------
Kaynak `ros-joint-states.csv` sürekli bir 500 Hz akışı DEĞİL; 2025-09-01 ile
2025-11-19 arasına yayılan bir Kafka/Elasticsearch dökümü ve satırlar zamana göre
sıralı değil. `extract_raw_features.py` yalnızca 20.000'lik chunk *içinde*
sıraladığı için parquet global olarak karışık kalıyor:

    * satırların %8,9'u bir öncekinin birebir kopyası
    * ardışık geçişlerin %17,5'inde dt > 10 ms
    * 8.477 adet > 1 s sıçrama
    * 100 örneklik pencerelerin %85,1'i en az bir kopukluk içeriyor

Bu hâliyle q̈ türevi ve LSTM penceresi anlamsız. Bu betik hattı düzeltir.

Ne yapar
--------
1. Global, kararlı zaman sıralaması (chunk-içi değil).
2. Bir öncekiyle birebir aynı olan satırları atar.
3. dt ∈ (0, --max-gap] koşulunu sağlayan KESİNTİSİZ koşulara böler.
4. Kısa koşuları atar (--min-run).
5. Her koşuyu tekdüze 2 ms (500 Hz) ızgaraya doğrusal ara-değerle oturtur —
   böylece Savitzky-Golay'ın sabit `delta` varsayımı gerçekten geçerli olur.
6. `run_id` ve `interp` (ara-değerlenmiş örnek maskesi) kolonlarını ekler.

Aşağı akış (generate_residuals, train_ae) `run_id` kolonunu görünce koşu bazlı
çalışır; pencereler asla iki koşuya yayılmaz.

Fiziksel sağlama
----------------
Koşu içi |Δq| değerleri UR10e maksimum eklem hızlarıyla karşılaştırılır. Global
sıralamadan sonra bu kontrol hiç devreye girmiyor (ölçüldü) — yani veri örnek
düzeyinde tutarlı, tek sorun zaman boşluklarıydı.

Kullanım
--------
    python3 prepare_dataset.py
    python3 prepare_dataset.py --max-gap 0.006 --min-run 200
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

# UR10e maksimum eklem hızları [rad/s] (taban/omuz/dirsek 120°/s, bilekler 180°/s)
QD_MAX = np.array([2.0944, 2.0944, 3.1416, 3.1416, 3.1416, 3.1416])

JOINT_COLS = [f"q_{j}" for j in range(1, 7)]


def fmt(s: float) -> str:
    return f"{s:.0f}s" if s < 60 else f"{s/60:.1f}dk"


def main() -> int:
    ap = argparse.ArgumentParser(description="Kesintisiz 500 Hz koşu çıkarımı (denetim F1)")
    ap.add_argument("--raw", default="ur10e_raw_features.parquet")
    ap.add_argument("--out", default="ur10e_clean.parquet")
    ap.add_argument("--dt", type=float, default=0.002,
                    help="hedef ızgara adımı [s] (varsayılan 2 ms = 500 Hz)")
    ap.add_argument("--max-gap", type=float, default=0.008,
                    help="koşuyu kesmeyen en büyük dt [s]. 8 ms ≈ en fazla 3 kayıp örnek; "
                         "%%4,9 ara-değerle 11.598 pencere verir.")
    ap.add_argument("--min-run", type=int, default=150,
                    help="tutulacak en kısa koşu (örnek). 150 = 1 pencere + kenar payı")
    ap.add_argument("--speed-margin", type=float, default=1.5,
                    help="|Δq| ≤ qd_max·dt·marj kontrolü için pay")
    args = ap.parse_args()

    t_wall = time.time()
    print("=" * 70)
    print("VERİ HAZIRLAMA — kesintisiz 500 Hz koşular")
    print("=" * 70)
    print(f"  girdi     : {args.raw}")
    print(f"  çıktı     : {args.out}")
    print(f"  ızgara    : {args.dt*1000:.1f} ms   boşluk sınırı: {args.max_gap*1000:.0f} ms   "
          f"min koşu: {args.min_run}")

    src = Path(args.raw)
    if not src.exists():
        print(f"HATA: {src} yok.", file=sys.stderr)
        return 1

    df = pd.read_parquet(src)
    cols = list(df.columns)
    if "t" not in cols:
        print("HATA: 't' kolonu yok.", file=sys.stderr)
        return 2
    feat_cols = [c for c in cols if c != "t"]
    A = df.to_numpy(np.float64)
    del df
    n0 = len(A)
    ti = cols.index("t")
    print(f"\n[1] Okundu: {n0:,} satır × {len(cols)} kolon   ({fmt(time.time()-t_wall)})")

    # ── global kararlı sıralama ──
    t_raw = A[:, ti]
    unsorted = int((np.diff(t_raw) < 0).sum())
    A = A[np.argsort(t_raw, kind="mergesort")]
    print(f"[2] Global zaman sıralaması: {unsorted:,} geriye sıçrama düzeltildi")
    print(f"    kayıt aralığı {pd.to_datetime(A[0, ti], unit='s').date()} .. "
          f"{pd.to_datetime(A[-1, ti], unit='s').date()}")

    # ── birebir yinelenen satırlar ──
    feat_idx = [cols.index(c) for c in feat_cols]
    F = A[:, feat_idx]
    keep = np.ones(len(A), bool)
    keep[1:] = ~np.all(F[1:] == F[:-1], axis=1)
    A = A[keep]
    print(f"[3] Yinelenen satır atıldı: {n0-len(A):,} (%{100*(n0-len(A))/n0:.1f}) "
          f"→ {len(A):,} satır")

    # ── koşulara böl ──
    t = A[:, ti]
    d = np.diff(t)
    Q = A[:, [cols.index(c) for c in JOINT_COLS]]
    dq = np.abs(np.diff(Q, axis=0))
    speed_ok = (dq <= (QD_MAX * d[:, None] * args.speed_margin)).all(axis=1)
    time_ok = (d > 0) & (d <= args.max_gap)
    ok = time_ok & speed_ok
    n_speed_cut = int((time_ok & ~speed_ok).sum())

    brk = np.where(~ok)[0]
    bounds = np.concatenate([[-1], brk, [len(A) - 1]])
    lens = np.diff(bounds)
    starts = bounds[:-1] + 1
    sel = lens >= args.min_run
    print(f"[4] Koşulara bölündü: {len(lens):,} koşu; "
          f"{int(sel.sum()):,} tanesi ≥ {args.min_run} örnek")
    print(f"    hız kontrolünün kestiği geçiş: {n_speed_cut:,}"
          f"{'  (sıfır → veri örnek düzeyinde tutarlı ✅)' if n_speed_cut == 0 else ''}")
    if not sel.any():
        print("HATA: hiç koşu kalmadı; --max-gap/--min-run gevşetilmeli.", file=sys.stderr)
        return 2

    # ── tekdüze ızgaraya oturt ──
    out_blocks, run_ids, interp_flags = [], [], []
    n_raw_kept = 0
    for rid, (s0, L) in enumerate(zip(starts[sel], lens[sel])):
        sl = slice(s0, s0 + L)
        ts = t[sl]
        n_grid = int(round((ts[-1] - ts[0]) / args.dt)) + 1
        grid = ts[0] + np.arange(n_grid) * args.dt
        blk = np.empty((n_grid, len(cols)))
        blk[:, ti] = grid
        for k in feat_idx:
            blk[:, k] = np.interp(grid, ts, A[sl, k])
        # ızgara noktası gerçek bir örneğe ≤ dt/2 uzaklıktaysa "ölçülmüş" sayılır
        j = np.searchsorted(ts, grid).clip(1, L - 1)
        nearest = np.minimum(np.abs(grid - ts[j]), np.abs(grid - ts[j - 1]))
        interp_flags.append(nearest > args.dt / 2)
        out_blocks.append(blk)
        run_ids.append(np.full(n_grid, rid, dtype=np.int32))
        n_raw_kept += L

    G = np.vstack(out_blocks)
    run_id = np.concatenate(run_ids)
    interp = np.concatenate(interp_flags)
    del out_blocks, run_ids, interp_flags

    out = pd.DataFrame(G, columns=cols)
    out["run_id"] = run_id
    out["interp"] = interp
    out.to_parquet(args.out, index=False)

    # ── rapor ──
    rl = np.bincount(run_id)
    n_win = int(np.maximum(0, (rl - 100) // 25 + 1).sum())
    sz = Path(args.out).stat().st_size / 1e6
    print(f"[5] Tekdüze {args.dt*1000:.0f} ms ızgaraya oturtuldu")
    print(f"    ham örnek {n_raw_kept:,} → ızgara örnek {len(G):,}  "
          f"(ara-değerlenmiş %{100*interp.mean():.1f})")
    print(f"\n{'':4}{'koşu sayısı':>14}{'medyan uzunluk':>16}{'en uzun':>10}{'toplam süre':>14}")
    print("    " + "-" * 52)
    print(f"{'':4}{len(rl):>14,}{int(np.median(rl)):>16,}{int(rl.max()):>10,}"
          f"{len(G)*args.dt/60:>13.1f}dk")
    print(f"\n  pencere (100/25, koşu içi): {n_win:,}   "
          f"[eski hat 44.974 idi ama %85,1'i kopuktu]")
    print(f"  eğitim/doğrulama tahmini  : {int(n_win*0.8):,} / {n_win-int(n_win*0.8):,}")
    print(f"\nTAMAM → {args.out}  ({len(G):,} satır, {sz:.1f} MB, {fmt(time.time()-t_wall)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
