#!/usr/bin/env python3
"""
extract_raw_features.py
=======================
ros-joint-states.csv  →  ur10e_raw_features.parquet   (27 ham kanal)

Önceki çalışmanın (PRF) ham özellik tanımını birebir uygular:

    18  eklem durumu   : 6 eklem × (position, velocity, effort)
     6  TCP kuvvet/tork: force.x/y/z, torque.x/y/z
     3  TCP konumu     : position.x/y/z
    ──
    27  kanal

Satır sırası ve chunk davranışı `generate_hybrid_residual.py` ile BİREBİR aynıdır
(CHUNKSIZE=20000, chunk içinde zaman sıralaması). Böylece üretilen parquet,
`ur10e_hybrid_residual.parquet` ile satır-satır hizalıdır ve `t` kolonları eşleşir —
iki modelin adil karşılaştırması bunu gerektiriyor.

Kullanım
--------
    python extract_raw_features.py
    python extract_raw_features.py --csv ros-joint-states.csv --out ur10e_raw_features.parquet
    python extract_raw_features.py --max-chunks 5          # hızlı deneme
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

# ─────────────── Sabitler (generate_hybrid_residual.py ile aynı) ───────────────

JNAMES = [
    "ur10e_shoulder_pan_joint",
    "ur10e_shoulder_lift_joint",
    "ur10e_elbow_joint",
    "ur10e_wrist_1_joint",
    "ur10e_wrist_2_joint",
    "ur10e_wrist_3_joint",
]
FTS_COL = "ur10e_tcp_fts_sensor"
POSE_COL = "ur10e_tcp_pose"
TIME_COLS = ["header.stamp.sec", "header.stamp.nanosec"]

CHUNKSIZE = 20_000          # generate_hybrid_residual.py ile aynı olmalı

_FLOAT = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"

_RE_JOINT = re.compile(
    r"'position':\s*(" + _FLOAT + r").*?"
    r"'velocity':\s*(" + _FLOAT + r").*?"
    r"'effort':\s*(" + _FLOAT + r")",
    re.DOTALL,
)
_RE_FTS = re.compile(
    r"'force\.x':\s*(" + _FLOAT + r").*?"
    r"'force\.y':\s*(" + _FLOAT + r").*?"
    r"'force\.z':\s*(" + _FLOAT + r").*?"
    r"'torque\.x':\s*(" + _FLOAT + r").*?"
    r"'torque\.y':\s*(" + _FLOAT + r").*?"
    r"'torque\.z':\s*(" + _FLOAT + r")",
    re.DOTALL,
)
# tcp_pose'ta anahtar sırası KARIŞIK geliyor → anahtar-bazlı arama şart
_RE_POS = {k: re.compile(r"'position\." + k + r"':\s*(" + _FLOAT + r")") for k in "xyz"}


def parse_joints(series: pd.Series):
    vals = series.to_numpy(dtype=object)
    n = len(vals)
    pos = np.zeros(n); vel = np.zeros(n); eff = np.zeros(n)
    miss = 0
    for i, cell in enumerate(vals):
        m = _RE_JOINT.search(cell if isinstance(cell, str) else str(cell))
        if m:
            pos[i] = float(m.group(1)); vel[i] = float(m.group(2)); eff[i] = float(m.group(3))
        else:
            miss += 1
    return pos, vel, eff, miss


def parse_fts(series: pd.Series):
    vals = series.to_numpy(dtype=object)
    out = np.zeros((len(vals), 6)); miss = 0
    for i, cell in enumerate(vals):
        m = _RE_FTS.search(cell if isinstance(cell, str) else str(cell))
        if m:
            for k in range(6):
                out[i, k] = float(m.group(k + 1))
        else:
            miss += 1
    return out, miss


def parse_tcp_pos(series: pd.Series):
    vals = series.to_numpy(dtype=object)
    out = np.zeros((len(vals), 3)); miss = 0
    for i, cell in enumerate(vals):
        s = cell if isinstance(cell, str) else str(cell)
        bad = False
        for k, ax in enumerate("xyz"):
            m = _RE_POS[ax].search(s)
            if m:
                out[i, k] = float(m.group(1))
            else:
                bad = True
        miss += bad
    return out, miss


def fmt(sec):
    return f"{sec:.0f}s" if sec < 60 else f"{sec/60:.1f}dk"


def main() -> int:
    ap = argparse.ArgumentParser(description="27 kanallı ham özellik çıkarımı (PRF spesifikasyonu)")
    ap.add_argument("--csv", default="ros-joint-states.csv")
    ap.add_argument("--out", default="ur10e_raw_features.parquet")
    ap.add_argument("--chunksize", type=int, default=CHUNKSIZE)
    ap.add_argument("--max-chunks", type=int, default=None)
    args = ap.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"HATA: {csv_path} yok.", file=sys.stderr)
        return 1

    print("=" * 66)
    print("HAM ÖZELLİK ÇIKARIMI — 27 kanal (18 eklem + 6 FTS + 3 TCP konum)")
    print("=" * 66)
    print(f"  girdi : {csv_path} ({csv_path.stat().st_size/1e9:.2f} GB)")
    print(f"  çıktı : {args.out}")
    print(f"  chunk : {args.chunksize}  (generate_hybrid_residual.py ile hizalı)\n")

    usecols = list(JNAMES) + [FTS_COL, POSE_COL] + TIME_COLS
    writer = None
    total = 0
    misses = 0
    t_wall = time.time()

    for ci, chunk in enumerate(pd.read_csv(csv_path, usecols=usecols,
                                           chunksize=args.chunksize, dtype=str)):
        if args.max_chunks is not None and ci >= args.max_chunks:
            print(f"\n→ --max-chunks={args.max_chunks} doldu.")
            break
        t0 = time.time()

        t = (chunk["header.stamp.sec"].astype(np.int64).to_numpy()
             + 1e-9 * chunk["header.stamp.nanosec"].astype(np.int64).to_numpy())
        order = np.argsort(t)                      # generate_hybrid_residual ile aynı
        chunk = chunk.iloc[order].reset_index(drop=True)
        t = t[order]

        n = len(chunk)
        q = np.zeros((n, 6)); qd = np.zeros((n, 6)); tau = np.zeros((n, 6))
        for j, col in enumerate(JNAMES):
            p, v, e, m = parse_joints(chunk[col])
            q[:, j], qd[:, j], tau[:, j] = p, v, e
            misses += m
        fts, m = parse_fts(chunk[FTS_COL]); misses += m
        tcp, m = parse_tcp_pos(chunk[POSE_COL]); misses += m

        out = {"t": t}
        for j in range(6):
            out[f"q_{j+1}"] = q[:, j]
        for j in range(6):
            out[f"qd_{j+1}"] = qd[:, j]
        for j in range(6):
            out[f"tau_{j+1}"] = tau[:, j]
        for k, nm in enumerate(["fx", "fy", "fz", "tx", "ty", "tz"]):
            out[nm] = fts[:, k]
        for k, nm in enumerate(["tcp_x", "tcp_y", "tcp_z"]):
            out[nm] = tcp[:, k]

        table = pa.Table.from_pandas(pd.DataFrame(out), preserve_index=False)
        if writer is None:
            writer = pq.ParquetWriter(args.out, table.schema)
        writer.write_table(table)

        total += n
        if ci % 10 == 0:
            print(f"  chunk {ci:4d} | {total:>10,} satır | {time.time()-t0:5.1f}s | "
                  f"toplam {fmt(time.time()-t_wall)}")

    if writer is not None:
        writer.close()
    print(f"\nTAMAM → {args.out}  ({total:,} satır, 27 kanal, parse hatası {misses:,}, "
          f"{fmt(time.time()-t_wall)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
