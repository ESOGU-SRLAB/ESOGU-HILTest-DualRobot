#!/usr/bin/env python3
"""
aggregate_seeds.py
==================
Çok tohumlu koşuları tek tabloya indirger: ortalama ± standart sapma.

Neden gerekli
-------------
Eski hatta her iki model TEK kez, sabit tohumla eğitilmişti ve tablolardaki
0,939'a karşı 0,837 gibi farkların hiçbirinde belirsizlik yoktu. Tek koşudan
gelen bir fark, eğitim gürültüsünden mi yoksa yöntemden mi geliyor ayırt
edilemez. Bu betik tohumlar arası dağılımı verir; makalede metrikler
"ort ± std (n tohum)" biçiminde raporlanmalıdır.

Kullanım
--------
    python3 aggregate_seeds.py --pattern 'fusion_v3_s*' --out fusion_v3_summary.json
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

import numpy as np


def collect(paths: list[Path]) -> list[dict]:
    out = []
    for p in paths:
        f = p / "results.json"
        if f.exists():
            out.append(json.loads(f.read_text(encoding="utf-8")))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pattern", default="fusion_v3_s*")
    ap.add_argument("--out", default="fusion_v3_summary.json")
    args = ap.parse_args()

    runs = collect(sorted(Path(p) for p in glob.glob(args.pattern)))
    if not runs:
        print(f"HATA: {args.pattern} ile eşleşen results.json yok", file=sys.stderr)
        return 2

    print("=" * 74)
    print(f"TOHUMLAR ARASI ÖZET — n = {len(runs)}")
    print("=" * 74)
    if len(runs) == 1:
        print("  ⚠ TEK tohum. Metriklerin hiçbirinde belirsizlik yok; makalede")
        print("    bunu açıkça belirt ya da SEEDS='0 1 2 3 4' ile yeniden koş.")

    summary: dict = {"n_seeds": len(runs), "seeds": [r["seed"] for r in runs]}

    models = sorted({k for r in runs for k in r["ranking"]})
    print(f"\n  {'Model':<26}{'AUC':>17}{'PR-AUC':>17}{'BestF1':>17}")
    print("  " + "-" * 72)
    summary["ranking"] = {}
    for m in models:
        cells, row = "", {}
        for k in ("auc", "pr_auc", "best_f1"):
            v = np.array([r["ranking"][m][k] for r in runs if m in r["ranking"]])
            row[k] = {"mean": float(v.mean()), "std": float(v.std(ddof=1)) if len(v) > 1 else 0.0,
                      "values": v.tolist()}
            cells += f"{v.mean():>11.3f} ±{v.std(ddof=1) if len(v) > 1 else 0.0:<5.3f}"
        summary["ranking"][m] = row
        print(f"  {m:<26}{cells}")

    print(f"\n  {'Çalışma noktası':<26}{'kesinlik':>17}{'geri çağırma':>17}{'F1':>17}")
    print("  " + "-" * 72)
    ops = sorted({k for r in runs for k in r["operating"]})
    summary["operating"] = {}
    for m in ops:
        cells, row = "", {}
        for k in ("precision", "recall", "f1"):
            v = np.array([r["operating"][m][k] for r in runs if m in r["operating"]])
            row[k] = {"mean": float(v.mean()), "std": float(v.std(ddof=1)) if len(v) > 1 else 0.0}
            cells += f"{v.mean():>11.3f} ±{v.std(ddof=1) if len(v) > 1 else 0.0:<5.3f}"
        summary["operating"][m] = row
        print(f"  {m:<26}{cells}")

    w = np.array([r["frozen"]["w_res"] for r in runs])
    print(f"\n  dondurulan w_res : {w.mean():.3f} ± {w.std(ddof=1) if len(w) > 1 else 0.0:.3f}"
          f"   değerler {sorted(set(w.tolist()))}")
    summary["frozen"] = {"w_res": {"mean": float(w.mean()), "values": w.tolist()}}

    # Eşik, kural rejim-koşulluysa sözlüktür (duran/hareketli/global). Her alanı
    # ayrı raporlamak şart: ölçüldü ki hareketli eşik tohumlar arası kararlı
    # (bağıl std %10) ama duran eşik değil (%45) — tek bir ortalama bunu gizler.
    rules = {r["frozen"].get("rule", "global_p97") for r in runs}
    print(f"  eşik kuralı      : {', '.join(sorted(rules))}")
    thr = [r["frozen"]["threshold"] for r in runs]
    fields = (["static", "moving", "global"] if isinstance(thr[0], dict) else [None])
    summary["frozen"]["threshold"] = {}
    for f in fields:
        v = np.array([t[f] if f else t for t in thr], dtype=float)
        sd = v.std(ddof=1) if len(v) > 1 else 0.0
        rel = 100 * sd / v.mean() if v.mean() else 0.0
        print(f"  dondurulan θ {(f or 'global'):<8}: {v.mean():.4f} ± {sd:.4f}"
              f"   (bağıl std %{rel:.0f})")
        summary["frozen"]["threshold"][f or "global"] = {
            "mean": float(v.mean()), "std": float(sd), "values": v.tolist()}

    # Ağırlığın tohumlar arasında oynaması, "w=0,95 en iyi" iddiasının ne kadar
    # sağlam olduğunu doğrudan gösterir. Tek bir değere kilitleniyorsa seçim
    # kararlıdır; oynuyorsa makale bir ARALIK raporlamalıdır.
    if len(set(w.tolist())) > 1:
        print("  ⚠ ağırlık tohuma göre değişiyor → makalede tek değer değil ARALIK ver")

    d = {sp: np.array([r["drift"][sp]["test_over_train"] for r in runs])
         for sp in ("residual", "raw")}
    print(f"\n  temiz test/eğitim skor oranı — kalıntı {d['residual'].mean():.1f}x · "
          f"ham {d['raw'].mean():.1f}x   (eski hatta her ikisi de ~70x)")
    summary["drift"] = {k: {"mean": float(v.mean()), "values": v.tolist()}
                        for k, v in d.items()}

    Path(args.out).write_text(json.dumps(summary, indent=2, ensure_ascii=False),
                              encoding="utf-8")
    print(f"\n  → {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
