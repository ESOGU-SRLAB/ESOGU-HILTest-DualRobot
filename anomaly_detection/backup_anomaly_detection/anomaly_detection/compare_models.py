#!/usr/bin/env python3
"""
compare_models.py
=================
İki modelin `evaluation.json` dosyalarını tek bir karşılaştırma tablosuna toplar.

    python compare_models.py --raw raw_ae_model --hybrid hybrid_ae_model

Çıktı:
    model_comparison.csv   makine-okunur tablo
    model_comparison.md    rapora yapıştırılabilir tablo
    model_comparison.png   AUC/F1 çubukları + ROC + skor dağılımı üst üste
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROWS = [
    ("Girdi kanalı", "features", "{:d}"),
    ("Pencere / stride", None, None),
    ("Gizli boyut", "hidden_dim", "{:d}"),
    ("Parametre", "total_params", "{:,d}"),
    ("Eğitilen epoch", "epochs_trained", "{:d}"),
    ("En iyi val loss", "best_val_loss", "{:.6f}"),
    ("Doğrulama penceresi", "n_val_windows", "{:,d}"),
    ("Anomali oranı (etiket)", "anomaly_ratio", "{:.3f}"),
    ("AUC", "auc", "{:.4f}"),
    ("Average Precision", "average_precision", "{:.4f}"),
    ("En iyi F1", "best_f1", "{:.4f}"),
    ("  precision", "best_f1_precision", "{:.4f}"),
    ("  recall", "best_f1_recall", "{:.4f}"),
    ("Eğitilmiş eşikte F1", "f1_at_trained_threshold", "{:.4f}"),
    ("  precision", "precision_at_trained_threshold", "{:.4f}"),
    ("  recall", "recall_at_trained_threshold", "{:.4f}"),
    ("Alarm oranı", "alarm_rate_at_trained_threshold", "{:.4f}"),
]


def get(d, k, fmt):
    v = d.get(k)
    if v is None:
        return "—"
    try:
        return fmt.format(v)
    except (ValueError, TypeError):
        return str(v)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default="raw_ae_model")
    ap.add_argument("--hybrid", default="hybrid_ae_model")
    ap.add_argument("--out-prefix", default="model_comparison")
    args = ap.parse_args()

    dirs = {"raw": Path(args.raw), "hybrid": Path(args.hybrid)}
    ev, missing = {}, []
    for k, p in dirs.items():
        f = p / "evaluation.json"
        if f.exists():
            ev[k] = json.loads(f.read_text(encoding="utf-8"))
        else:
            missing.append(str(f))
    if missing:
        print("Eksik dosya(lar):", *missing, sep="\n  ", file=sys.stderr)
        print("\nÖnce evaluate_ae.py'yi her iki mod için çalıştır.", file=sys.stderr)
        if not ev:
            return 1

    keys = list(ev.keys())
    table = []
    for label, key, fmt in ROWS:
        if key is None:
            vals = [f"{ev[k]['window_size']} / {ev[k]['stride']}" for k in keys]
        else:
            vals = [get(ev[k], key, fmt) for k in keys]
        table.append([label] + vals)

    # gecikme
    for k in keys:
        lat = ev[k].get("latency", {}).get("batch1")
        if lat:
            table.append([f"Gecikme (batch=1, p50)"] +
                         [f"{ev[j].get('latency',{}).get('batch1',{}).get('ms_per_call_p50','—')} ms"
                          for j in keys])
            table.append([f"Kapasite payı (500 Hz)"] +
                         [f"{ev[j].get('headroom_batch1','—')}×" for j in keys])
            break

    # CUSUM
    if any("cusum" in ev[k] for k in keys):
        table.append(["AE+CUSUM F1"] +
                     [get(ev[k].get("cusum", {}), "f1_combined", "{:.4f}") for k in keys])
        table.append(["CUSUM ek tespit"] +
                     [get(ev[k].get("cusum", {}), "n_extra_over_ae", "{:,d}") for k in keys])

    df = pd.DataFrame(table, columns=["Metrik"] + [k.upper() for k in keys])
    df.to_csv(f"{args.out_prefix}.csv", index=False, encoding="utf-8")

    w = max(len(r[0]) for r in table) + 2
    print("=" * 70)
    print("MODEL KARŞILAŞTIRMASI")
    print("=" * 70)
    hdr = f"  {'Metrik':<{w}}" + "".join(f"{k.upper():>16}" for k in keys)
    print(hdr); print("  " + "-" * (len(hdr) - 2))
    for r in table:
        print(f"  {r[0]:<{w}}" + "".join(f"{str(v):>16}" for v in r[1:]))

    md = ["| Metrik | " + " | ".join(k.upper() for k in keys) + " |",
          "|---|" + "---|" * len(keys)]
    md += ["| " + " | ".join(str(x) for x in r) + " |" for r in table]
    Path(f"{args.out_prefix}.md").write_text("\n".join(md), encoding="utf-8")

    # figür
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from sklearn.metrics import roc_curve

        fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))
        x = np.arange(len(keys)); wdt = 0.35
        ax[0].bar(x - wdt / 2, [ev[k]["auc"] for k in keys], wdt, label="AUC")
        ax[0].bar(x + wdt / 2, [ev[k]["best_f1"] for k in keys], wdt, label="en iyi F1")
        for i, k in enumerate(keys):
            ax[0].text(i - wdt / 2, ev[k]["auc"] + .01, f"{ev[k]['auc']:.3f}",
                       ha="center", fontsize=8)
            ax[0].text(i + wdt / 2, ev[k]["best_f1"] + .01, f"{ev[k]['best_f1']:.3f}",
                       ha="center", fontsize=8)
        ax[0].set_xticks(x); ax[0].set_xticklabels([k.upper() for k in keys])
        ax[0].set_ylim(0, 1.08); ax[0].legend(); ax[0].grid(alpha=.3, axis="y")
        ax[0].set_title("AUC ve F1")

        for k in keys:
            f = dirs[k] / "eval_scores.npz"
            if not f.exists():
                continue
            z = np.load(f, allow_pickle=True)
            fpr, tpr, _ = roc_curve(z["labels"], z["scores"])
            ax[1].plot(fpr, tpr, lw=2, label=f"{k.upper()} (AUC={ev[k]['auc']:.3f})")
            s = z["scores"]
            lo, hi = np.percentile(s, [0.5, 99.5])
            ax[2].hist(np.clip(s, lo, hi), bins=70, alpha=.55, density=True, label=k.upper())
        ax[1].plot([0, 1], [0, 1], "k--", lw=.8)
        ax[1].set_xlabel("FPR"); ax[1].set_ylabel("TPR"); ax[1].set_title("ROC")
        ax[1].legend(); ax[1].grid(alpha=.3)
        ax[2].set_xlabel("anomali skoru"); ax[2].set_yscale("log")
        ax[2].set_title("Skor dağılımı"); ax[2].legend(); ax[2].grid(alpha=.3)

        fig.suptitle("Ham (27 kanal) vs Hibrit/FMU (12 artık kanalı)", fontsize=12)
        fig.tight_layout(); fig.savefig(f"{args.out_prefix}.png", dpi=140)
        print(f"\n→ {args.out_prefix}.png")
    except Exception as e:
        print(f"  (grafik atlandı: {e})")

    print(f"→ {args.out_prefix}.csv · {args.out_prefix}.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
