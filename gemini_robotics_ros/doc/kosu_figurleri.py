#!/usr/bin/env python3
"""28 Ağustos 2026 gerçek hücre koşularının figürlerini ve tablolarını üretir.

    python3 doc/kosu_figurleri.py

Veri kaynağı, recorder_node'un yazdığı kayıt klasörleridir
(recordings/gemini_robotics/<koşu>/): run_summary.txt, meta.json, events.jsonl,
er_queries.csv, arrivals.csv, surfaces.csv, scan_poses.csv. Rapor bu dosyaları
doğrudan okur; elle girilmiş sonuç yoktur.
"""
import csv
import json
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
RUNS_DIR = os.path.join(PKG, "recordings", "gemini_robotics")
FIGDIR = os.path.join(HERE, "figures")

INK = "#22252a"
OK = "#2f9e44"
BAD = "#c92a2a"
WARN = "#e8590c"
ACCENT = "#1f6feb"


def _csv(path):
    return list(csv.DictReader(open(path))) if os.path.exists(path) else []


def load_runs(runs_dir=RUNS_DIR):
    runs = []
    for name in sorted(os.listdir(runs_dir)):
        d = os.path.join(runs_dir, name)
        if not os.path.isdir(d) or not os.path.exists(os.path.join(d, "meta.json")):
            continue
        meta = json.load(open(os.path.join(d, "meta.json")))
        summary = open(os.path.join(d, "run_summary.txt"), encoding="utf-8").read() \
            if os.path.exists(os.path.join(d, "run_summary.txt")) else ""
        m = re.search(r"bitiş\s*:\s*(\S+)\s*(.*)", summary)
        outcome, detail = (m.group(1), m.group(2).strip()) if m else ("?", "")
        m = re.search(r"süre\s*:\s*([\d.]+)", summary)
        dur = float(m.group(1)) if m else float("nan")
        er = _csv(os.path.join(d, "er_queries.csv"))
        runs.append(dict(
            name=name, note=meta.get("note", ""), outcome=outcome, detail=detail,
            duration=dur, er=er, scans=_csv(os.path.join(d, "scan_poses.csv")),
            surfaces=_csv(os.path.join(d, "surfaces.csv")),
            arrivals=_csv(os.path.join(d, "arrivals.csv")),
            latency=[float(r["latency_s"]) for r in er if r.get("latency_s")],
        ))
    return runs


def run_table(runs):
    """Rapordaki koşu tablosunun satırları."""
    rows = []
    for i, r in enumerate(runs, 1):
        found = sum(1 for s in r["scans"] if s.get("found") == "True")
        rows.append([
            f"{i}", r["name"].split("_", 1)[1], r["note"][:58],
            "BAŞARILI" if r["outcome"] == "DONE" else "BAŞARISIZ",
            f"{r['duration']:.0f}", f"{len(r['er'])}",
            f"{np.mean(r['latency']):.1f}" if r["latency"] else "-",
            f"{found}/{len(r['scans'])}",
        ])
    return rows


# --------------------------------------------------------------------------- #
def fig_outcomes(runs):
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(9.6, 4.2), dpi=170,
                                  gridspec_kw=dict(width_ratios=[1.5, 1.0]))
    x = np.arange(len(runs))
    colors = [OK if r["outcome"] == "DONE" else BAD for r in runs]
    ax.bar(x, [r["duration"] for r in runs], color=colors)
    ax.set_xticks(x)
    ax.set_xticklabels([r["name"].split("_")[1] for r in runs], rotation=60,
                       ha="right", fontsize=7)
    ax.set_ylabel("koşu süresi [s]", fontsize=9)
    ax.grid(axis="y", alpha=0.3, lw=0.5)
    for xi, r in zip(x, runs):
        ax.annotate("✓" if r["outcome"] == "DONE" else "✗", (xi, r["duration"]),
                    ha="center", va="bottom", fontsize=9,
                    color=OK if r["outcome"] == "DONE" else BAD)
    ax.set_title("koşu süreleri ve sonuçları", fontsize=10, color=INK)

    # başarısızlık nedenleri
    fails = {}
    for r in runs:
        if r["outcome"] != "DONE":
            fails[r["detail"]] = fails.get(r["detail"], 0) + 1
    labels = list(fails) or ["yok"]
    ax2.barh(np.arange(len(labels)), [fails.get(l, 0) for l in labels], color=BAD)
    ax2.set_yticks(np.arange(len(labels)))
    ax2.set_yticklabels([l[:34] for l in labels], fontsize=7.5)
    ax2.invert_yaxis()
    ax2.set_xlabel("koşu sayısı", fontsize=9)
    ax2.grid(axis="x", alpha=0.3, lw=0.5)
    ax2.set_title("başarısızlık nedenleri", fontsize=10, color=INK)

    n_ok = sum(1 for r in runs if r["outcome"] == "DONE")
    fig.suptitle(f"28 Ağustos 2026 gerçek hücre koşuları — {len(runs)} deneme, "
                 f"{n_ok} tanesi görevi tamamladı", fontsize=11, color=INK)
    fig.tight_layout()
    out = os.path.join(FIGDIR, "fig_runs_outcome.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


def fig_er_latency(runs):
    all_lat, by_call = [], {}
    for r in runs:
        for row in r["er"]:
            if not row.get("latency_s"):
                continue
            v = float(row["latency_s"])
            all_lat.append(v)
            by_call.setdefault(row["call"], []).append(v)
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(9.4, 3.8), dpi=170)
    ax.hist(all_lat, bins=12, color=ACCENT, edgecolor="white")
    ax.axvline(np.mean(all_lat), color=WARN, lw=1.4)
    ax.annotate(f"ortalama {np.mean(all_lat):.1f} s", (np.mean(all_lat), 0),
                textcoords="offset points", xytext=(6, 40), fontsize=8, color=WARN)
    ax.set_xlabel("model çağrısı gecikmesi [s]", fontsize=9)
    ax.set_ylabel("çağrı sayısı", fontsize=9)
    ax.grid(axis="y", alpha=0.3, lw=0.5)
    ax.set_title(f"{len(all_lat)} ER çağrısının gecikme dağılımı", fontsize=10, color=INK)

    names = list(by_call)
    ax2.boxplot([by_call[n] for n in names], labels=names, vert=True)
    ax2.set_ylabel("gecikme [s]", fontsize=9)
    ax2.grid(axis="y", alpha=0.3, lw=0.5)
    ax2.set_title("çağrı tipine göre", fontsize=10, color=INK)
    fig.suptitle("Gemini Robotics ER 2 çağrılarının ölçülen maliyeti "
                 "(gerçek hücre, canlı model)", fontsize=11, color=INK)
    fig.tight_layout()
    out = os.path.join(FIGDIR, "fig_er_latency.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)
    return all_lat


def fig_phases(runs):
    """Temsilî bir koşunun aşama zaman çizgisi (events.jsonl'deki state geçişleri)."""
    run = next((r for r in runs if r["outcome"] == "DONE"), runs[0])
    path = os.path.join(RUNS_DIR, run["name"], "events.jsonl")
    states = []
    for line in open(path, encoding="utf-8"):
        try:
            e = json.loads(line)
        except ValueError:
            continue
        if e.get("_source") == "status" and e.get("state"):
            states.append((float(e.get("_t_rel_s", 0.0)), e["state"]))
    er_pts = [(float(r["t_rel_s"]), r["call"], float(r["latency_s"]))
              for r in run["er"] if r.get("latency_s")]
    fig, ax = plt.subplots(figsize=(9.6, 3.4), dpi=170)
    labels = []
    for i, (t, s) in enumerate(states):
        t_end = states[i + 1][0] if i + 1 < len(states) else run["duration"]
        if s not in labels:
            labels.append(s)
        y = labels.index(s)
        ax.barh(y, max(t_end - t, 0.4), left=t, height=0.62, color=ACCENT, alpha=0.85)
    for t, call, lat in er_pts:
        ax.annotate("", xy=(t, -1.0), xytext=(t, -0.4),
                    arrowprops=dict(arrowstyle="->", color=WARN, lw=1.0))
        ax.annotate(f"{call} ({lat:.0f}s)", (t, -1.5), fontsize=6.5, color=WARN,
                    rotation=90, ha="center", va="top")
    ax.set_yticks(range(len(labels)))
    ax.set_yticklabels(labels, fontsize=7)
    ax.set_ylim(-4.2, len(labels) - 0.3)
    ax.set_xlabel("koşu başından itibaren [s]", fontsize=9)
    ax.grid(axis="x", alpha=0.3, lw=0.5)
    for sp in ("top", "right", "left"):
        ax.spines[sp].set_visible(False)
    ax.set_title(f"Bir görevin aşama zaman çizgisi — {run['name']} "
                 f"({run['duration']:.0f} s, {run['note'][:40]})\n"
                 "turuncu oklar: Gemini ER 2 çağrıları", fontsize=10, color=INK)
    fig.tight_layout()
    out = os.path.join(FIGDIR, "fig_run_phases.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


def fig_surfaces(runs):
    """Düzlem oturtma kalitesi: artık RMS ve normalin eksene oturtulması."""
    rms, dev, snapped = [], [], 0
    for r in runs:
        for s in r["surfaces"]:
            if s.get("rms_residual_mm"):
                rms.append(float(s["rms_residual_mm"]))
            if s.get("normal_deviation_deg"):
                dev.append(float(s["normal_deviation_deg"]))
            if s.get("snapped") == "True":
                snapped += 1
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(9.4, 3.6), dpi=170)
    ax.hist(rms, bins=10, color=ACCENT, edgecolor="white")
    ax.set_xlabel("düzlem uydurma artığı RMS [mm]", fontsize=9)
    ax.set_ylabel("yüzey sayısı", fontsize=9)
    ax.grid(axis="y", alpha=0.3, lw=0.5)
    ax.set_title(f"{len(rms)} yüzey oturtmasının artığı", fontsize=10, color=INK)
    ax2.hist(dev, bins=10, color=WARN, edgecolor="white")
    ax2.axvline(15.0, color=BAD, ls="--", lw=1.2)
    ax2.annotate("snap toleransı 15°", (15.0, 0), textcoords="offset points",
                 xytext=(-4, 34), fontsize=7.5, color=BAD, rotation=90, ha="right")
    ax2.set_xlabel("ölçülen normalin dünya eksenine sapması [°]", fontsize=9)
    ax2.grid(axis="y", alpha=0.3, lw=0.5)
    ax2.set_title(f"{snapped}/{len(dev)} normal eksene oturtuldu", fontsize=10,
                  color=INK)
    fig.suptitle("Kavrama yüzeyinin ölçüm kalitesi (bütün koşular birlikte)",
                 fontsize=11, color=INK)
    fig.tight_layout()
    out = os.path.join(FIGDIR, "fig_surface_quality.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


def main():
    runs = load_runs()
    if not runs:
        print("kayıt bulunamadı:", RUNS_DIR)
        return
    os.makedirs(FIGDIR, exist_ok=True)
    fig_outcomes(runs)
    fig_er_latency(runs)
    fig_phases(runs)
    fig_surfaces(runs)
    for row in run_table(runs):
        print(" | ".join(row))


if __name__ == "__main__":
    main()
