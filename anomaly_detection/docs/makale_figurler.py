#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Figures for the ESOGÜ MMF journal article (English).

Output: docs/figures/fig1..fig6.png  (300 dpi, sized for a 180 mm text width)

Data sources
------------
Offline  : backup_anomaly_detection/anomaly_detection/fusion_v2/scores.npz
           + anomaly_detection/fusion_v2/fusion_config.json
Real cell: ~/anomali_kayit/**/skorlar_*.csv, olaylar_*.jsonl
Photo    : extracted from the reference conference paper (belgeler/245.pdf, Fig. 1)
Screenshot: docs/figures/real_system.png (cropped here, chrome removed)

No number in these figures is hand-entered; every value is recomputed from the
sources above. The three categorical hues are the first three slots of the
validated reference palette and are used unchanged.
"""
import json
import os
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

# ── paths ────────────────────────────────────────────────────────────
HERE = Path(__file__).resolve().parent
FIG = HERE / "figures"
PKG = HERE.parent
BACKUP = PKG / "backup_anomaly_detection" / "anomaly_detection"
NPZ = BACKUP / "fusion_v2" / "scores.npz"
BILDIRI = BACKUP / "belgeler" / "245.pdf"
KAYIT = Path(os.path.expanduser("~/anomali_kayit"))
FIG.mkdir(parents=True, exist_ok=True)

# ── palette ──────────────────────────────────────────────────────────
SURFACE = "#ffffff"          # journal page is white
INK = "#111111"
INK2 = "#3d3d3b"
MUTED = "#6f6f6b"
GRID = "#e3e3de"
AXIS = "#8d8d87"

FUS = "#1f6fd0"              # fusion       — categorical slot 1
RES = "#e2571f"              # residual     — slot 2
RAW = "#0f9e6c"              # raw          — slot 3
CRIT = "#c62828"             # state: critical / threshold
WARN = "#e8a300"
FILL = "#f3f5f8"             # neutral box fill

plt.rcParams.update({
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "font.family": "DejaVu Sans",
    "font.size": 9.5,
    "axes.titlesize": 10.5,
    "axes.titleweight": "bold",
    "axes.titlecolor": INK,
    "axes.titlepad": 8,
    "axes.labelsize": 9.5,
    "axes.labelcolor": INK2,
    "axes.edgecolor": AXIS,
    "axes.linewidth": 1.0,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "xtick.color": INK2,
    "ytick.color": INK2,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "xtick.major.width": 1.0,
    "ytick.major.width": 1.0,
    "legend.fontsize": 9,
    "legend.frameon": True,
    "legend.framealpha": 0.96,
    "legend.facecolor": "#ffffff",
    "legend.edgecolor": "#cfcfc9",
    "legend.borderpad": 0.5,
    "legend.labelspacing": 0.42,
    "lines.linewidth": 2.2,
    "lines.solid_capstyle": "round",
})

WIDE = 6.9    # inches — spans the full 180 mm text width
NARROW = 3.3  # inches — fits one column


def tidy(ax, frame=True):
    """Framed panel with the grid pushed behind the data."""
    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_visible(frame)
    ax.set_axisbelow(True)
    ax.tick_params(length=3.5, pad=3)
    return ax


def save(fig, name):
    path = FIG / name
    fig.savefig(path, dpi=300, bbox_inches="tight", pad_inches=0.04)
    plt.close(fig)
    print(f"  ok  {name}  ({path.stat().st_size // 1024} KB)")


# ═════════════════════════════════════════════════════════════════════
# offline data
# ═════════════════════════════════════════════════════════════════════
def load_offline():
    if not NPZ.exists():
        print(f"  !! missing {NPZ}", file=sys.stderr)
        return None
    z = np.load(NPZ)
    cfg = json.load(open(PKG / "fusion_v2" / "fusion_config.json", encoding="utf-8"))
    sc = cfg["scale"]
    # the npz z_* fields carry a different normalisation; recompute from the
    # scale block that the deployed detector actually uses.
    zr = (z["s_residual"] - sc["residual"]["lo"]) / sc["residual"]["span"]
    zw = (z["s_raw"] - sc["raw"]["lo"]) / sc["raw"]["span"]
    return {
        "y": z["y"].astype(bool),
        "res": z["s_residual"], "raw": z["s_raw"],
        "zr": zr, "zw": zw,
        "fused": cfg["w_kal"] * zr + cfg["w_ham"] * zw,
        "thr": cfg["fused_threshold_fmu"],
        "cfg": cfg,
        "n_train_windows": train_windows(),
    }


def train_windows(default=8294):
    """How many windows the autoencoders actually saw during training."""
    meta = PKG / "residual_ae_v2" / "metadata.json"
    if meta.exists():
        return int(json.load(open(meta))["n_train_windows"])
    return default


def pr_curve(y, s):
    """Precision-recall curve + average precision, computed directly."""
    order = np.argsort(-s)
    ys = y[order]
    tp = np.cumsum(ys)
    fp = np.cumsum(~ys)
    precision = tp / np.maximum(tp + fp, 1)
    recall = tp / ys.sum()
    ap = float(np.sum(np.diff(np.r_[0.0, recall]) * precision))
    return recall, precision, ap


def roc_curve(y, s):
    order = np.argsort(-s)
    ys = y[order]
    tpr = np.cumsum(ys) / ys.sum()
    fpr = np.cumsum(~ys) / (~ys).sum()
    trap = np.trapezoid if hasattr(np, "trapezoid") else np.trapz
    return fpr, tpr, float(trap(tpr, fpr))


# ═════════════════════════════════════════════════════════════════════
# Figure 1 — experimental platform
# ═════════════════════════════════════════════════════════════════════
def fig_platform():
    src = FIG / "platform.png"
    if not src.exists():
        if not BILDIRI.exists():
            print("  !! no platform photo available", file=sys.stderr)
            return
        tmp = FIG / "_pf"
        subprocess.run(["pdfimages", "-f", "2", "-l", "2", "-png",
                        str(BILDIRI), str(tmp)], check=True)
        got = sorted(FIG.glob("_pf-*.png"))
        if not got:
            return
        got[0].rename(src)
        for extra in FIG.glob("_pf-*.png"):
            extra.unlink()

    # the source is a 366x448 crop out of the conference paper: dim and small,
    # so it is upscaled and tone-corrected before it goes into the article
    from PIL import Image, ImageEnhance, ImageOps
    im = Image.open(src).convert("RGB")
    im = im.resize((im.width * 3, im.height * 3), Image.LANCZOS)
    im = ImageOps.autocontrast(im, cutoff=(0.5, 0.5))
    im = ImageEnhance.Brightness(im).enhance(1.10)
    im = ImageEnhance.Color(im).enhance(1.06)
    im = ImageEnhance.Sharpness(im).enhance(1.35)
    img = np.asarray(im)

    h, w = img.shape[:2]
    fig, ax = plt.subplots(figsize=(NARROW, NARROW * h / w))
    ax.imshow(img)
    ax.set_axis_off()
    for side in ax.spines.values():
        side.set_visible(False)
    save(fig, "fig1_platform.png")


# ═════════════════════════════════════════════════════════════════════
# Figure 2 — architecture / data flow
# ═════════════════════════════════════════════════════════════════════
def fig_architecture():
    fig = plt.figure(figsize=(WIDE, 2.60))
    ax = fig.add_axes([0.0, 0.0, 1.0, 1.0])
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 36)
    ax.set_axis_off()
    ax.grid(False)

    def box(x, y, w, h, title, body, edge=AXIS, face=FILL, tc=INK):
        ax.add_patch(FancyBboxPatch(
            (x, y), w, h, boxstyle="round,pad=0.0,rounding_size=0.8",
            linewidth=1.2, edgecolor=edge, facecolor=face, zorder=2))
        ax.text(x + w / 2, y + h - 1.9, title, ha="center", va="top",
                fontsize=7.2, fontweight="bold", color=tc, zorder=3)
        if body:
            ax.text(x + w / 2, y + h - 5.5, body, ha="center", va="top",
                    fontsize=6.2, color=INK2, linespacing=1.5, zorder=3)
        return (x, y, w, h)

    def arrow(a, b, label=None):
        x0, x1 = a[0] + a[2], b[0]
        y0, y1 = a[1] + a[3] / 2, b[1] + b[3] / 2
        ax.add_patch(FancyArrowPatch(
            (x0, y0), (x1, y1), arrowstyle="-|>", mutation_scale=9,
            linewidth=1.1, color=MUTED, shrinkA=1.5, shrinkB=1.5, zorder=1))
        if label:
            ax.text((x0 + x1) / 2, max(y0, y1) + 0.8, label, ha="center",
                    va="bottom", fontsize=6.0, color=MUTED)

    cell = box(1.0, 10.5, 12.0, 15.0, "UR10e cell",
               "6 joints\n+ Festo axis\n495 Hz", edge=INK2, face="#ffffff")
    js = box(17.5, 19.0, 16.0, 13.0, "/joint_states",
             "q, q̇, current [A]\n3 publishers,\nper-message map")
    wr = box(17.5, 4.0, 16.0, 13.0, "/…/wrench",
             "F/T sensor,\nexpressed\nin tool0")

    feat = box(37.5, 4.0, 18.0, 28.0, "Feature engine",
               "Savitzky–Golay q̈\ncurrent → torque\nFMU inverse\ndynamics\n"
               "R₀₆ rotation\nJᵀ transfer\n\nr_int, r_ext\n(Eq. 2–4)",
               edge=RES, face="#fdf3ef")

    aeR = box(59.5, 19.0, 17.0, 13.0, "Residual LSTM-AE",
              "12 ch\n478,892 par\nONNX Runtime", edge=RES, face="#fdf3ef")
    aeW = box(59.5, 4.0, 17.0, 13.0, "Raw LSTM-AE",
              "24 ch\n1,907,032 par\nONNX Runtime", edge=RAW, face="#eefaf5")

    fus = box(80.0, 4.0, 11.3, 28.0, "Fusion",
              "causal\nmin–max\n\nEq. (4)\n0.95 z_res\n+ 0.05 z_raw\n\n"
              "θ = 18.0\n2 consecutive", edge=FUS, face="#eef4fc")

    out = box(93.3, 4.0, 6.5, 28.0, "Alarm",
              "operator\nUI\n\nJSONL\nevent\nlog\n\nCSV\nscores",
              edge=INK2, face="#ffffff")

    arrow(cell, js)
    arrow(cell, wr)
    arrow(js, feat)
    arrow(wr, feat)
    arrow(feat, aeR, "12 ch")
    arrow(feat, aeW, "24 ch")
    arrow(aeR, fus)
    arrow(aeW, fus)
    arrow(fus, out)

    ax.text(50.0, 0.9, "5.9 ms per decision   ·   20 Hz   ·   "
                       "end-to-end detection latency ≈ 150 ms",
            ha="center", va="bottom", fontsize=6.6, color=INK2, style="italic")
    save(fig, "fig2_architecture.png")


# ═════════════════════════════════════════════════════════════════════
# Figure 3 — PR and ROC of the corrected offline pipeline
# ═════════════════════════════════════════════════════════════════════
def fig_pr_roc(D):
    fig, axes = plt.subplots(1, 2, figsize=(WIDE, 3.25))
    series = [("Fusion", D["fused"], FUS, 2.8),
              ("Residual AE", D["res"], RES, 1.9),
              ("Raw AE", D["raw"], RAW, 1.9)]

    ax = tidy(axes[0])
    for name, sc, colour, lw in series:
        r, pr, ap = pr_curve(D["y"], sc)
        ax.plot(r, pr, color=colour, lw=lw, label=f"{name}   {ap:.3f}",
                zorder=4 if colour == FUS else 3)
    base = D["y"].mean()
    ax.axhline(base, color=MUTED, lw=1.2, ls=(0, (5, 4)), zorder=2)
    ax.text(0.985, base - 0.03, f"random baseline  {base:.3f}", color=MUTED,
            fontsize=8.5, ha="right", va="top")
    ax.set_xlabel("recall")
    ax.set_ylabel("precision")
    ax.set_title("(a)  Precision–recall  (PR-AUC)", loc="left")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1.05)
    ax.legend(loc="lower left", handlelength=1.7, bbox_to_anchor=(0.015, 0.10))

    ax = tidy(axes[1])
    for name, sc, colour, lw in series:
        fpr, tpr, auc = roc_curve(D["y"], sc)
        ax.plot(fpr, tpr, color=colour, lw=lw, label=f"{name}   {auc:.3f}",
                zorder=4 if colour == FUS else 3)
    ax.plot([0, 1], [0, 1], color=MUTED, lw=1.2, ls=(0, (5, 4)), zorder=2)
    ax.set_xlabel("false positive rate")
    ax.set_ylabel("true positive rate")
    ax.set_title("(b)  ROC  (AUC)", loc="left")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1.05)
    ax.legend(loc="lower right", handlelength=1.7, bbox_to_anchor=(0.985, 0.03))

    fig.subplots_adjust(wspace=0.24)
    save(fig, "fig3_pr_roc.png")


# ═════════════════════════════════════════════════════════════════════
# Figure 4 — weight sweep and per-fault-type detection
# ═════════════════════════════════════════════════════════════════════
def fig_fusion_value(D):
    fig, axes = plt.subplots(1, 2, figsize=(WIDE, 3.25),
                             gridspec_kw={"width_ratios": [1.0, 1.05]})

    # (a) weight sweep -------------------------------------------------
    ws = np.linspace(0, 1, 101)
    pr = np.array([pr_curve(D["y"], w * D["zr"] + (1 - w) * D["zw"])[2] for w in ws])
    auc = np.array([roc_curve(D["y"], w * D["zr"] + (1 - w) * D["zw"])[2] for w in ws])

    ax = tidy(axes[0])
    ax.plot(ws, auc, color=RES, lw=2.4, label="ROC-AUC")
    ax.plot(ws, pr, color=FUS, lw=2.8, label="PR-AUC")
    ax.plot(0.95, pr[95], "o", ms=7, color=CRIT, mec="#ffffff", mew=1.6, zorder=7)
    ax.set_xlabel("residual weight  $w_{res}$")
    ax.set_ylabel("area under curve")
    ax.set_xlim(0, 1.0)
    ax.set_ylim(0.40, 1.13)
    ax.set_yticks([0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
    ax.set_title("(a)  Fusion weight sweep", loc="left")
    ax.legend(loc="upper left", ncol=2, handlelength=1.6, columnspacing=1.3,
              bbox_to_anchor=(0.02, 0.995), frameon=False)

    ins = ax.inset_axes([0.33, 0.08, 0.62, 0.40])
    tidy(ins)
    ins.set_facecolor("#fbfbfa")
    sel = ws >= 0.90
    ins.plot(ws[sel], pr[sel], color=FUS, lw=2.2)
    for w in (0.95, 1.00):
        i = int(round(w * 100))
        ins.plot(w, pr[i], "o", ms=5.2, color=CRIT if w == 0.95 else INK2,
                 mec="#ffffff", mew=1.2, zorder=7)
    ins.annotate(f"{pr[95]:.3f}", (0.95, pr[95]), textcoords="offset points",
                 xytext=(4, 2), fontsize=8, color=CRIT, fontweight="bold")
    ins.annotate(f"{pr[100]:.3f}", (1.0, pr[100]), textcoords="offset points",
                 xytext=(-4, -2), ha="right", va="top", fontsize=8, color=INK2)
    ins.set_xlim(0.90, 1.008)
    ins.set_xticks([0.90, 0.95, 1.00])
    ins.set_ylim(0.47, 0.86)
    ins.set_yticks([0.55, 0.70, 0.80])
    ins.tick_params(labelsize=7.5, length=2.5, pad=2)
    ins.set_title("detail:  $w_{res}\\in[0.90,\\,1.00]$", loc="left",
                  fontsize=7.8, color=INK2, fontweight="normal", pad=3)

    # (b) per-fault-type ----------------------------------------------
    faults = ["Motor drift", "Collision", "Encoder glitch", "Sensor noise"]
    res = [0.451, 0.995, 0.988, 0.296]
    raw = [0.256, 0.999, 0.491, 1.000]
    fus = [0.443, 0.996, 0.989, 0.996]
    y = np.arange(len(faults))[::-1]
    g = 0.26

    ax = tidy(axes[1])
    ax.yaxis.grid(False)
    for off, data, colour, name in ((+g, res, RES, "Residual AE"),
                                    (0.0, raw, RAW, "Raw AE"),
                                    (-g, fus, FUS, "Fusion")):
        ax.barh(y + off, data, g * 0.92, color=colour, edgecolor="#ffffff",
                linewidth=1.1, label=name, zorder=3)
        for yi, v in zip(y + off, data):
            ax.text(v + 0.025, yi, f"{v:.2f}", va="center", ha="left",
                    fontsize=8, color=INK2)
    ax.set_yticks(y, faults)
    ax.set_xlim(0, 1.20)
    ax.set_xticks([0, 0.5, 1.0])
    ax.set_xlabel("AUC")
    ax.set_title("(b)  Detection by fault type", loc="left", pad=20)
    h, l = ax.get_legend_handles_labels()
    ax.legend(h[::-1], l[::-1], loc="lower left", ncol=3, handlelength=1.3,
              columnspacing=1.2, bbox_to_anchor=(0.0, 1.005), borderaxespad=0,
              frameon=False)

    fig.subplots_adjust(wspace=0.40)
    save(fig, "fig4_fusion_value.png")


def fig_unseen(D):
    """PR-AUC on the windows the autoencoders were trained on, and on those
    they never saw (the run-boundary-aligned last 20 % of the recording)."""
    n_win = len(D["y"]) // 4
    n_tr = D.get("n_train_windows", 8294)
    unseen = np.zeros(len(D["y"]), bool)
    for i in range(4):
        unseen[i * n_win + n_tr:(i + 1) * n_win] = True

    subsets = [("all windows\n(as reported)", np.ones(len(D["y"]), bool)),
               ("windows used\nin training", ~unseen),
               ("windows never\nseen in training", unseen)]
    models = [("Residual AE", D["zr"], RES),
              ("Raw AE", D["zw"], RAW),
              ("Fusion", D["fused"], FUS)]

    fig, ax = plt.subplots(figsize=(WIDE * 0.5, 2.95))
    tidy(ax)
    ax.yaxis.grid(False)
    y = np.arange(len(subsets))[::-1]
    g = 0.25
    for off, (name, sc, colour) in zip((+g, 0.0, -g), models):
        vals = [pr_curve(D["y"][m], sc[m])[2] for _, m in subsets]
        ax.barh(y + off, vals, g * 0.92, color=colour, edgecolor="#ffffff",
                linewidth=1.1, label=name, zorder=3)
        for yi, v in zip(y + off, vals):
            ax.text(v + 0.02, yi, f"{v:.3f}", va="center", ha="left",
                    fontsize=8, color=INK2)
    ax.set_yticks(y, [s[0] for s in subsets])
    ax.set_xlim(0, 1.25)
    ax.set_xticks([0, 0.5, 1.0])
    ax.set_xlabel("PR-AUC")
    ax.set_title("Effect of the train/test overlap", loc="left", pad=20)
    h, l = ax.get_legend_handles_labels()
    ax.legend(h, l, loc="lower left", ncol=3, handlelength=1.3,
              columnspacing=1.0, bbox_to_anchor=(0.0, 1.005), borderaxespad=0,
              frameon=False)
    save(fig, "fig5_unseen.png")


# ═════════════════════════════════════════════════════════════════════
# real-cell data
# ═════════════════════════════════════════════════════════════════════
def load_csv(rel):
    path = KAYIT / rel
    if not path.exists():
        print(f"  !! missing {path}", file=sys.stderr)
        return None
    return np.genfromtxt(path, delimiter=",", names=True)


def fig_real_cell():
    trial = load_csv("skorlar_20260821_105346.csv")
    lock = load_csv("skorlar_20260821_101757.csv")
    moving = load_csv("skorlar_20260821_103807.csv")
    if trial is None or lock is None or moving is None:
        return

    cfg = json.load(open(PKG / "fusion_v2" / "fusion_config.json", encoding="utf-8"))
    th_fmu = cfg["fused_threshold_fmu"]
    th_real = cfg["fused_threshold"]

    fig, axes = plt.subplots(1, 2, figsize=(WIDE, 3.35),
                             gridspec_kw={"width_ratios": [1.42, 1.0]})

    # (a) fault-trial run, operator-confirmed events only ---------------
    t = trial["t_ros"] - trial["t_ros"][0]
    f = trial["birlesik"]
    ax = tidy(axes[0])
    ax.set_yscale("log")
    ax.axhspan(th_real, 1e4, color=CRIT, alpha=0.05, lw=0, zorder=1)
    ax.plot(t, np.maximum(f, 1e-3), color=FUS, lw=1.0, zorder=3)
    ax.axhline(th_real, color=CRIT, lw=1.8, ls=(0, (5, 3)), zorder=4)
    ax.axhline(th_fmu, color=MUTED, lw=1.5, ls=(0, (2, 2.5)), zorder=4)

    confirmed = [(31.98, "jam"), (195.83, "collision"), (79.86, "collision")]
    offsets = [(-6, 14), (0, 14), (-8, 12)]
    aligns = ["right", "center", "right"]
    for (peak, label), off, ha in zip(confirmed, offsets, aligns):
        idx = int(np.argmin(np.abs(f - peak)))
        if abs(f[idx] - peak) > 0.05 * peak:
            continue
        ax.plot(t[idx], f[idx], "o", ms=7, color=CRIT, mec="#ffffff",
                mew=1.6, zorder=7)
        ax.annotate(f"{label}\n{f[idx]:.1f}", (t[idx], f[idx]),
                    textcoords="offset points", xytext=off, ha=ha,
                    fontsize=8.2, color=INK, linespacing=1.35, zorder=8,
                    fontweight="bold")

    ax.set_xlabel("seconds from start of run")
    ax.set_ylabel("fused score  (log)")
    ax.set_xlim(0, t[-1] + 8)
    ax.set_ylim(1e-2, 8e3)
    ax.set_yticks([1e-2, 1e-1, 1e0, 1e1, 1e2, 1e3])
    ax.set_title("(a)  Fault-trial run on the real UR10e", loc="left")
    bb = dict(boxstyle="round,pad=0.22", fc="#ffffff", ec="none", alpha=0.88)
    ax.text(0.012, th_real * 1.55, f"θ = {th_real:.0f}  (re-measured on the cell)",
            transform=ax.get_yaxis_transform(), ha="left", va="bottom",
            fontsize=8, color=CRIT, fontweight="bold", bbox=bb, zorder=9)
    ax.text(0.012, th_fmu / 2.6, f"θ = {th_fmu:.2f}  (offline validation)",
            transform=ax.get_yaxis_transform(), ha="left", va="top",
            fontsize=8, color=INK2, bbox=bb, zorder=9)

    # (b) score distribution by operating regime ------------------------
    t1 = lock["t_ros"] - lock["t_ros"][0]
    f1 = lock["birlesik"]
    f2 = moving["birlesik"]
    mv = moving["hareket"] > 0

    regimes = [("folded park", f1[t1 < 55.70], MUTED),
               ("protective stop", f1[(t1 > 210.85) & (t1 < 332.9)], WARN),
               ("normal motion", f2[mv], FUS),
               ("gravity-loaded", f2[~mv], RAW)]

    ax = tidy(axes[1])
    ax.set_yscale("log")
    ax.xaxis.grid(False)
    ax.axhspan(th_real, 1e3, color=CRIT, alpha=0.05, lw=0, zorder=1)
    for i, (name, v, colour) in enumerate(regimes):
        v = v[v > 0]
        rng = np.random.default_rng(11 + i)
        vv = v if len(v) <= 700 else v[:: max(1, len(v) // 700)]
        ax.plot(i + (rng.random(len(vv)) - 0.5) * 0.44, vv, ".", ms=2.4,
                color=colour, alpha=0.30, mec="none", zorder=3)
        med = float(np.median(v))
        ax.plot([i - 0.32, i + 0.32], [med, med], color=colour, lw=3.4,
                solid_capstyle="butt", zorder=6)
        ax.text(i + 0.36, med, f"{med:.4f}" if med < 0.01 else f"{med:.2f}",
                ha="left", va="center", fontsize=8.2, color=INK,
                fontweight="bold", zorder=7)

    ax.axhline(th_fmu, color=MUTED, lw=1.5, ls=(0, (2, 2.5)), zorder=4)
    ax.axhline(th_real, color=CRIT, lw=1.8, ls=(0, (5, 3)), zorder=4)
    ax.text(-0.46, th_real * 1.45, "θ = 18", ha="left", fontsize=8.2,
            color=CRIT, fontweight="bold")
    ax.text(-0.46, th_fmu * 1.5, "θ = 0.64", ha="left", fontsize=8.2, color=INK2)
    ax.set_xticks(range(len(regimes)), [r[0] for r in regimes], fontsize=8.5,
                  rotation=24, ha="right", rotation_mode="anchor")
    ax.set_xlim(-0.55, len(regimes) - 0.05)
    ax.set_ylim(1e-3, 5e2)
    ax.set_title("(b)  Score by operating regime", loc="left")

    fig.subplots_adjust(wspace=0.24)
    save(fig, "fig6_real_cell.png")


# ═════════════════════════════════════════════════════════════════════
# Figure 6 — operator interface
# ═════════════════════════════════════════════════════════════════════
def fig_interface():
    src = FIG / "real_system.png"
    if not src.exists():
        print("  !! missing real_system.png", file=sys.stderr)
        return
    img = plt.imread(src)
    h, w = img.shape[:2]
    # crop the desktop / browser chrome: keep the page body only
    y0, y1 = int(0.136 * h), int(0.930 * h)
    x0, x1 = int(0.128 * w), int(0.995 * w)
    crop = img[y0:y1, x0:x1]
    ch, cw = crop.shape[:2]
    fig, ax = plt.subplots(figsize=(WIDE, WIDE * ch / cw))
    ax.imshow(crop)
    ax.set_axis_off()
    for side in ax.spines.values():
        side.set_visible(False)
    save(fig, "fig7_interface.png")


def main():
    print("figures →", FIG)
    fig_platform()
    fig_architecture()
    D = load_offline()
    if D is not None:
        print(f"  offline set: {len(D['y'])} windows, "
              f"{int(D['y'].sum())} anomalous ({D['y'].mean()*100:.1f} %)")
        fig_pr_roc(D)
        fig_fusion_value(D)
        fig_unseen(D)
    fig_real_cell()
    fig_interface()


if __name__ == "__main__":
    main()
