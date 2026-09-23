#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR10e anomali tespiti — rapor şekilleri üreteci.

Çıktı: docs/figures/*.png (200 dpi, 16 cm sayfa genişliğine göre ölçülü)

Veri kaynakları
---------------
Çevrimdışı (Bölüm 6)  : fusion_v2/scores.npz + fusion_v2/fusion_config.json
Gerçek robot (Bölüm 10): ~/anomali_kayit/**/skorlar_*.csv, olaylar_*.jsonl

Renkler, `dataviz` referans paletinin ilk üç kategorik yuvası ve sabit durum
renkleridir; ilk üç yuva her iki kipte de tüm-çift doğrulamasını geçtiği için
(CVD ΔE 9,2 / normal görüş ΔE 24,0, açık zemin) değiştirilmeden kullanılmıştır.
"""
import json
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FuncFormatter

# ── yollar ───────────────────────────────────────────────────────────
HERE = Path(__file__).resolve().parent
FIG = HERE / "figures"
PKG = HERE.parent
KAYIT = Path(os.path.expanduser("~/anomali_kayit"))
NPZ = Path(os.path.expanduser(
    "~/Desktop/backup_anomaly_detection/anomaly_detection/fusion_v2/scores.npz"))
FIG.mkdir(parents=True, exist_ok=True)

# ── palet (dataviz referans örneği, açık kip) ────────────────────────
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"

BIR = "#2a78d6"   # birleşim   — kategorik yuva 1
KAL = "#eb6834"   # kalıntı    — yuva 2
HAM = "#1baf7a"   # ham        — yuva 3
KRITIK = "#d03b3b"   # durum: kritik (eşik / alarm)
IYI = "#0ca30c"      # durum: iyi
UYARI = "#fab219"    # durum: uyarı

plt.rcParams.update({
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "font.family": "DejaVu Sans",
    "font.size": 8.5,
    "axes.titlesize": 9.5,
    "axes.titleweight": "bold",
    "axes.titlecolor": INK,
    "axes.labelsize": 8.5,
    "axes.labelcolor": INK2,
    "axes.edgecolor": AXIS,
    "axes.linewidth": 0.8,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.6,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "legend.fontsize": 8,
    "legend.frameon": False,
    "lines.linewidth": 1.8,
})


def tr(v, basamak=3):
    """Türkçe ondalık ayırıcı ile sayı."""
    return f"{v:.{basamak}f}".replace(".", ",")


def tr_eksen(ax, eksen="both", basamak=1):
    """Eksen etiketlerinde nokta yerine virgül."""
    fmt = FuncFormatter(lambda v, _: f"{v:.{basamak}f}".replace(".", ","))
    if eksen in ("x", "both"):
        ax.xaxis.set_major_formatter(fmt)
    if eksen in ("y", "both"):
        ax.yaxis.set_major_formatter(fmt)


def duzenle(ax):
    """Üst/sağ çerçeveyi kaldır, ızgarayı geriye it."""
    for yan in ("top", "right"):
        ax.spines[yan].set_visible(False)
    ax.set_axisbelow(True)
    return ax


def kaydet(fig, ad):
    yol = FIG / ad
    fig.savefig(yol, dpi=200, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print(f"  ✓ {ad}  ({yol.stat().st_size // 1024} KB)")


# ═════════════════════════════════════════════════════════════════════
# ÇEVRİMDIŞI VERİ (Bölüm 6)
# ═════════════════════════════════════════════════════════════════════
def cevrimdisi_yukle():
    if not NPZ.exists():
        return None
    z = np.load(NPZ)
    cfg = json.load(open(PKG / "fusion_v2" / "fusion_config.json", encoding="utf-8"))
    sc = cfg["scale"]
    zk = (z["s_residual"] - sc["residual"]["lo"]) / sc["residual"]["span"]
    zh = (z["s_raw"] - sc["raw"]["lo"]) / sc["raw"]["span"]
    return {
        "y": z["y"],
        "kal": z["s_residual"], "ham": z["s_raw"],
        "zk": zk, "zh": zh,
        "bir": cfg["w_kal"] * zk + cfg["w_ham"] * zh,
        "thr": cfg["fused_threshold_fmu"],
        "cfg": cfg,
    }


def pr_egrisi(y, s):
    """Elle PR eğrisi + ortalama kesinlik (sklearn'e bağımlı kalmamak için)."""
    o = np.argsort(-s)
    ys = y[o]
    tp = np.cumsum(ys)
    fp = np.cumsum(~ys)
    P = ys.sum()
    kesinlik = tp / np.maximum(tp + fp, 1)
    duyarlilik = tp / P
    # ortalama kesinlik = Σ (R_n − R_{n−1}) · P_n
    ap = float(np.sum(np.diff(np.r_[0.0, duyarlilik]) * kesinlik))
    return duyarlilik, kesinlik, ap


def roc_egrisi(y, s):
    o = np.argsort(-s)
    ys = y[o]
    tpr = np.cumsum(ys) / ys.sum()
    fpr = np.cumsum(~ys) / (~ys).sum()
    auc = float(np.trapezoid(tpr, fpr)) if hasattr(np, "trapezoid") \
        else float(np.trapz(tpr, fpr))
    return fpr, tpr, auc


def sekil_pr_roc(D):
    fig, axes = plt.subplots(1, 2, figsize=(6.3, 2.9))
    seri = [("Birleşim (0,95/0,05)", D["bir"], BIR, 2.1),
            ("Kalıntı ÖzK.", D["kal"], KAL, 1.5),
            ("Ham ÖzK.", D["ham"], HAM, 1.5)]

    ax = duzenle(axes[0])
    for ad, s, renk, lw in seri:
        r, p, ap = pr_egrisi(D["y"], s)
        ax.plot(r, p, color=renk, lw=lw, label=f"{ad}  ({tr(ap)})")
    taban = D["y"].mean()
    ax.axhline(taban, color=MUTED, lw=0.9, ls=(0, (4, 3)))
    ax.text(0.02, taban + 0.025, f"rastgele  {tr(taban)}", color=MUTED,
            fontsize=7, ha="left", va="bottom")
    ax.set_xlabel("duyarlılık (recall)")
    ax.set_ylabel("kesinlik (precision)")
    ax.set_title("Kesinlik–duyarlılık  (PR-AUC)", loc="left")
    ax.set_xlim(0, 1); ax.set_ylim(0, 1.08)
    tr_eksen(ax)
    ax.legend(loc="lower left", handlelength=1.6,
              bbox_to_anchor=(0.0, 0.16))

    ax = duzenle(axes[1])
    for ad, s, renk, lw in seri:
        fpr, tpr, auc = roc_egrisi(D["y"], s)
        ax.plot(fpr, tpr, color=renk, lw=lw, label=f"{ad}  ({tr(auc)})")
    ax.plot([0, 1], [0, 1], color=MUTED, lw=0.9, ls=(0, (4, 3)))
    ax.set_xlabel("yanlış pozitif oranı")
    ax.set_ylabel("doğru pozitif oranı")
    ax.set_title("ROC  (AUC)", loc="left")
    ax.set_xlim(0, 1); ax.set_ylim(0, 1.08)
    tr_eksen(ax)
    ax.legend(loc="lower right", handlelength=1.6)

    kaydet(fig, "sekil_pr_roc.png")


def sekil_karisiklik(D):
    y = D["y"]; pred = D["bir"] >= D["thr"]
    TP = int((pred & y).sum()); FP = int((pred & ~y).sum())
    FN = int((~pred & y).sum()); TN = int((~pred & ~y).sum())
    M = np.array([[TN, FP], [FN, TP]], float)
    satir = M / M.sum(axis=1, keepdims=True)          # satır normalleştirmesi

    fig, (ax, axb) = plt.subplots(
        1, 2, figsize=(6.3, 2.6), gridspec_kw={"width_ratios": [1.05, 1.0]})

    ax.grid(False)
    ax.imshow(satir, cmap="Blues", vmin=0, vmax=1)
    etk = [["DN\n(doğru negatif)", "YP\n(yanlış pozitif)"],
           ["YN\n(yanlış negatif)", "DP\n(doğru pozitif)"]]
    for i in range(2):
        for j in range(2):
            koyu = satir[i, j] > 0.55
            ax.text(j, i - 0.22, etk[i][j], ha="center", va="center", fontsize=7.5,
                    color=("#ffffff" if koyu else INK2), linespacing=1.25)
            ax.text(j, i + 0.22, f"{int(M[i, j]):,}".replace(",", ".")
                    + f"\n%{tr(satir[i, j]*100, 1)}", ha="center", va="center",
                    fontsize=9, fontweight="bold",
                    color=("#ffffff" if koyu else INK), linespacing=1.25)
    ax.set_xticks([0, 1], ["normal", "anomali"])
    ax.set_yticks([0, 1], ["normal", "anomali"])
    ax.set_xlabel("tahmin"); ax.set_ylabel("gerçek etiket")
    ax.set_title(f"Karışıklık matrisi  (θ = {tr(D['thr'], 4)})", loc="left")
    for yan in ("top", "right", "bottom", "left"):
        ax.spines[yan].set_visible(False)
    ax.tick_params(length=0)

    # yan panel: türetilmiş ölçütler
    duzenle(axb)
    kesinlik = TP / (TP + FP); duyarlilik = TP / (TP + FN)
    f1 = 2 * kesinlik * duyarlilik / (kesinlik + duyarlilik)
    ozgulluk = TN / (TN + FP)
    adlar = ["kesinlik", "duyarlılık", "F1", "özgüllük"]
    deger = [kesinlik, duyarlilik, f1, ozgulluk]
    yy = np.arange(len(adlar))[::-1]
    axb.barh(yy, deger, height=0.42, color=BIR, edgecolor=SURFACE, linewidth=1.2)
    for v, y_ in zip(deger, yy):
        axb.text(v + 0.02, y_, tr(v), va="center",
                 fontsize=8.5, color=INK, fontweight="bold")
    axb.set_yticks(yy, adlar)
    axb.set_xlim(0, 1.14); axb.set_xticks([0, 0.5, 1.0]); tr_eksen(axb, "x")
    axb.xaxis.grid(True); axb.yaxis.grid(False)
    axb.set_title("Çalışma noktası ölçütleri", loc="left")
    kaydet(fig, "sekil_karisiklik.png")
    return TP, FP, FN, TN


def sekil_agirlik(D):
    ws = np.linspace(0, 1, 101)
    pr, auc = [], []
    for w in ws:
        s = w * D["zk"] + (1 - w) * D["zh"]
        pr.append(pr_egrisi(D["y"], s)[2])
        auc.append(roc_egrisi(D["y"], s)[2])
    pr = np.array(pr); auc = np.array(auc)

    fig, ax = plt.subplots(figsize=(6.3, 2.6))
    duzenle(ax)
    ax.plot(ws, pr, color=BIR, label="PR-AUC")
    ax.plot(ws, auc, color=KAL, label="ROC-AUC")
    ax.axvline(0.95, color=KRITIK, lw=1.3, ls=(0, (4, 3)))
    ax.plot(0.95, pr[95], "o", ms=6, color=KRITIK, mec=SURFACE, mew=1.3, zorder=6)
    ax.annotate(f"bildirinin ağırlığı  w_kal = 0,95\nPR-AUC {tr(pr[95])}",
                (0.95, pr[95]), textcoords="offset points", xytext=(-10, 16),
                ha="right", fontsize=7.5, color=KRITIK, fontweight="bold",
                arrowprops=dict(arrowstyle="-", color=KRITIK, lw=0.8))
    # uçurum: saf kalıntı
    ax.annotate(f"saf kalıntı (w = 1)\nPR-AUC {tr(pr[-1])}", (1.0, pr[-1]),
                textcoords="offset points", xytext=(-34, 2), ha="right",
                va="bottom", fontsize=7.5, color=INK2, linespacing=1.3,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax.annotate(f"saf ham (w = 0)\nPR-AUC {tr(pr[0])}", (0.0, pr[0]),
                textcoords="offset points", xytext=(6, -4), ha="left",
                va="top", fontsize=7.5, color=INK2)
    ax.set_xlabel("w_kal  (kalıntı modelinin ağırlığı;  w_ham = 1 − w_kal)")
    ax.set_ylabel("başarım")
    ax.set_xlim(-0.02, 1.02); ax.set_ylim(0.42, 1.06)
    tr_eksen(ax)
    ax.legend(loc="lower left", handlelength=1.6, bbox_to_anchor=(0.10, 0.02))
    ax.set_title(f"Ağırlık duyarlılığı — %5 ham ağırlık PR-AUC'yi "
                 f"{tr(pr[-1])}'ten {tr(pr[95])}'e taşıyor", loc="left")
    kaydet(fig, "sekil_agirlik.png")
    return ws, pr, auc


def sekil_ariza_tipi():
    tipler = ["Motor\nkayması", "Çarpışma", "Gizyazar\nhatası", "Sensör\ngürültüsü"]
    kal = [0.451, 0.995, 0.988, 0.296]
    ham = [0.256, 0.999, 0.491, 1.000]
    bir = [0.443, 0.996, 0.989, 0.996]
    x = np.arange(len(tipler)); g = 0.26

    fig, ax = plt.subplots(figsize=(6.3, 2.6))
    duzenle(ax)
    ax.xaxis.grid(False)
    for ofs, veri, renk, ad in ((-g, kal, KAL, "Kalıntı ÖzK."),
                                (0.0, ham, HAM, "Ham ÖzK."),
                                (+g, bir, BIR, "Birleşim")):
        ax.bar(x + ofs, veri, g * 0.92, color=renk, edgecolor=SURFACE,
               linewidth=1.2, label=ad)
        for xi, v in zip(x + ofs, veri):
            ax.text(xi, v + 0.02, tr(v, 2), ha="center",
                    va="bottom", fontsize=7, color=INK2)
    ax.set_xticks(x, tipler)
    ax.set_ylim(0, 1.14); ax.set_yticks([0, 0.5, 1.0]); tr_eksen(ax, "y")
    ax.set_ylabel("tespit oranı")
    ax.set_title("Arıza tipine göre tespit oranı — iki model birbirinin "
                 "kör noktasını kapatıyor", loc="left", pad=20)
    ax.legend(loc="lower left", ncol=3, handlelength=1.4,
              bbox_to_anchor=(0.0, 1.01), borderaxespad=0)
    kaydet(fig, "sekil_ariza_tipi.png")


# ═════════════════════════════════════════════════════════════════════
# GERÇEK ROBOT (Bölüm 10)
# ═════════════════════════════════════════════════════════════════════
def csv_yukle(ad):
    yol = KAYIT / ad
    if not yol.exists():
        return None
    return np.genfromtxt(yol, delimiter=",", names=True)


def sekil_kilit():
    d = csv_yukle("skorlar_20260821_101757.csv")
    if d is None:
        return
    t = d["t_ros"] - d["t_ros"][0]
    f = d["birlesik"]
    ta = np.where(d["thr_uyarlanabilir"] > 0, d["thr_uyarlanabilir"], np.nan)
    ia = int(np.argmax(d["alarm"] > 0))

    fig, ax = plt.subplots(figsize=(6.3, 3.1))
    duzenle(ax)
    ax.set_yscale("log")
    ax.axvspan(t[ia], t[-1], color=KRITIK, alpha=0.06, lw=0)
    ax.plot(t, np.maximum(f, 1e-3), color=BIR, lw=0.8, label="birleşik skor")
    ax.plot(t, ta, color=KRITIK, lw=1.7, ls=(0, (4, 3)),
            label="uyarlanabilir eşik (donmuş, 0,0103)")

    ipk = int(np.argmax(f))
    ax.plot(t[ipk], f[ipk], "o", ms=6, color=KRITIK, mec=SURFACE, mew=1.3, zorder=6)
    ax.annotate("el sıkışması — 10,27  (10:21:27)\nkoşunun en yüksek değeri,\n"
                "yeni olay kaydı ÜRETİLMEDİ",
                (t[ipk], f[ipk]), textcoords="offset points", xytext=(-12, 22),
                ha="right", fontsize=7.5, color=INK, linespacing=1.3,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax.annotate("koruyucu durdurma platosu\n122 s · medyan 0,32",
                (271, 0.28), textcoords="offset points", xytext=(-6, -22),
                ha="right", va="top", fontsize=7.5, color=INK2, linespacing=1.3,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax.annotate(f"alarm +{t[ia]:.0f} s'de kilitlendi;\nkayıt sonuna kadar "
                f"{t[-1]-t[ia]:.0f} s hiç açılmadı",
                (t[ia] + 10, 2.6e-3), ha="left", va="center", fontsize=7.5,
                color=KRITIK, fontweight="bold", linespacing=1.35)

    ax.set_xlabel("koşu başlangıcından itibaren saniye")
    ax.set_ylabel("birleşik skor  (log)")
    ax.set_xlim(0, t[-1]); ax.set_ylim(1e-3, 600)
    ax.legend(loc="upper right", handlelength=1.9)
    ax.set_title("Uyarlanabilir kuralın kalıcı kilitlenmesi — 10:17 koşusu", loc="left")
    kaydet(fig, "sekil_kilit.png")


def sekil_rejim():
    """Poza bağımlılık. Duruş/hareket ayrımı hareket bayrağı olan 10:38 koşusundan,
    katlanmış park pozu ile koruyucu durdurma platosu 10:17 koşusundan gelir."""
    d1 = csv_yukle("skorlar_20260821_101757.csv")
    d2 = csv_yukle("skorlar_20260821_103807.csv")
    if d1 is None or d2 is None:
        return
    t1 = d1["t_ros"] - d1["t_ros"][0]; f1 = d1["birlesik"]
    f2 = d2["birlesik"]; mv = d2["hareket"] > 0

    rej = [("katlanmış\npark pozu", f1[t1 < 55.70], "10:17", MUTED),
           ("koruyucu\ndurdurma", f1[(t1 > 210.85) & (t1 < 332.9)], "10:17", UYARI),
           ("normal\nhareket", f2[mv], "10:38", BIR),
           ("sıkışma\n(11 s)", f1[(t1 >= 199.85) & (t1 <= 210.85)], "10:17", KRITIK),
           ("yerçekimi yüklü\nduruş", f2[~mv], "10:38", HAM)]

    fig, ax = plt.subplots(figsize=(6.3, 3.0))
    duzenle(ax)
    ax.set_yscale("log")
    ax.xaxis.grid(False)
    for i, (ad, v, kosu, renk) in enumerate(rej):
        v = v[v > 0]
        rng = np.random.default_rng(7 + i)
        vv = v if len(v) <= 800 else v[:: max(1, len(v) // 800)]
        x = i + (rng.random(len(vv)) - 0.5) * 0.44
        ax.plot(x, vv, ".", ms=2.0, color=renk, alpha=0.30, mec="none")
        med = np.median(v)
        ax.plot([i - 0.30, i + 0.30], [med, med], color=renk, lw=2.6,
                solid_capstyle="butt", zorder=5)
        ax.text(i, v.max() * 1.7, tr(med, 4 if med < 0.01 else 2),
                ha="center", fontsize=8, color=INK, fontweight="bold")
        ax.text(i, 4.5e-4, kosu, ha="center", fontsize=6.8, color=MUTED)
    ax.axhline(0.6436, color=KRITIK, lw=1.3, ls=(0, (4, 3)))
    ax.text(-0.42, 0.6436 * 1.25, "θ_FMU = 0,6436", ha="left", fontsize=8,
            color=KRITIK, fontweight="bold")
    ax.set_xticks(range(len(rej)), [r[0] for r in rej])
    ax.set_xlim(-0.55, len(rej) - 0.45)
    ax.set_ylabel("birleşik skor  (log)")
    ax.set_ylim(3e-4, 200)
    ax.set_title("Kalıntı poza bağlıdır — üç kat büyüklük fark", loc="left")
    kaydet(fig, "sekil_rejim.png")


# etiketli küme (Bölüm 10.4/10.5) — operatörün doğruladığı tepe değerleri
GERCEK = [10.27, 31.98, 79.86, 146.44, 195.83]
YANLIS = [17.01, 10.29, 8.31, 16.80, 9.96, 8.54, 8.36, 17.43, 10.61, 8.41,
          17.37, 10.32, 8.39, 17.16, 10.42, 8.30,
          32.98, 90.55, 19.17, 9.78, 25.49,
          13.28, 10.50, 8.67, 9.61, 11.59, 9.55, 24.21, 8.28]
GOZLEM_SAAT = 45 / 60.0


def sekil_esik():
    th = np.geomspace(7.0, 220.0, 900)
    g = np.array([sum(v >= x for v in GERCEK) for x in th])
    y = np.array([sum(v >= x for v in YANLIS) for x in th]) / GOZLEM_SAAT

    fig, axes = plt.subplots(2, 1, figsize=(6.3, 3.5), sharex=True,
                             gridspec_kw={"hspace": 0.16})
    for ax in axes:
        duzenle(ax)
        ax.set_xscale("log")
        ax.axvline(18.0, color=KRITIK, lw=1.4, ls=(0, (4, 3)), zorder=1)

    ax = axes[0]
    ax.step(th, g, where="post", color=IYI, lw=2.1)
    ax.set_ylabel("yakalanan\ngerçek olay")
    ax.set_ylim(-0.3, 6.3); ax.set_yticks([0, 1, 2, 3, 4, 5])
    ax.text(18.7, 5.9, "seçilen θ = 18,0", color=KRITIK, fontsize=8.5,
            fontweight="bold", va="top")
    ax.plot(18.0, 4, "o", ms=6, color=KRITIK, mec=SURFACE, mew=1.3, zorder=6)
    ax.annotate("31,98 — en zayıf doğrulanmış gerçek olay;\n"
                "θ = 18 buraya %78 pay bırakıyor",
                (31.98, 4), textcoords="offset points", xytext=(16, -20),
                fontsize=7.5, color=INK2, linespacing=1.3,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax.annotate("10,27 — kaçan tek olay\n(robotun kendi koruyucu durdurması zaten kesti)",
                (10.27, 4.5), textcoords="offset points", xytext=(8, -30),
                ha="left", va="top", fontsize=7.5, color=INK2, linespacing=1.3,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax.set_title("Eşik takası — 34 etiketli olay, ~45 dakika gerçek robot gözlemi",
                 loc="left")

    ax = axes[1]
    ax.step(th, y, where="post", color=KRITIK, lw=2.1)
    ax.set_ylabel("yanlış alarm\n(saat başına)")
    ax.set_xlabel("mutlak eşik  θ  (log)")
    ax.set_ylim(-1.5, 46)
    ax.plot(18.0, 6.7, "o", ms=6, color=KRITIK, mec=SURFACE, mew=1.3, zorder=6)
    ax.annotate("FMU eşiği (0,64) bu ölçeğin solunda kalıyor;\n"
                "orada oran 38,7 yanlış alarm/saat",
                (7.2, 38.7), textcoords="offset points", xytext=(150, 4),
                ha="right", va="bottom", fontsize=7.5, color=INK2, linespacing=1.3,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax.annotate("muayene çevriminin tavanı 17,43 —\nθ = 18 o çevrimin 16 periyodik\n"
                "yanlış alarmını birden susturuyor",
                (17.43, 14.7), textcoords="offset points", xytext=(30, -6),
                fontsize=7.5, color=INK2, linespacing=1.3,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax.annotate("6,7", (18.0, 6.7), textcoords="offset points", xytext=(8, 5),
                fontsize=8.5, color=KRITIK, fontweight="bold")
    for ax in axes:
        ax.set_xlim(7, 220)
        ax.set_xticks([8, 10, 18, 30, 50, 100, 200])
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
        ax.minorticks_off()
    kaydet(fig, "sekil_esik.png")


def sekil_muayene():
    d = csv_yukle("anomali_kayit_inspection/skorlar_20260821_114720.csv")
    if d is None:
        return
    t = d["t_ros"] - d["t_ros"][0]
    f = d["birlesik"]

    fig, ax = plt.subplots(figsize=(6.3, 2.9))
    duzenle(ax)
    ax.plot(t, f, color=BIR, lw=0.8)
    ax.axhline(18.0, color=KRITIK, lw=1.5, ls=(0, (4, 3)))
    ax.text(6, 18.5, "θ = 18,0", ha="left", fontsize=8, color=KRITIK,
            fontweight="bold")
    ax.axhline(8.0, color=MUTED, lw=0.9, ls=(0, (2, 3)))
    ax.text(6, 8.4, "olay eşiği ≈ 8", ha="left", fontsize=7.5, color=MUTED)

    # ana tepeler ~92 s aralıklı
    ana = [35, 132, 224, 316, 407]
    for a_, b_ in zip(ana, ana[1:]):
        ax.annotate("", (a_, 27.0), xytext=(b_, 27.0),
                    arrowprops=dict(arrowstyle="<->", color=MUTED, lw=0.8))
        ax.text((a_ + b_) / 2, 27.4, "≈92 s", ha="center", va="bottom",
                fontsize=7, color=MUTED)
    ax.annotate("her çevrimde aynı üçlü örüntü:\n"
                "6,2 s / tepe ~17  →  3,0 s / ~10,3  →  0,15 s / ~8,4",
                (35, 17.0), textcoords="offset points", xytext=(30, 30),
                fontsize=7.5, color=INK2, linespacing=1.3,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax.set_xlabel("koşu başlangıcından itibaren saniye")
    ax.set_ylabel("birleşik skor")
    ax.set_xlim(0, t[-1]); ax.set_ylim(0, 30.5)
    tr_eksen(ax, "y", 0)
    ax.set_title("Yanlış alarmlar gürültü değil — yörüngeyle birebir tekrarlı",
                 loc="left")
    kaydet(fig, "sekil_muayene.png")


def sekil_tepeler():
    fig, ax = plt.subplots(figsize=(6.3, 2.4))
    duzenle(ax)
    ax.set_xscale("log")
    ax.yaxis.grid(False)
    rng = np.random.default_rng(3)
    for i, (veri, renk) in enumerate(((YANLIS, MUTED), (GERCEK, KRITIK))):
        yy = i + (rng.random(len(veri)) - 0.5) * 0.28
        ax.plot(veri, yy, "o", ms=6.5, color=renk, alpha=0.75, mec=SURFACE,
                mew=1.0)
    ax.axvline(18.0, color=KRITIK, lw=1.4, ls=(0, (4, 3)))
    ax.text(19, 1.72, "θ = 18,0", color=KRITIK, fontsize=8.5,
            fontweight="bold", va="top")
    ax.annotate("90,55 — bir YANLIŞ alarmın tepesi;\nüç gerçek olaydan yüksek",
                (90.55, 0), textcoords="offset points", xytext=(-12, -20),
                ha="right", va="top", fontsize=7.5, color=INK2, linespacing=1.3,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax.annotate("10,27 — kaçan\ntek gerçek olay", (10.27, 1),
                textcoords="offset points", xytext=(0, 18), ha="center",
                fontsize=7.5, color=INK2, linespacing=1.3,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax.set_yticks([0, 1], [f"yanlış alarm\n(n = {len(YANLIS)})",
                           f"gerçek olay\n(n = {len(GERCEK)})"])
    ax.tick_params(axis="y", labelsize=7.5)
    ax.set_ylim(-0.95, 1.75)
    ax.set_xlim(7, 300)
    ax.set_xticks([8, 10, 18, 30, 50, 100, 200])
    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
    ax.minorticks_off()
    ax.set_xlabel("olay tepe skoru  (log)")
    ax.set_title("İki sınıfın örtüşmesi eşikle temizlenemez", loc="left")
    kaydet(fig, "sekil_tepeler.png")


# ═════════════════════════════════════════════════════════════════════
def main():
    print("Şekiller üretiliyor →", FIG)
    D = cevrimdisi_yukle()
    if D is None:
        print(f"  ! {NPZ} yok — Bölüm 6 şekilleri atlanıyor", file=sys.stderr)
    else:
        sekil_pr_roc(D)
        TP, FP, FN, TN = sekil_karisiklik(D)
        print(f"    (DP={TP} YP={FP} YN={FN} DN={TN})")
        sekil_agirlik(D)
    sekil_ariza_tipi()
    sekil_kilit()
    sekil_rejim()
    sekil_esik()
    sekil_muayene()
    sekil_tepeler()
    print("bitti.")


if __name__ == "__main__":
    main()
