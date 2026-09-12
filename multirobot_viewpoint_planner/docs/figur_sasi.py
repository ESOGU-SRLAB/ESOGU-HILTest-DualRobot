#!/usr/bin/env python3
"""Şasi senaryosunun GÜNCEL figürlerini üretir (plan + octomap karşılaştırması).

    python3 docs/figur_sasi.py             # hepsi
    python3 docs/figur_sasi.py octomap     # yalnız adı geçen

NEDEN AYRI BİR DOSYA: figur_uret.py kapı bölümünün figürlerini üretiyor ve orada
kalıyor; bu dosya şasi senaryosunun Eylül 2026 durumunu üretir. Ortografik kamera,
voksel çizimi ve ColorOcTree okuyucusu ORADAN alınır (aynı dizin, aynı paket), yani
tek kopya vardır.

plan_visualizer.py BU KUTUDA ÇALIŞMAZ: mpl_toolkits.mplot3d (sistem paketi) pip
matplotlib'i gölgeliyor ve Axes3D import edilemiyor. Bu yüzden plan figürleri de
buradaki ortografik izdüşümle çizilir.

Kaplama tanımı (raporda da böyle yazılıdır): belief haritasındaki her şasi
vokseli 2 cm'lik çözünürlüğe açılır (budanmış 4 cm yapraklar 8 alt voksele
bölünür), occupancy haritasının DOLU voksellerine aynı işlem uygulanır ve
kaplama = |kesişim| / |şasi vokselleri| olarak hesaplanır.
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from figur_uret import camera, project, draw_voxels, frame, read_ot  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
PLAN = os.path.join(PKG, "plans", "multirobot_viewpoint_plan.json")
PCDS = os.path.expanduser("~/colcon_ws/src/pcds")

UR_CMAP = "autumn"
KAWA_CMAP = "winter"
INK = "#22252a"


# --------------------------------------------------------------------------- #
# voksel yardımcıları
# --------------------------------------------------------------------------- #
def voxel_keys(tree):
    """Yaprakları en ince çözünürlüğe açar; anahtar = (i, j, k) tam sayı ızgarası.

    Budanmış yapraklar (4 cm) octomap'te tek düğüm olarak durur ama 8 tane 2 cm'lik
    vokseli temsil eder; saymadan önce açılmazsa kaplama yanlış çıkar."""
    res = tree["res"]
    keys = {}
    for c, s, rgb in zip(tree["centers"], tree["sizes"], tree["rgb"]):
        n = int(round(s / res))
        if n <= 1:
            keys[tuple(np.round(c / res).astype(int))] = tuple(rgb)
            continue
        off = (np.arange(n) - (n - 1) / 2.0) * res
        for dx in off:
            for dy in off:
                for dz in off:
                    k = tuple(np.round((c + [dx, dy, dz]) / res).astype(int))
                    keys[k] = tuple(rgb)
    return keys, res


def coverage(belief_path, occ_path):
    belief = read_ot(belief_path, occupied_only=False)
    occ = read_ot(occ_path, occupied_only=True)
    kb, res = voxel_keys(belief)
    ko, _ = voxel_keys(occ)
    covered = {k for k in kb if k in ko}
    return dict(res=res, belief=kb, covered=covered,
                frac=len(covered) / max(1, len(kb)))


def per_color_coverage(cov):
    """Renk (yani şasi parçası) başına kaplama."""
    tot, hit = {}, {}
    for k, rgb in cov["belief"].items():
        tot[rgb] = tot.get(rgb, 0) + 1
        if k in cov["covered"]:
            hit[rgb] = hit.get(rgb, 0) + 1
    return {c: (hit.get(c, 0), n) for c, n in tot.items()}


# --------------------------------------------------------------------------- #
# FİGÜR — güncel çok-robot planı, dört görünüm
# --------------------------------------------------------------------------- #
def _arrows(ax, vps, basis, cmap, scale=0.16):
    P = np.array([v["position"] for v in vps])
    # rotation satırları kamera eksenlerini verir; bakış ekseni üçüncü satırın
    # tersidir (viewpoint üretiminde -Z ileri yönü kullanılmıştır).
    D = np.array([-np.asarray(v["rotation"])[2] for v in vps])
    gain = np.array([v.get("new_points_covered") or 0 for v in vps], dtype=float)
    xy0, _ = project(P, basis)
    xy1, _ = project(P + D * scale, basis)
    norm = plt.Normalize(gain.min(), max(gain.max(), gain.min() + 1))
    cols = plt.get_cmap(cmap)(norm(gain))
    for a, b, c in zip(xy0, xy1, cols):
        ax.annotate("", xy=b, xytext=a,
                    arrowprops=dict(arrowstyle="-|>", color=c, lw=1.1,
                                    shrinkA=0, shrinkB=0))
    ax.scatter(xy0[:, 0], xy0[:, 1], s=16, c=cols, edgecolors="white", linewidths=0.4,
               zorder=5)
    return np.vstack([xy0, xy1])


def fig_plan():
    plan = json.load(open(PLAN))
    ur, kawa = plan["ur_viewpoints"], plan["kawasaki_viewpoints"]
    belief = read_ot(os.path.join(PCDS, "real_pcds", "beliefMap_real.ot"),
                     occupied_only=False)
    views = [(22, 210, "izometrik"), (2, 180, "ön (−X'ten)"),
             (2, 270, "yan (−Y'den)"), (88, 200, "tepe")]
    fig, axes = plt.subplots(2, 2, figsize=(9.6, 8.4), dpi=170)
    for ax, (elev, azim, title) in zip(axes.ravel(), views):
        basis = camera(elev, azim)
        draw_voxels(ax, belief["centers"], np.full_like(belief["rgb"], 150),
                    belief["sizes"], basis, alpha=0.55)
        a = _arrows(ax, ur, basis, UR_CMAP)
        b = _arrows(ax, kawa, basis, KAWA_CMAP)
        pts, _ = project(belief["centers"], basis)
        frame(ax, [pts, a, b], pad=1.06)
        ax.set_title(title, fontsize=10, color=INK)
    fig.suptitle(
        f"Güncel çok-robot planı — {len(ur)} UR10e (sıcak renkler) + {len(kawa)} "
        f"Kawasaki (soğuk renkler) bakış-noktası\n"
        f"renk = o bakış-noktasının getirdiği YENİ nokta sayısı; oklar görüş ekseni; "
        f"gri voksel bulutu şasinin kendisi", fontsize=11, color=INK)
    fig.tight_layout()
    out = os.path.join(HERE, "fig_multirobot_plan.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


def _fig_plan_single(vps, cmap, fname, title):
    belief = read_ot(os.path.join(PCDS, "real_pcds", "beliefMap_real.ot"),
                     occupied_only=False)
    fig, axes = plt.subplots(1, 2, figsize=(9.4, 4.6), dpi=170)
    for ax, (elev, azim, sub) in zip(axes, [(22, 210, "izometrik"), (88, 200, "tepe")]):
        basis = camera(elev, azim)
        draw_voxels(ax, belief["centers"], np.full_like(belief["rgb"], 150),
                    belief["sizes"], basis, alpha=0.5)
        a = _arrows(ax, vps, basis, cmap)
        pts, _ = project(belief["centers"], basis)
        frame(ax, [pts, a], pad=1.06)
        ax.set_title(sub, fontsize=10, color=INK)
    P = np.array([v["position"] for v in vps])
    fig.suptitle(f"{title}\nX aralığı {P[:, 0].min():.2f} … {P[:, 0].max():.2f} m, "
                 f"yükseklik {P[:, 2].min():.2f} … {P[:, 2].max():.2f} m",
                 fontsize=11, color=INK)
    fig.tight_layout()
    out = os.path.join(HERE, fname)
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


def fig_plan_ur():
    plan = json.load(open(PLAN))
    _fig_plan_single(plan["ur_viewpoints"], UR_CMAP, "fig_multirobot_ur10e.png",
                     f"Aynı plan, yalnız UR10e'nin {len(plan['ur_viewpoints'])} "
                     f"bakış-noktası")


def fig_plan_kawasaki():
    plan = json.load(open(PLAN))
    _fig_plan_single(plan["kawasaki_viewpoints"], KAWA_CMAP,
                     "fig_multirobot_kawasaki.png",
                     f"Aynı plan, yalnız Kawasaki'nin "
                     f"{len(plan['kawasaki_viewpoints'])} bakış-noktası")


# --------------------------------------------------------------------------- #
# FİGÜR — gerçek robot ile simülasyonun octomap'i yan yana
# --------------------------------------------------------------------------- #
def _octomap_panel(ax, cov, basis, title):
    res = cov["res"]
    keys = np.array(list(cov["belief"].keys()), dtype=float) * res
    covered = np.array([k in cov["covered"] for k in cov["belief"]])
    rgb = np.where(covered[:, None], np.array([90, 160, 100]), np.array([200, 70, 60]))
    draw_voxels(ax, keys, rgb.astype(np.uint8), np.full(len(keys), res), basis)
    pts, _ = project(keys, basis)
    frame(ax, [pts], pad=1.03)
    ax.set_title(title, fontsize=10, color=INK)
    return pts


def fig_octomap(prefix="", belief_real=None, occ_real=None, belief_sim=None,
                occ_sim=None, out_name="fig_octomap_real_vs_sim.png",
                label_real="Gerçek robot", label_sim="Simülasyon"):
    belief_real = belief_real or os.path.join(PCDS, "real_pcds", "beliefMap_real.ot")
    occ_real = occ_real or os.path.join(PCDS, "real_pcds", "occupancyMap_real.ot")
    belief_sim = belief_sim or os.path.join(PCDS, "sim_pcds", "beliefMap_sim.ot")
    occ_sim = occ_sim or os.path.join(PCDS, "sim_pcds", "occupancyMap_sim.ot")
    cr = coverage(belief_real, occ_real)
    cs = coverage(belief_sim, occ_sim)
    fig, axes = plt.subplots(2, 2, figsize=(9.4, 8.0), dpi=170)
    for row, (cov, label) in enumerate(((cr, label_real), (cs, label_sim))):
        for col, (elev, azim, sub) in enumerate([(20, 210, "izometrik"),
                                                 (4, 275, "yan")]):
            _octomap_panel(axes[row, col], cov, camera(elev, azim),
                           f"{label} — {sub}  (kaplama %{100 * cov['frac']:.1f})")
    fig.suptitle("Şasi octomap'i: hangi yüzey görüldü, hangisi görülmedi\n"
                 "yeşil = sensörle kaplanan şasi vokseli · kırmızı = kaplanmayan · "
                 "2 cm çözünürlük, İKİ HARİTA DA AYNI kamera açılarıyla çizildi",
                 fontsize=11, color=INK)
    fig.tight_layout()
    out = os.path.join(HERE, out_name)
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out, f"(real %{100 * cr['frac']:.1f}, sim %{100 * cs['frac']:.1f})")
    return cr, cs


# --------------------------------------------------------------------------- #
# FİGÜR — parça başına kaplama (gerçek vs sim)
# --------------------------------------------------------------------------- #
def fig_part_coverage(belief_real=None, occ_real=None, belief_sim=None, occ_sim=None,
                      out_name="fig_part_coverage.png", title_extra=""):
    belief_real = belief_real or os.path.join(PCDS, "real_pcds", "beliefMap_real.ot")
    occ_real = occ_real or os.path.join(PCDS, "real_pcds", "occupancyMap_real.ot")
    belief_sim = belief_sim or os.path.join(PCDS, "sim_pcds", "beliefMap_sim.ot")
    occ_sim = occ_sim or os.path.join(PCDS, "sim_pcds", "occupancyMap_sim.ot")
    cr, cs = coverage(belief_real, occ_real), coverage(belief_sim, occ_sim)
    pr, ps = per_color_coverage(cr), per_color_coverage(cs)
    colors = sorted(pr, key=lambda c: pr[c][0] / pr[c][1])
    real = [100 * pr[c][0] / pr[c][1] for c in colors]
    sim = [100 * ps.get(c, (0, 1))[0] / max(1, ps.get(c, (0, 1))[1]) for c in colors]
    size = [pr[c][1] for c in colors]

    fig, ax = plt.subplots(figsize=(9.6, 4.6), dpi=170)
    x = np.arange(len(colors))
    ax.bar(x - 0.2, real, 0.4, color="#c81e1e", label="Gerçek robot")
    ax.bar(x + 0.2, sim, 0.4, color="#1f6fd0", label="Simülasyon")
    ax.set_xticks(x)
    ax.set_xticklabels([f"{np.asarray(c)}" for c in colors], rotation=90, fontsize=4)
    ax.set_xlabel("şasi parçaları (belief haritasındaki renge göre, en kötüden en iyiye)",
                  fontsize=9)
    ax.set_ylabel("kaplama [%]", fontsize=9)
    ax.set_ylim(0, 105)
    ax.grid(axis="y", alpha=0.3, lw=0.5)
    ax.legend(fontsize=8.5, loc="lower right")
    worst = [(colors[i], real[i], sim[i], size[i]) for i in range(min(6, len(colors)))]
    txt = "\n".join(f"{np.asarray(c)}: gerçek %{r:.0f} / sim %{s:.0f}  ({n} voksel)"
                    for c, r, s, n in worst)
    ax.text(0.01, 0.97, "en kötü altı parça\n" + txt, transform=ax.transAxes,
            va="top", fontsize=6.5, color=INK,
            bbox=dict(boxstyle="round", fc="white", ec="#cccccc", alpha=0.9))
    ax.set_title("Parça başına kaplama — gerçek robot ile simülasyon" + title_extra,
                 fontsize=11, color=INK)
    fig.tight_layout()
    out = os.path.join(HERE, out_name)
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)
    return cr, cs


FIGURES = {
    "plan": fig_plan,
    "ur": fig_plan_ur,
    "kawasaki": fig_plan_kawasaki,
    "octomap": fig_octomap,
    "parts": fig_part_coverage,
}


if __name__ == "__main__":
    wanted = [a for a in sys.argv[1:] if a in FIGURES] or list(FIGURES)
    for key in wanted:
        print(f"  {key} ...", flush=True)
        FIGURES[key]()
