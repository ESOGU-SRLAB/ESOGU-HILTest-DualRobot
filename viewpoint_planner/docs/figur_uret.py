#!/usr/bin/env python3
"""Tek-kol UR10e muayene raporunun figürlerini üretir.

    python3 docs/figur_uret.py            # hepsi
    python3 docs/figur_uret.py octomap    # yalnız adı geçen

NEDEN KENDİ İZDÜŞÜMÜ: bu kutuda mplot3d KIRIK (sistem paketi
/usr/lib/python3/dist-packages/mpl_toolkits pip matplotlib'i gölgeliyor, Axes3D
import edilemiyor), bu yüzden paketin kendi plan_visualizer'ı da çalışmıyor.
Ortografik kamera + painter's-algorithm sıralaması ile voksel/ok çizimi burada.

NEDEN KOPYA: ColorOcTree okuyucusu ve çizim yardımcıları
multirobot_viewpoint_planner/docs altındakilerin aynısıdır. İki paket birbirine
bağımlı olmasın diye bilerek kopyalanmıştır; birinde düzeltme yapılırsa diğerine de
taşıyın.
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
PLAN = os.path.join(PKG, "plans", "viewpoint_plan.json")
PCDS = os.path.expanduser("~/colcon_ws/src/pcds")
SINGLE = os.path.join(PCDS, "single_ur10e")
INK = "#22252a"
ACCENT = "#1f6feb"
WARM = "#d9480f"


# --------------------------------------------------------------------------- #
# ortografik kamera
# --------------------------------------------------------------------------- #
def camera(elev_deg, azim_deg):
    e, a = np.radians(elev_deg), np.radians(azim_deg)
    fwd = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    return right, up, fwd


def project(P, basis):
    right, up, fwd = basis
    return np.stack([P @ right, P @ up], axis=-1), P @ fwd


def frame(ax, pts_list, pad=1.04):
    allp = np.concatenate([p for p in pts_list if len(p)])
    lo, hi = allp.min(0), allp.max(0)
    c, r = (lo + hi) / 2.0, (hi - lo).max() / 2.0 * pad
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_aspect("equal")
    ax.axis("off")


# --------------------------------------------------------------------------- #
# ColorOcTree (.ot) okuyucu — düğüm başına float32 log-odds | uint8 rgb | uint8 maske
# --------------------------------------------------------------------------- #
def read_ot(path, occupied_only=True):
    raw = open(path, "rb").read()
    i = raw.index(b"data\n") + 5
    head = raw[:i].decode("ascii", "replace").splitlines()
    res = float([l.split()[1] for l in head if l.startswith("res ")][0])
    size = int([l.split()[1] for l in head if l.startswith("size ")][0])
    buf = raw[i:]
    if len(buf) != size * 8:
        raise ValueError(f"{path}: beklenmeyen gövde boyutu {len(buf)} != {size * 8}")
    b = np.frombuffer(buf, dtype=np.uint8).reshape(size, 8)
    logodds = b[:, :4].copy().view(np.float32).ravel()
    rgb = b[:, 4:7]
    children = b[:, 7]
    centers = np.zeros((size, 3))
    sizes = np.zeros(size)
    leaf = np.zeros(size, dtype=bool)
    stack = [(np.zeros(3), res * (2 ** 16))]
    n = 0
    while stack:
        c, s = stack.pop()
        centers[n], sizes[n] = c, s
        ch = children[n]
        n += 1
        if ch == 0:
            leaf[n - 1] = True
            continue
        off = s / 4.0
        kids = []
        for bit in range(8):
            if ch & (1 << bit):
                kids.append((c + np.array([off if bit & 1 else -off,
                                           off if bit & 2 else -off,
                                           off if bit & 4 else -off]), s / 2.0))
        stack.extend(reversed(kids))
    keep = leaf & (logodds > 0) if occupied_only else leaf
    return dict(res=res, centers=centers[keep], rgb=rgb[keep], sizes=sizes[keep])


def voxel_keys(tree):
    """Budanmış yaprakları en ince çözünürlüğe açar (4 cm yaprak = 8 x 2 cm voksel)."""
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
                    keys[tuple(np.round((c + [dx, dy, dz]) / res).astype(int))] = tuple(rgb)
    return keys, res


def coverage(belief_path, occ_path):
    kb, res = voxel_keys(read_ot(belief_path, occupied_only=False))
    ko, _ = voxel_keys(read_ot(occ_path, occupied_only=True))
    covered = {k for k in kb if k in ko}
    return dict(res=res, belief=kb, covered=covered,
                frac=len(covered) / max(1, len(kb)))


def per_color_coverage(cov):
    tot, hit = {}, {}
    for k, rgb in cov["belief"].items():
        tot[rgb] = tot.get(rgb, 0) + 1
        if k in cov["covered"]:
            hit[rgb] = hit.get(rgb, 0) + 1
    return {c: (hit.get(c, 0), n) for c, n in tot.items()}


_CUBE_FACES = [
    ((0, 0, -1), [(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1)]),
    ((0, 0, +1), [(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]),
    ((0, -1, 0), [(-1, -1, -1), (1, -1, -1), (1, -1, 1), (-1, -1, 1)]),
    ((0, +1, 0), [(-1, 1, -1), (1, 1, -1), (1, 1, 1), (-1, 1, 1)]),
    ((-1, 0, 0), [(-1, -1, -1), (-1, 1, -1), (-1, 1, 1), (-1, -1, 1)]),
    ((+1, 0, 0), [(1, -1, -1), (1, 1, -1), (1, 1, 1), (1, -1, 1)]),
]


def draw_voxels(ax, centers, rgb, sizes, basis, alpha=1.0):
    right, up, fwd = basis
    polys, cols, depths = [], [], []
    half = sizes[:, None] / 2.0
    for normal, corners in _CUBE_FACES:
        nrm = np.array(normal, dtype=float)
        if nrm @ fwd >= 0:
            continue
        shade = 0.55 + 0.45 * abs(nrm @ (-fwd))
        C = np.array(corners, dtype=float)
        pts = centers[:, None, :] + C[None, :, :] * half[:, None, :]
        xy, _ = project(pts.reshape(-1, 3), basis)
        polys.append(xy.reshape(-1, 4, 2))
        cols.append(np.clip(np.asarray(rgb) / 255.0 * shade, 0, 1))
        depths.append(centers @ fwd)
    P = np.concatenate(polys)
    C = np.concatenate(cols)
    D = np.concatenate(depths)
    o = np.argsort(-D)
    ax.add_collection(PolyCollection(P[o], facecolors=C[o], edgecolors="none",
                                     alpha=alpha))


# --------------------------------------------------------------------------- #
# plan yardımcıları
# --------------------------------------------------------------------------- #
def load_plan():
    return json.load(open(PLAN))


def chassis_voxels():
    return read_ot(os.path.join(SINGLE, "real_data",
                                "beliefMap_single_ur10e_real.ot"), occupied_only=False)


# UR10e eklemlerinin hangileri 2π kaydırılabilir (elbow ±π ile sınırlı) — sıralama
# maliyeti bu bilgiyle hesaplanır, planlayıcı da aynısını kullanır.
RAIL_WEIGHT = 2.0       # order_rail_weight: 1 m ray kaç radyan eklem yoluna bedel


def joint_cost(a, b):
    """Planlayıcının sıralamada kullandığı maliyet: ray metresi ağırlıklı eklem yolu."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = np.abs(b - a)
    return float(d[0] * RAIL_WEIGHT + d[1:].sum())


# --------------------------------------------------------------------------- #
# FİGÜR — plan, dört görünüm
# --------------------------------------------------------------------------- #
def fig_plan():
    plan = load_plan()
    vps = plan["ur_viewpoints"]
    belief = chassis_voxels()
    P = np.array([v["position"] for v in vps])
    D = np.array([-np.asarray(v["rotation"])[2] for v in vps])
    gain = np.array([v.get("new_points_covered") or 0 for v in vps], float)
    norm = plt.Normalize(gain.min(), gain.max())
    cols = plt.get_cmap("autumn")(norm(gain))

    views = [(22, 210, "izometrik"), (2, 180, "ön (−X'ten)"),
             (2, 270, "yan (−Y'den)"), (88, 200, "tepe")]
    fig, axes = plt.subplots(2, 2, figsize=(9.6, 8.4), dpi=170)
    for ax, (elev, azim, title) in zip(axes.ravel(), views):
        basis = camera(elev, azim)
        draw_voxels(ax, belief["centers"], np.full_like(belief["rgb"], 150),
                    belief["sizes"], basis, alpha=0.55)
        xy0, _ = project(P, basis)
        xy1, _ = project(P + D * 0.16, basis)
        for a, b, c in zip(xy0, xy1, cols):
            ax.annotate("", xy=b, xytext=a,
                        arrowprops=dict(arrowstyle="-|>", color=c, lw=1.1,
                                        shrinkA=0, shrinkB=0))
        ax.scatter(xy0[:, 0], xy0[:, 1], s=18, c=cols, edgecolors="white",
                   linewidths=0.4, zorder=5)
        pts, _ = project(belief["centers"], basis)
        frame(ax, [pts, xy0, xy1], pad=1.06)
        ax.set_title(title, fontsize=10, color=INK)
    fig.suptitle(f"Tek-kol UR10e planı — {len(vps)} bakış-noktası\n"
                 f"renk = o pozun getirdiği YENİ nokta sayısı (açık sarı = çok), "
                 f"oklar görüş ekseni, gri voksel bulutu şasinin kendisi",
                 fontsize=11, color=INK)
    fig.tight_layout()
    out = os.path.join(HERE, "fig_vp_plan.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


# --------------------------------------------------------------------------- #
# FİGÜR — tur sırası ve hop maliyeti
# --------------------------------------------------------------------------- #
def fig_order():
    plan = load_plan()
    vps = plan["ur_viewpoints"]
    P = np.array([v["position"] for v in vps])
    Q = [v["joint_positions"] for v in vps]
    costs = [joint_cost(Q[i], Q[i + 1]) for i in range(len(Q) - 1)]

    fig = plt.figure(figsize=(9.6, 4.8), dpi=170)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.15, 1.0])
    ax = fig.add_subplot(gs[0])
    basis = camera(24, 208)
    belief = chassis_voxels()
    draw_voxels(ax, belief["centers"], np.full_like(belief["rgb"], 165),
                belief["sizes"], basis, alpha=0.45)
    xy, _ = project(P, basis)
    ax.plot(xy[:, 0], xy[:, 1], color=ACCENT, lw=1.3, zorder=6)
    ax.scatter(xy[:, 0], xy[:, 1], s=26, c="white", edgecolors=ACCENT, linewidths=1.0,
               zorder=7)
    for i, (x, y) in enumerate(xy):
        ax.annotate(str(i + 1), (x, y), fontsize=5.5, ha="center", va="center",
                    zorder=8, color=INK)
    pts, _ = project(belief["centers"], basis)
    frame(ax, [pts, xy], pad=1.05)
    ax.set_title("tur sırası (1 → %d)" % len(vps), fontsize=10, color=INK)

    ax2 = fig.add_subplot(gs[1])
    x = np.arange(len(costs))
    ax2.bar(x, np.degrees(costs), color=[WARM if c > np.mean(costs) * 2 else ACCENT
                                         for c in costs])
    ax2.set_xticks(x[::2])
    ax2.set_xticklabels([f"{vps[i]['id']}→{vps[i + 1]['id']}" for i in x][::2],
                        rotation=90, fontsize=5)
    ax2.set_ylabel("hop maliyeti [° eşdeğer, ray 1 m = %.0f rad]" % RAIL_WEIGHT,
                   fontsize=8.5)
    ax2.grid(axis="y", alpha=0.3, lw=0.5)
    ax2.set_title("ardışık iki durak arası eklem-uzayı yolu", fontsize=10, color=INK)
    fig.suptitle("Eklem-uzayı sıralaması: tur, kol hiç home'a uğramadan akar\n"
                 "toplam %.0f° eşdeğer yol, hop ortalaması %.0f°" %
                 (np.degrees(sum(costs)), np.degrees(np.mean(costs))),
                 fontsize=11, color=INK)
    fig.tight_layout()
    out = os.path.join(HERE, "fig_vp_order.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


# --------------------------------------------------------------------------- #
# FİGÜR — marjinal kazanç ve birikimli kaplama
# --------------------------------------------------------------------------- #
def fig_gain():
    plan = load_plan()
    vps = sorted(plan["ur_viewpoints"], key=lambda v: v.get("rank", 0))
    gain = np.array([v.get("new_points_covered") or 0 for v in vps], float)
    cum = np.cumsum(gain)
    total_pts = cum[-1] / max(1e-9, plan["coverage_achieved"])

    fig, ax = plt.subplots(figsize=(8.6, 4.2), dpi=170)
    x = np.arange(1, len(vps) + 1)
    ax.bar(x, gain, color=ACCENT, label="yeni nokta (marjinal kazanç)")
    ax.set_xlabel("seçim sırası (greedy rank)", fontsize=9)
    ax.set_ylabel("yeni kaplanan hedef nokta", fontsize=9)
    ax.grid(axis="y", alpha=0.3, lw=0.5)
    ax2 = ax.twinx()
    ax2.plot(x, 100 * cum / total_pts, color=WARM, lw=1.6, marker="o", ms=2.5,
             label="birikimli kaplama")
    ax2.set_ylabel("birikimli kaplama [%]", fontsize=9, color=WARM)
    ax2.tick_params(axis="y", colors=WARM)
    floor = 0.0035 * total_pts
    ax.axhline(floor, color="#444444", ls="--", lw=1.0)
    ax.annotate(f"min_marginal_coverage tabanı ≈ {floor:.0f} nokta\n"
                f"(0.0035 x {total_pts:.0f} hedef)", (len(vps) * 0.55, floor),
                textcoords="offset points", xytext=(0, 10), fontsize=7.5, color="#444444")
    ax.set_title("Azalan getiri eğrisi: bakış-noktası sayısını taban belirler\n"
                 f"{len(vps)} bakış-noktası, planlayıcı tahmini kaplama "
                 f"%{100 * plan['coverage_achieved']:.1f}", fontsize=10.5, color=INK)
    fig.tight_layout()
    out = os.path.join(HERE, "fig_vp_gain.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


# --------------------------------------------------------------------------- #
# FİGÜR — gerçek/sim octomap ve tek kol ile çift kol karşılaştırması
# --------------------------------------------------------------------------- #
def _panel(ax, cov, basis, title):
    res = cov["res"]
    keys = np.array(list(cov["belief"].keys()), float) * res
    covered = np.array([k in cov["covered"] for k in cov["belief"]])
    rgb = np.where(covered[:, None], np.array([90, 160, 100]), np.array([200, 70, 60]))
    draw_voxels(ax, keys, rgb.astype(np.uint8), np.full(len(keys), res), basis)
    pts, _ = project(keys, basis)
    frame(ax, [pts], pad=1.03)
    ax.set_title(title, fontsize=10, color=INK)


def fig_octomap():
    cr = coverage(os.path.join(SINGLE, "real_data", "beliefMap_single_ur10e_real.ot"),
                  os.path.join(SINGLE, "real_data", "occupancyMap_single_ur10e_real.ot"))
    cs = coverage(os.path.join(SINGLE, "sim_data", "beliefMap_single_ur10e_sim.ot"),
                  os.path.join(SINGLE, "sim_data", "occupancyMap_single_ur10e_sim.ot"))
    fig, axes = plt.subplots(2, 2, figsize=(9.4, 8.0), dpi=170)
    for row, (cov, label) in enumerate(((cr, "Gerçek robot"), (cs, "Simülasyon"))):
        for col, (elev, azim, sub) in enumerate([(20, 210, "izometrik"),
                                                 (4, 275, "yan")]):
            _panel(axes[row, col], cov, camera(elev, azim),
                   f"{label} — {sub}  (kaplama %{100 * cov['frac']:.1f})")
    fig.suptitle("Tek-kol koşusunun octomap'i: yeşil = kaplanan şasi vokseli, "
                 "kırmızı = kaplanmayan\n2 cm çözünürlük, iki harita da AYNI kamera "
                 "açılarıyla çizildi", fontsize=11, color=INK)
    fig.tight_layout()
    out = os.path.join(HERE, "fig_vp_octomap.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out, f"(real %{100 * cr['frac']:.1f}, sim %{100 * cs['frac']:.1f})")


def fig_single_vs_multi():
    """Aynı şasi, aynı gerçek hücre: tek kol ile iki kol yan yana."""
    c1 = coverage(os.path.join(SINGLE, "real_data", "beliefMap_single_ur10e_real.ot"),
                  os.path.join(SINGLE, "real_data", "occupancyMap_single_ur10e_real.ot"))
    c2 = coverage(os.path.join(PCDS, "real_pcds", "beliefMap_real.ot"),
                  os.path.join(PCDS, "real_pcds", "occupancyMap_real.ot"))
    fig, axes = plt.subplots(1, 2, figsize=(9.4, 4.4), dpi=170)
    basis = camera(20, 210)
    _panel(axes[0], c1, basis, f"Tek kol (UR10e) — %{100 * c1['frac']:.1f}")
    _panel(axes[1], c2, basis, f"İki kol (UR10e + Kawasaki) — %{100 * c2['frac']:.1f}")
    fig.suptitle("İkinci kolun gerçek donanımdaki katkısı: aynı şasi, aynı ölçüt\n"
                 f"fark +{100 * (c2['frac'] - c1['frac']):.1f} puan — kırmızı "
                 "bölgelerin nereden kapandığına bakınız", fontsize=11, color=INK)
    fig.tight_layout()
    out = os.path.join(HERE, "fig_vp_single_vs_multi.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


def fig_part_coverage():
    cr = coverage(os.path.join(SINGLE, "real_data", "beliefMap_single_ur10e_real.ot"),
                  os.path.join(SINGLE, "real_data", "occupancyMap_single_ur10e_real.ot"))
    cs = coverage(os.path.join(SINGLE, "sim_data", "beliefMap_single_ur10e_sim.ot"),
                  os.path.join(SINGLE, "sim_data", "occupancyMap_single_ur10e_sim.ot"))
    pr, ps = per_color_coverage(cr), per_color_coverage(cs)
    colors = sorted(pr, key=lambda c: pr[c][0] / pr[c][1])
    real = [100 * pr[c][0] / pr[c][1] for c in colors]
    sim = [100 * ps.get(c, (0, 1))[0] / max(1, ps.get(c, (0, 1))[1]) for c in colors]
    fig, ax = plt.subplots(figsize=(9.6, 4.4), dpi=170)
    x = np.arange(len(colors))
    ax.bar(x - 0.2, real, 0.4, color="#c81e1e", label="Gerçek robot")
    ax.bar(x + 0.2, sim, 0.4, color="#1f6fd0", label="Simülasyon")
    ax.set_xticks([])
    ax.set_xlabel("şasi parçaları (en kötüden en iyiye)", fontsize=9)
    ax.set_ylabel("kaplama [%]", fontsize=9)
    ax.set_ylim(0, 105)
    ax.grid(axis="y", alpha=0.3, lw=0.5)
    ax.legend(fontsize=8.5, loc="lower right")
    n_bad = sum(1 for v in real if v < 50)
    ax.set_title("Tek-kol koşusunda parça başına kaplama — gerçek robot ile simülasyon\n"
                 f"%50'nin altında kalan parça sayısı: gerçek {n_bad}, "
                 f"sim {sum(1 for v in sim if v < 50)}", fontsize=10.5, color=INK)
    fig.tight_layout()
    out = os.path.join(HERE, "fig_vp_part_coverage.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


FIGURES = {
    "plan": fig_plan,
    "order": fig_order,
    "gain": fig_gain,
    "octomap": fig_octomap,
    "compare": fig_single_vs_multi,
    "parts": fig_part_coverage,
}


if __name__ == "__main__":
    wanted = [a for a in sys.argv[1:] if a in FIGURES] or list(FIGURES)
    for key in wanted:
        print(f"  {key} ...", flush=True)
        FIGURES[key]()
