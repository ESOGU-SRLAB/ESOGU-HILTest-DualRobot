#!/usr/bin/env python3
"""Kapı bölümünün figürlerini üretir (Şekil 13, 14, 15).

    python3 docs/figur_uret.py

NEDEN KENDİ İZDÜŞÜMÜM: bu makinede mplot3d KIRIK. Sistem paketi
/usr/lib/python3/dist-packages/mpl_toolkits gerçek bir paket (__init__.py var) ve pip
matplotlib'i gölgeliyor; Axes3D hiç import edilemiyor ve ROS'un sys.path sıralaması
yüzünden PYTHONPATH oyunları da işe yaramıyor. Bu yüzden ortografik kamera + derinliğe
göre sıralama (painter's algorithm) elle yazıldı. Küçük ve tam kontrol edilebilir.

Octomap .ot dosyaları da elle okunur (ColorOcTree binary düzeni read_ot içinde
belgelendi) -- kutuda octomap Python binding'i yok.
"""
import os
import struct

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

HERE = os.path.dirname(os.path.abspath(__file__))
DOORS_PLAN = "/home/cem/colcon_ws/src/doors_inspection/plans/doors_viewpoint_plan.json"
DOORS_MESH = os.path.expanduser("~/.cache/doors_inspection/doors_merged.stl")
PCDS = os.path.expanduser("~/colcon_ws/src/pcds/doors")


# --------------------------------------------------------------------------- #
# ortografik kamera
# --------------------------------------------------------------------------- #
def camera(elev_deg, azim_deg):
    """(right, up, forward) dünya koordinatlarında; forward kameradan sahneye bakar."""
    e, a = np.radians(elev_deg), np.radians(azim_deg)
    fwd = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    return right, up, fwd


def project(P, basis):
    right, up, fwd = basis
    return np.stack([P @ right, P @ up], axis=-1), P @ fwd


# --------------------------------------------------------------------------- #
# ColorOcTree (.ot) okuyucu
# --------------------------------------------------------------------------- #
def read_ot(path, occupied_only=True):
    """Düğüm başına düzen (octomap OcTreeBaseImpl::readNodesRecurs +
    ColorOcTreeNode::readData):  float32 log-odds | uint8 r,g,b | uint8 children maskesi.
    Kök merkezi (0,0,0); S boyutlu bir düğümün çocukları S/2 boyutunda ve ±S/4'te."""
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


# --------------------------------------------------------------------------- #
# çizim yardımcıları
# --------------------------------------------------------------------------- #
_CUBE_FACES = [
    ((0, 0, -1), [(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1)]),
    ((0, 0, +1), [(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]),
    ((0, -1, 0), [(-1, -1, -1), (1, -1, -1), (1, -1, 1), (-1, -1, 1)]),
    ((0, +1, 0), [(-1, 1, -1), (1, 1, -1), (1, 1, 1), (-1, 1, 1)]),
    ((-1, 0, 0), [(-1, -1, -1), (-1, 1, -1), (-1, 1, 1), (-1, -1, 1)]),
    ((+1, 0, 0), [(1, -1, -1), (1, 1, -1), (1, 1, 1), (1, -1, 1)]),
]


def draw_voxels(ax, centers, rgb, sizes, basis, alpha=1.0):
    """Kübleri görünür yüzleriyle çizer (arka yüz elenir, derinliğe göre sıralanır)."""
    right, up, fwd = basis
    polys, cols, depths = [], [], []
    half = sizes[:, None] / 2.0
    for normal, corners in _CUBE_FACES:
        nrm = np.array(normal, dtype=float)
        if nrm @ fwd >= 0:                      # kameradan uzağa bakan yüz
            continue
        shade = 0.55 + 0.45 * abs(nrm @ (-fwd))
        C = np.array(corners, dtype=float)      # (4,3) birim küpte
        for k in range(4):
            pass
        pts = centers[:, None, :] + C[None, :, :] * half[:, None, :]
        xy, _ = project(pts.reshape(-1, 3), basis)
        polys.append(xy.reshape(-1, 4, 2))
        cols.append(np.clip(rgb / 255.0 * shade, 0, 1))
        depths.append(centers @ fwd)
    P = np.concatenate(polys)
    C = np.concatenate(cols)
    D = np.concatenate(depths)
    o = np.argsort(-D)
    ax.add_collection(PolyCollection(P[o], facecolors=C[o], edgecolors="none",
                                     alpha=alpha))


def draw_mesh(ax, tris, normals, basis, base=(0.62, 0.66, 0.74)):
    right, up, fwd = basis
    xy, _ = project(tris.reshape(-1, 3), basis)
    xy = xy.reshape(-1, 3, 2)
    depth = tris.mean(axis=1) @ fwd
    o = np.argsort(-depth)
    shade = np.clip(0.35 + 0.65 * np.abs(normals @ (-fwd)), 0.25, 1.0)[o]
    cols = np.stack([base[0] * shade, base[1] * shade, base[2] * shade,
                     np.ones_like(shade)], axis=1)
    ax.add_collection(PolyCollection(xy[o], facecolors=cols, edgecolors="none"))


def frame(ax, pts_list, pad=1.04):
    allp = np.concatenate([p for p in pts_list if len(p)])
    lo, hi = allp.min(0), allp.max(0)
    c, r = (lo + hi) / 2.0, (hi - lo).max() / 2.0 * pad
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_aspect("equal")
    ax.axis("off")


# --------------------------------------------------------------------------- #
# Şekil 11 — kapı planı, dört açıdan
# --------------------------------------------------------------------------- #
def fig_viewpoints():
    import json
    import trimesh

    plan = json.load(open(DOORS_PLAN))
    m = trimesh.load(DOORS_MESH)
    try:
        m = m.simplify_quadric_decimation(face_count=20000)
    except Exception:
        pass
    tris, normals = m.triangles, m.face_normals

    views = [(14, 125, "Ön-üst (UR tarafı, +X)"),
             (14, -55, "Arka-üst (Kawasaki tarafı, −X)"),
             (72, 125, "Tepeden — iki kolun karşılıklı duruşu"),
             (2, 180, "Yandan (−Y) — yükseklik dağılımı")]

    fig, axes = plt.subplots(2, 2, figsize=(13.2, 12.4))
    for ax, (elev, azim, title) in zip(axes.ravel(), views):
        basis = camera(elev, azim)
        draw_mesh(ax, tris, normals, basis)
        pts = [tris.reshape(-1, 3)]
        for key, col in (("ur_viewpoints", "#1f6fd0"),
                         ("kawasaki_viewpoints", "#c81e1e")):
            vps = plan[key]
            P = np.array([v["position"] for v in vps])
            pts.append(P)
            pxy, _ = project(P, basis)
            ax.scatter(pxy[:, 0], pxy[:, 1], c=col, s=78, zorder=5,
                       edgecolor="white", linewidth=1.0)
            for v, p2 in zip(vps, pxy):
                p = np.array(v["position"])
                R = np.array(v["rotation"])
                tip, _ = project((p + R[:, 2] * 0.32)[None, :], basis)
                ax.annotate("", xy=tip[0], xytext=p2, zorder=4,
                            arrowprops=dict(arrowstyle="-|>", color=col, lw=1.5))
                ax.text(p2[0], p2[1] + 0.05, v["id"].split("_")[-1], fontsize=6.5,
                        color=col, ha="center", zorder=6, fontweight="bold")
        # çerçeveyi mesh + bakış-noktalarını kapsayacak şekilde kur
        proj_pts = []
        for P in pts:
            xy, _ = project(P, basis)
            proj_pts.append(np.column_stack([xy, np.zeros(len(xy))]))
        frame(ax, proj_pts)
        ax.set_title(title, fontsize=11.5, pad=6)

    fig.suptitle("Kapı planı — 9 UR10e (mavi) + 3 Kawasaki (kırmızı), dört bakış açısı",
                 fontsize=14, y=0.985)
    fig.tight_layout(rect=(0, 0, 1, 0.972))
    out = os.path.join(HERE, "fig_doors_viewpoints.png")
    fig.savefig(out, dpi=125, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


# --------------------------------------------------------------------------- #
# Şekil 12 — octomap çıktıları, sim vs gerçek
# --------------------------------------------------------------------------- #
def fig_octomaps(sim_occ, real_occ, sim_pct, real_pct):
    maps = [(sim_occ, f"SİMÜLASYON — %{sim_pct:.1f} kaplama"),
            (real_occ, f"GERÇEK ROBOT — %{real_pct:.1f} kaplama (snap 2 + cloud_fix)")]
    views = [(12, 125, "+X yüzü (UR'nin gördüğü)"),
             (12, -55, "−X yüzü (Kawasaki'nin gördüğü)")]

    fig, axes = plt.subplots(2, 2, figsize=(13.2, 12.0))
    for r, (path, row_title) in enumerate(maps):
        d = read_ot(path)
        for c, (elev, azim, vtitle) in enumerate(views):
            ax = axes[r, c]
            basis = camera(elev, azim)
            draw_voxels(ax, d["centers"], d["rgb"], d["sizes"], basis)
            xy, _ = project(d["centers"], basis)
            frame(ax, [np.column_stack([xy, np.zeros(len(xy))])], pad=1.08)
            ax.set_title(f"{row_title}\n{vtitle}   ({len(d['centers'])} voksel)",
                         fontsize=10.5, pad=6)
    fig.suptitle("Kapı octomap rekonstrüksiyonu — yakalanan bulutlardan, 2 cm voksel\n"
                 "Renkler belief haritasının 3×4 yama ızgarasından gelir; boşluk = "
                 "hiç görülmemiş yüzey",
                 fontsize=13.5, y=0.985)
    fig.tight_layout(rect=(0, 0, 1, 0.958))
    out = os.path.join(HERE, "fig_doors_octomap.png")
    fig.savefig(out, dpi=125, bbox_inches="tight")
    plt.close(fig)
    print("yazıldı:", out)


# --------------------------------------------------------------------------- #
# Şekil 13 — ölçülen kaplama karşılaştırması
# --------------------------------------------------------------------------- #
def fig_coverage():
    labels = ["ham\n(düzeltmesiz)", "snap r=2", "cloud_fix", "snap r=2\n+ cloud_fix"]
    real = [36.0, 52.1, 49.6, 62.9]
    sim = [93.0, 93.1, 23.6, 50.5]

    fig, ax = plt.subplots(figsize=(9.4, 5.4))
    x = np.arange(len(labels))
    w = 0.36
    ax.bar(x - w / 2, real, w, label="Gerçek robot", color="#c81e1e")
    ax.bar(x + w / 2, sim, w, label="Simülasyon", color="#1f6fd0")
    for i, v in enumerate(real):
        ax.text(x[i] - w / 2, v + 1.5, f"{v:.1f}", ha="center", fontsize=10)
    for i, v in enumerate(sim):
        ax.text(x[i] + w / 2, v + 1.5, f"{v:.1f}", ha="center", fontsize=10)
    ax.axvspan(1.5, 3.5, color="#999999", alpha=0.12)
    ax.text(2.5, 88, "cloud_fix YALNIZ GERÇEK içindir —\nsim'e uygulanınca çökertir",
            ha="center", fontsize=9.5, color="#444444", style="italic")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=10)
    ax.set_ylabel("Octomap kaplaması (%)", fontsize=11)
    ax.set_ylim(0, 100)
    ax.set_title("Kapı octomap kaplaması — hizalama düzeltmelerinin ÖLÇÜLEN etkisi\n"
                 "(2 cm voksel, builder'ın kendi raporundan)", fontsize=12)
    ax.legend(fontsize=10, loc="lower left")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = os.path.join(HERE, "fig_doors_coverage.png")
    fig.savefig(out, dpi=145)
    plt.close(fig)
    print("yazıldı:", out)


if __name__ == "__main__":
    import sys
    sim_occ = sys.argv[1] if len(sys.argv) > 1 else f"{PCDS}/sim_pcds/occupancyMap_doors_sim.ot"
    real_occ = sys.argv[2] if len(sys.argv) > 2 else f"{PCDS}/real_pcds/occupancyMap_doors_real.ot"
    fig_viewpoints()
    fig_octomaps(sim_occ, real_occ, sim_pct=93.0, real_pct=62.9)
    fig_coverage()
