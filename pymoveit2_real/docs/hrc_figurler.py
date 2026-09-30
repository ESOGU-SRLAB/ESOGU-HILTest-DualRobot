#!/usr/bin/env python3
"""İnsan-robot işbirliği (vidalama) raporunun figürlerini üretir.

    python3 docs/hrc_figurler.py            # bütün fig_hrc_*.png dosyaları
    python3 docs/hrc_figurler.py tcp gpio   # yalnız adı geçenler

NEDEN KENDİ İZDÜŞÜMÜ: bu kutuda mplot3d KIRIK
(/usr/lib/python3/dist-packages/mpl_toolkits pip matplotlib'i gölgeliyor, Axes3D
import edilemiyor). Bu yüzden ortografik kamera + painter's-algorithm sıralaması
elle yazıldı; multirobot_viewpoint_planner/docs/figur_uret.py'deki yaklaşımın aynısı.

VERİ KAYNAĞI, ELLE KOPYA DEĞİL:
  * Poz/geometri  -> whole_cell_hw.urdf.xacro (use_gripper:=true) xacro ile açılıp
                     ayrıştırılır; FK burada hesaplanır.
  * Waypoint'ler  -> examples/human_robot_collaboration_scenario.py'nin main()
                     bloğu ayrıştırılıp exec edilir. Senaryodaki bir açı değişince
                     figürler kendiliğinden değişir; rapor da onları yeniden okur.
"""
import math
import os
import re
import subprocess
import sys
import textwrap
import xml.etree.ElementTree as ET

import numpy as np
import warnings

import matplotlib
matplotlib.use("Agg")
# mplot3d bu kutuda import edilemiyor (dosya başındaki nota bakın); uyarısı gürültü.
warnings.filterwarnings("ignore", message="Unable to import Axes3D")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection, LineCollection
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC = os.path.dirname(PKG)
SCENARIO = os.path.join(PKG, "examples", "human_robot_collaboration_scenario.py")
XACRO = os.path.join(
    SRC, "Universal_Robots_ROS2_Tutorials", "my_robot_cell", "my_robot_cell_control",
    "urdf", "whole_cell_hw.urdf.xacro")
CACHE = os.path.expanduser("~/.cache/hrc_report")

ARM_JOINTS = [
    "ur10e_base_to_robot_mount", "ur10e_shoulder_pan_joint",
    "ur10e_shoulder_lift_joint", "ur10e_elbow_joint",
    "ur10e_wrist_1_joint", "ur10e_wrist_2_joint", "ur10e_wrist_3_joint",
]
JOINT_SHORT = ["ray", "base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
TOOL_LINK = "ur10e_gripper_base_link"
# Vidalama ucunun gripper frame'indeki yeri: senaryonun attach ettiği silindirin
# merkezi (attach_screwdriver -> add_collision_cylinder position).
SCREWDRIVER_IN_GRIPPER = np.array([0.077, 0.16, -0.026])

INK = "#22252a"
ACCENT = "#1f6feb"
WARM = "#d9480f"
GREEN = "#2f9e44"
GREY = "#9aa0a6"


# --------------------------------------------------------------------------- #
# URDF + ileri kinematik
# --------------------------------------------------------------------------- #
def urdf_path():
    """xacro'yu açar (bir kez), sonuç ~/.cache altında tutulur."""
    os.makedirs(CACHE, exist_ok=True)
    out = os.path.join(CACHE, "whole_cell_hw_gripper.urdf")
    if (os.path.exists(out) and os.path.getsize(out) > 0
            and os.path.getmtime(out) > os.path.getmtime(XACRO)):
        return out
    # xacro paketleri AMENT_PREFIX_PATH'ten bulur; bu script çıplak bir kabuktan da
    # çalıştırılabildiği için workspace'i kendisi source eden bir kabuk kullanıyoruz
    # (aksi halde 'package mobile_manipulator_description not found' ile düşer).
    cmd = ("source /opt/ros/humble/setup.bash && "
           "source ~/colcon_ws/install/setup.bash >/dev/null 2>&1; "
           f"xacro '{XACRO}' use_gripper:=true")
    with open(out, "w") as fh:
        subprocess.run(["bash", "-c", cmd], stdout=fh, check=True)
    return out


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def xf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def _origin(el):
    o = el.find("origin") if el is not None else None
    xyz = [float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None else [0, 0, 0])]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0").split() if o is not None else [0, 0, 0])]
    return xyz, rpy


class Cell:
    """whole_cell_hw URDF'inin ihtiyacımız olan kadarı: eklem zinciri + görsel geometri."""

    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.joints = {}
        self.children = {}
        for j in root.findall("joint"):
            name = j.get("name")
            xyz, rpy = _origin(j)
            axis_el = j.find("axis")
            axis = np.array([float(v) for v in axis_el.get("xyz").split()]) if axis_el is not None \
                else np.array([0.0, 0.0, 1.0])
            mimic = j.find("mimic")
            self.joints[name] = dict(
                type=j.get("type"), parent=j.find("parent").get("link"),
                child=j.find("child").get("link"), origin=xf(xyz, rpy), axis=axis,
                mimic=(mimic.get("joint"), float(mimic.get("multiplier", 1.0)))
                if mimic is not None else None)
            self.children.setdefault(j.find("parent").get("link"), []).append(name)
        self.links = {l.get("name"): l for l in root.findall("link")}
        parents = {d["child"] for d in self.joints.values()}
        self.root_link = next(n for n in self.links if n not in parents)

    def fk(self, values):
        """values: eklem adı -> konum (rad|m). Döner: link adı -> 4x4 dünya dönüşümü."""
        poses = {self.root_link: np.eye(4)}
        stack = [self.root_link]
        while stack:
            link = stack.pop()
            for jname in self.children.get(link, []):
                j = self.joints[jname]
                q = values.get(jname, 0.0)
                if j["mimic"]:
                    src, mult = j["mimic"]
                    q = values.get(src, 0.0) * mult
                T = poses[link] @ j["origin"]
                if j["type"] == "prismatic":
                    T = T @ xf(j["axis"] * q, (0, 0, 0))
                elif j["type"] in ("revolute", "continuous"):
                    a = j["axis"] / np.linalg.norm(j["axis"])
                    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                    R = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)
                    S = np.eye(4)
                    S[:3, :3] = R
                    T = T @ S
                poses[j["child"]] = T
                stack.append(j["child"])
        return poses

    def visuals(self, link):
        """Bir linkin görsel geometrileri: (yerel dönüşüm, tür, parametre)."""
        out = []
        for v in self.links[link].findall("visual"):
            xyz, rpy = _origin(v)
            T = xf(xyz, rpy)
            g = v.find("geometry")
            mesh, box, cyl = g.find("mesh"), g.find("box"), g.find("cylinder")
            if mesh is not None:
                scale = [float(s) for s in mesh.get("scale", "1 1 1").split()]
                out.append((T, "mesh", (mesh.get("filename"), scale)))
            elif box is not None:
                out.append((T, "box", [float(s) for s in box.get("size").split()]))
            elif cyl is not None:
                out.append((T, "cyl", (float(cyl.get("radius")), float(cyl.get("length")))))
        return out


# --------------------------------------------------------------------------- #
# üçgen ağları: yükle, seyrelt, önbellekle
# --------------------------------------------------------------------------- #
def _decimate(verts, faces, budget):
    if len(faces) <= budget:
        return verts, faces
    import open3d as o3d  # yalnız seyreltme için; render tamamen numpy
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(verts),
                                  o3d.utility.Vector3iVector(faces))
    m = m.simplify_quadric_decimation(int(budget))
    return np.asarray(m.vertices), np.asarray(m.triangles)


def mesh_tris(filename, scale, budget):
    """STL/DAE -> (V, F), seyreltilmiş ve ~/.cache altında npz olarak saklanmış."""
    path = filename.replace("file://", "")
    if path.startswith("package://"):
        return None
    if not os.path.exists(path):
        return None
    key = re.sub(r"[^A-Za-z0-9]+", "_", path)[-90:] + f"_{budget}"
    npz = os.path.join(CACHE, key + ".npz")
    if os.path.exists(npz):
        d = np.load(npz)
        V, F = d["V"], d["F"]
    else:
        import trimesh
        m = trimesh.load(path, force="mesh")
        V, F = _decimate(np.asarray(m.vertices), np.asarray(m.faces), budget)
        os.makedirs(CACHE, exist_ok=True)
        np.savez_compressed(npz, V=V, F=F)
    return V * np.asarray(scale), F


def box_tris(size):
    sx, sy, sz = [s / 2.0 for s in size]
    V = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    F = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                  [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return V, F


def cyl_tris(radius, length, n=16):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    top = np.stack([radius * np.cos(a), radius * np.sin(a), np.full(n, length / 2)], 1)
    bot = np.stack([radius * np.cos(a), radius * np.sin(a), np.full(n, -length / 2)], 1)
    V = np.vstack([top, bot, [[0, 0, length / 2], [0, 0, -length / 2]]])
    F = []
    for i in range(n):
        k = (i + 1) % n
        F += [[i, k, n + k], [i, n + k, n + i], [2 * n, k, i], [2 * n + 1, n + i, n + k]]
    return V, np.array(F)


# --------------------------------------------------------------------------- #
# ortografik kamera + painter's algorithm
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


def draw_tris(ax, parts, basis, light=(0.3, 0.5, 0.8)):
    """parts: (V_dünya, F, temel_renk, alpha) listesi. Tek PolyCollection'da çizer."""
    polys, colors, depth = [], [], []
    L = np.asarray(light, float)
    L /= np.linalg.norm(L)
    for V, F, base, alpha in parts:
        tri = V[F]                                    # (n,3,3)
        xy, dz = project(tri.reshape(-1, 3), basis)
        xy = xy.reshape(-1, 3, 2)
        dz = dz.reshape(-1, 3).mean(axis=1)
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        ln = np.linalg.norm(n, axis=1)
        ln[ln == 0] = 1.0
        shade = 0.45 + 0.55 * np.clip((n / ln[:, None]) @ L, 0, 1)
        rgb = np.asarray(matplotlib.colors.to_rgb(base))
        polys.append(xy)
        colors.append(np.clip(shade[:, None] * rgb[None, :], 0, 1))
        depth.append(dz)
    polys = np.concatenate(polys)
    colors = np.concatenate(colors)
    depth = np.concatenate(depth)
    order = np.argsort(-depth)                        # uzaktan yakına
    pc = PolyCollection(polys[order], facecolors=colors[order], edgecolors="none",
                        antialiased=False)
    ax.add_collection(pc)
    return polys.reshape(-1, 2)


def fit(ax, pts, pad=1.05, equal=True):
    pts = np.asarray(pts).reshape(-1, 2)
    lo, hi = pts.min(0), pts.max(0)
    c, half = (lo + hi) / 2, (hi - lo).max() / 2 * pad
    if equal:
        ax.set_xlim(c[0] - half, c[0] + half)
        ax.set_ylim(c[1] - half, c[1] + half)
    ax.set_aspect("equal")
    ax.axis("off")


# --------------------------------------------------------------------------- #
# sahne: hangi linkler, hangi bütçe, hangi renk
# --------------------------------------------------------------------------- #
ARM_LINKS = ("ur10e_base", "ur10e_shoulder", "ur10e_upper_arm", "ur10e_forearm",
             "ur10e_wrist_1", "ur10e_wrist_2", "ur10e_wrist_3", "ur10e_robot_mount",
             "ur10e_flange", "ur10e_tool0", "ur10e_cable_channel", "ur10e_sick_camera")
GRIPPER_LINKS = ("ur10e_gripper_base_link", "ur10e_gripper_upper_1",
                 "ur10e_gripper_lower_1")

# Hücrenin bu senaryoya ait parçaları URDF'te ayrı linkler olarak duruyor; renk ve
# üçgen bütçesi buradan seçilir. (renk, bütçe)
STYLE = {
    "ur10e_screwdriver": ("#e8590c", 4000),      # vidalama aleti ve standı
    "ur10e_screw_feeder": ("#2f9e44", 2500),     # vida besleyici
    "human_link": ("#7048e8", 2500),             # operatör mankeni
    "light_curtain_link": ("#f59f00", 800),      # ışık perdesi
    "ur10e_alumunium_table": ("#868e96", 3000),  # iş tezgâhı
    "ur10e_stackable_bin": ("#adb5bd", 600),
    "ur10e_conveyor_belt": ("#adb5bd", 600),
}
ARM_COLOR = "#c8ccd4"
GRIPPER_COLOR = "#7a8290"
SKIP_PREFIX = ("base_link", "link", "ota_", "wheel_", "caster_", "ifarlab_ray", "sim_")


def scene_parts(cell, poses, **kw):
    return [pt for pt, _ in scene_parts_named(cell, poses, **kw)]


def scene_parts_named(cell, poses, links=None, budget_arm=2500, budget_chassis=250,
                chassis=True, region=None):
    """Sahneyi üçgen kümelerine çevirir.

    links   : yalnız bu linkler (None -> UR tarafındaki her şey)
    region  : (xmin, xmax, ymin, ymax) dünya kutusu; dışında kalan parçalar atılır
              (şasinin uzak ucu figürü küçültmesin diye).
    """
    parts = []
    for link in cell.links:
        if link not in poses:
            continue
        if links is not None:
            if link not in links:
                continue
        else:
            if link.startswith(SKIP_PREFIX) or "kawasaki" in link:
                continue
            if "chassis" in link and not chassis:
                continue
        if link in STYLE:
            color, budget = STYLE[link]
        elif link.startswith(ARM_LINKS):
            color, budget = ARM_COLOR, budget_arm
        elif link in GRIPPER_LINKS:
            color, budget = GRIPPER_COLOR, 4000
        elif "chassis" in link:
            color, budget = "#8d9099", budget_chassis
        elif link == "ur10e_table":
            color, budget = "#6f757e", 1500
        else:
            color, budget = "#9aa0a6", 500
        for T_local, kind, spec in cell.visuals(link):
            if kind == "mesh":
                got = mesh_tris(spec[0], spec[1], budget)
                if got is None:
                    continue
                V, F = got
            elif kind == "box":
                V, F = box_tris(spec)
            else:
                V, F = cyl_tris(*spec)
            T = poses[link] @ T_local
            Vw = (T[:3, :3] @ V.T).T + T[:3, 3]
            if region is not None and not (link.startswith(ARM_LINKS)
                                           or link in GRIPPER_LINKS):
                c = Vw.mean(0)
                if not (region[0] <= c[0] <= region[1] and region[2] <= c[1] <= region[3]):
                    continue
            parts.append(((Vw, F, color, 1.0), link))
    return parts


# --------------------------------------------------------------------------- #
# senaryo kaynağından waypoint'ler ve tur listesi
# --------------------------------------------------------------------------- #
class _Stub:
    screw_speed = 0.01
    travel_speed = 0.1
    gripper_open_position = 0.0
    gripper_release_position = 0.003
    gripper_closed_position = 0.026


def scenario_data():
    """main() içindeki waypoint tanımlarını ve tur listesini kaynaktan okur."""
    src = open(SCENARIO).read()
    start = src.index("    FIRST = 1.85")
    end = src.index("    try:", start)
    block = textwrap.dedent(src[start:end])
    ns = {"math": math, "robot_controller": _Stub()}
    exec(compile(block, SCENARIO, "exec"), ns)
    names = [k for k, v in ns.items()
             if isinstance(v, list) and len(v) == 7 and all(isinstance(x, float) for x in v)]
    return ns, {n: ns[n] for n in names}, ns["safe_joint_configurations"]


def joint_values(q, gripper=0.0):
    v = dict(zip(ARM_JOINTS, q))
    v["ur10e_gripper_joint"] = gripper
    return v


def tcp_of(cell, q, gripper=0.0):
    """Vidalama ucunun dünya konumu (attach edilen silindirin merkezi)."""
    T = cell.fk(joint_values(q, gripper))[TOOL_LINK]
    return T[:3, :3] @ SCREWDRIVER_IN_GRIPPER + T[:3, 3]


# --------------------------------------------------------------------------- #
# eklem hız sınırları (süre tahmini için)
#   ur10e_*: my_robot_cell_control/config/ur10e/joint_limits.yaml
#            (base/shoulder/elbow 120 °/s, wrist'ler 180 °/s)
#   ray    : whole_cell_hw URDF'indeki ur10e_base_to_robot_mount limiti, 1.25 m/s
# --------------------------------------------------------------------------- #
VMAX = np.array([1.25] + [math.radians(v) for v in (120, 120, 180, 180, 180, 180)])


def seq_steps(seq):
    """Tur listesini (hareketler + olaylar) düz bir adım dizisine çevirir."""
    steps = []
    for item in seq:
        if isinstance(item, dict):
            if "joints" in item:
                steps.append(dict(kind="move", q=item["joints"],
                                  speed=item.get("speed"), label="screw"))
            else:
                key = next(iter(item))
                steps.append(dict(kind="event", key=key, value=item[key]))
        else:
            steps.append(dict(kind="move", q=item, speed=None, label="travel"))
    return steps


def name_of(wps, q, tol=1e-9):
    for n, v in wps.items():
        if all(abs(a - b) < tol for a, b in zip(q, v)):
            return n
    return "?"


def seg_duration(q0, q1, scale):
    """Kaba süre tahmini: en yavaş eklemin yolu / (limit * ölçek). Profil üçgen/
    trapez olduğu için gerçek süre bundan büyüktür; oran karşılaştırması için yeter."""
    dq = np.abs(np.asarray(q1) - np.asarray(q0))
    return float(np.max(dq / (VMAX * scale)))


# --------------------------------------------------------------------------- #
# FİGÜR 1 — hücre genel görünümü
# --------------------------------------------------------------------------- #
def fig_cell(cell, wps, seq):
    poses = cell.fk(joint_values(wps["holdScrewer"], gripper=0.026))
    fig, axes = plt.subplots(1, 2, figsize=(8.4, 4.6), dpi=200)

    # --- sol: hücrenin tamamı ---
    ax = axes[0]
    basis = camera(16, 208)
    parts = scene_parts(cell, poses, budget_arm=1500, budget_chassis=120)
    pts = draw_tris(ax, parts, basis)
    for link, label, off in (("human_link", "operatör", (14, 12)),
                             ("light_curtain_link", "ışık perdesi", (-6, -16))):
        P = np.array([poses[link][:3, 3]])
        if link == "human_link":
            P = np.array([[1.65, 2.45, 1.0]])
        elif link == "light_curtain_link":
            P = np.array([[-1.9, 3.25, 0.4]])
        xy, _ = project(P, basis)
        color = STYLE[link][0]
        ax.scatter(xy[:, 0], xy[:, 1], s=16, c=color, zorder=6)
        ax.annotate(label, xy[0], textcoords="offset points", xytext=off,
                    fontsize=7.5, color=color, weight="bold")
    fit(ax, pts)
    ax.set_title("hücrenin tamamı", fontsize=9, color=INK)

    # --- sağ: çalışma alanı yakın plan ---
    ax = axes[1]
    # Operatör mankeni kameranın önünde kalıyor; yakın planda o ve şasi çıkarılır
    # (vidalar şasinin değil, alüminyum tezgâhın üstünde).
    basis = camera(22, 36)
    region = (0.1, 1.45, 1.7, 3.3)
    parts = [pt for pt, link in scene_parts_named(cell, poses, budget_arm=4000,
                                                  chassis=False, region=region)
             if link != "human_link" and not link.startswith("barrier")]
    pts = draw_tris(ax, parts, basis)

    marks = [("vidalama aleti standı\n(holdScrewer)", tcp_of(cell, wps["holdScrewer"], 0.026), ACCENT, (14, 14)),
             ("vida besleyici\n(tookScrew)", tcp_of(cell, wps["tookScrew"], 0.026), GREEN, (-96, 26)),
             ("1. vida", tcp_of(cell, wps["firstOpt"]), WARM, (-40, -22)),
             ("2. vida", tcp_of(cell, wps["secondOpt"]), WARM, (-40, -8)),
             ("3. vida", tcp_of(cell, wps["thirdOpt"]), WARM, (16, 6)),
             ("4. vida", tcp_of(cell, wps["fourthOpt"]), WARM, (16, -14))]
    P = np.array([m[1] for m in marks])
    xy, _ = project(P, basis)
    for (label, _, color, off), (x, y) in zip(marks, xy):
        ax.scatter([x], [y], s=34, c=color, zorder=6, edgecolors="white", linewidths=0.8)
        ax.annotate(label, (x, y), textcoords="offset points", xytext=off,
                    fontsize=7.2, color=color, weight="bold",
                    arrowprops=dict(arrowstyle="-", color=color, lw=0.7,
                                    shrinkA=0, shrinkB=3))
    fit(ax, np.vstack([pts, xy]))
    ax.set_title("çalışma alanı (poz: holdScrewer)", fontsize=9, color=INK)

    fig.suptitle("IFARLAB hücresi — insan-robot işbirlikli vidalama senaryosunun sahnesi\n"
                 "UR10e 2 m'lik Festo rayında, OnRobot 2FG7 gripper, vidalama aleti standı ve vida besleyici",
                 fontsize=10, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_hrc_cell.png"), bbox_inches="tight")
    plt.close(fig)


# --------------------------------------------------------------------------- #
# FİGÜR 2 — dört anahtar poz
# --------------------------------------------------------------------------- #
def fig_postures(cell, wps, seq):
    picks = [("holdScrewer — aleti kavra", "holdScrewer", 0.026),
             ("tookScrew — vida al", "tookScrew", 0.026),
             ("firstTop — 1. vida üstü", "firstTop", 0.026),
             ("thirdOpt — 3. vida sıkımı", "thirdOpt", 0.026)]
    fig, axes = plt.subplots(2, 2, figsize=(7.6, 6.6), dpi=200)
    basis = camera(20, 36)
    for ax, (title, key, grip) in zip(axes.ravel(), picks):
        poses = cell.fk(joint_values(wps[key], gripper=grip))
        parts = [pt for pt, link in scene_parts_named(
            cell, poses, budget_arm=2500, chassis=False, region=(0.1, 1.45, 1.7, 3.3))
            if link != "human_link" and not link.startswith("barrier")]
        pts = draw_tris(ax, parts, basis)
        tip, _ = project(np.array([tcp_of(cell, wps[key], grip)]), basis)
        ax.scatter(tip[:, 0], tip[:, 1], s=30, c=WARM, zorder=5,
                   edgecolors="white", linewidths=0.7)
        fit(ax, pts)
        ax.set_title(title, fontsize=9, color=INK)
    fig.suptitle("Turun dört anahtar pozu (URDF'ten ileri kinematik; kırmızı nokta = vidalama ucu)",
                 fontsize=10, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_hrc_postures.png"), bbox_inches="tight")
    plt.close(fig)


# --------------------------------------------------------------------------- #
# FİGÜR 3 — vidalama ucunun izlediği yol
# --------------------------------------------------------------------------- #
def fig_tcp_path(cell, wps, seq):
    steps = [s for s in seq_steps(seq) if s["kind"] == "move"]
    P = np.array([tcp_of(cell, s["q"]) for s in steps])
    slow = np.array([s["speed"] is not None for s in steps])
    labels = [name_of(wps, s["q"]) for s in steps]
    poses = cell.fk(joint_values(wps["holdScrewer"]))
    bg_parts = [pt for pt, link in scene_parts_named(
        cell, poses, budget_arm=1, chassis=False, region=(0.2, 1.3, 2.0, 3.2))
        if link not in ("human_link",) and not link.startswith(("barrier", "ur10e_base",
        "ur10e_shoulder", "ur10e_upper", "ur10e_forearm", "ur10e_wrist", "ur10e_robot_mount",
        "ur10e_gripper", "ur10e_flange", "ur10e_tool0", "ur10e_cable", "ur10e_sick"))]

    named = {"holdScrewer": (10, 10), "tookScrew": (-52, 12), "firstTop": (-38, -16),
             "secondTop": (6, -16), "thirdTop": (8, 10), "fourthTop": (10, -14),
             "frontOfScrewer": (8, 16), "safeWaypoint": (-56, 8)}
    fig, axes = plt.subplots(1, 2, figsize=(8.2, 4.4), dpi=200)
    for ax, (elev, azim, vtitle) in zip(axes, [(22, 36, "eğik görünüm"),
                                               (86, 36, "üstten görünüm")]):
        basis = camera(elev, azim)
        bg = draw_tris(ax, bg_parts, basis)
        xy, _ = project(P, basis)
        segs = np.stack([xy[:-1], xy[1:]], axis=1)
        ax.add_collection(LineCollection(
            segs, colors=[WARM if slow[i + 1] else ACCENT for i in range(len(segs))],
            linewidths=[2.6 if slow[i + 1] else 1.4 for i in range(len(segs))], zorder=6))
        ax.scatter(xy[:, 0], xy[:, 1], s=16, c="white", edgecolors=INK,
                   linewidths=0.7, zorder=7)
        seen = set()
        for i, (x, y) in enumerate(xy):
            if elev > 60:      # üstten görünümde etiketler üst üste biniyor
                break
            if labels[i] in named and labels[i] not in seen:
                seen.add(labels[i])
                ax.annotate(labels[i], (x, y), textcoords="offset points",
                            xytext=named[labels[i]], fontsize=6.8, color=INK, zorder=8,
                            arrowprops=dict(arrowstyle="-", color=GREY, lw=0.6,
                                            shrinkA=0, shrinkB=2))
        # yola göre kadrajla: arka plan sahneyi küçültmesin
        fit(ax, xy, pad=1.45)
        ax.set_title(vtitle, fontsize=9, color=INK)
    total = float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())
    fig.suptitle("Vidalama ucunun tur boyunca izlediği yol — toplam %.2f m\n"
                 "mavi: seyir (hız ölçeği 0.1) · kırmızı: vidalama geçişi (0.01) · "
                 "arka plan: tezgâh ve vida besleyici" % total,
                 fontsize=10, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_hrc_tcp_path.png"), bbox_inches="tight")
    plt.close(fig)
    return total


# --------------------------------------------------------------------------- #
# FİGÜR 4 — eklem profili
# --------------------------------------------------------------------------- #
def fig_joint_profile(cell, wps, seq):
    steps = seq_steps(seq)
    moves, events = [], []
    for s in steps:
        if s["kind"] == "move":
            moves.append(s)
        else:
            events.append((len(moves) - 0.5, s["key"]))
    Q = np.array([m["q"] for m in moves])
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(7.8, 5.4), dpi=200,
                                  gridspec_kw=dict(height_ratios=[3, 1]), sharex=True)
    x = np.arange(len(moves))
    cmap = plt.get_cmap("tab10")
    for k in range(1, 7):
        ax.plot(x, np.degrees(Q[:, k]), lw=1.4, color=cmap((k - 1) % 10),
                label=JOINT_SHORT[k], marker="o", ms=2.5)
    ax.set_ylabel("eklem açısı [°]", fontsize=9)
    ax.legend(ncol=6, fontsize=7.5, frameon=False, loc="upper center")
    ax.grid(alpha=0.25, lw=0.5)
    for xe, key in events:
        color = {"wait_green_button": GREEN, "screwdriver": WARM}.get(key, GREY)
        ax.axvline(xe, color=color, lw=0.8, alpha=0.75,
                   ls="-" if key in ("wait_green_button",) else (0, (2, 2)))

    ax2.step(x, [m["speed"] if m["speed"] else 0.1 for m in moves], where="mid",
             color=INK, lw=1.2)
    ax2.set_yscale("log")
    ax2.set_ylabel("hız ölçeği", fontsize=9)
    ax2.set_xlabel("hareket adımı (tur sırası)", fontsize=9)
    ax2.grid(alpha=0.25, lw=0.5)
    ax2.set_xticks(x[::2])
    ax2.set_xticklabels([name_of(wps, m["q"]) for m in moves][::2], rotation=60,
                        ha="right", fontsize=6)
    fig.suptitle("Tur boyunca eklem açıları ve hız ölçeği\n"
                 "yeşil çizgi: yeşil buton beklemesi · kesikli kırmızı: vidalama çıkışı anahtarlanıyor",
                 fontsize=10, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_hrc_joint_profile.png"), bbox_inches="tight")
    plt.close(fig)


# --------------------------------------------------------------------------- #
# FİGÜR 5 — tur zaman çizgisi (tahmini süreler)
# --------------------------------------------------------------------------- #
def fig_timeline(cell, wps, seq):
    steps = seq_steps(seq)
    rows, t = [], 0.0
    q_prev = None
    WAIT_BETWEEN = 0.5      # safe_joint_sequence(..., wait_time=0.5)
    for s in steps:
        if s["kind"] == "move":
            scale = s["speed"] or 0.1
            dt = seg_duration(q_prev, s["q"], scale) if q_prev is not None else 2.0
            rows.append((t, dt, "vidalama geçişi" if s["speed"] else "seyir",
                         name_of(wps, s["q"])))
            t += dt
            q_prev = s["q"]
        else:
            key, val = s["key"], s["value"]
            if key == "wait_green_button":
                dt, kind, lab = 3.0, "operatör", "yeşil buton"
            elif key == "wait":
                dt, kind, lab = float(val), "bekleme", f"{val} s"
            elif key == "screwdriver":
                dt, kind, lab = 0.2, "vidalama", "DOUT0 " + ("ON" if val else "OFF")
            elif key == "gripper_position":
                dt, kind, lab = 1.7, "gripper", f"{val:.3f} m"
            else:
                dt, kind, lab = 0.5, "planning scene", key.replace("_screwdriver", "")
            rows.append((t, dt, kind, lab))
            t += dt
        t += WAIT_BETWEEN

    colors = {"seyir": ACCENT, "vidalama geçişi": WARM, "operatör": GREEN,
              "bekleme": "#adb5bd", "vidalama": "#f08c00", "gripper": "#7048e8",
              "planning scene": "#0c8599"}
    lanes = list(colors)
    fig, ax = plt.subplots(figsize=(8.0, 3.9), dpi=200)
    for t0, dt, kind, lab in rows:
        y = lanes.index(kind)
        ax.add_patch(FancyBboxPatch((t0, y - 0.32), max(dt, 0.25), 0.64,
                                    boxstyle="round,pad=0.0,rounding_size=0.08",
                                    facecolor=colors[kind], edgecolor="none", alpha=0.9))
    n_do = 0
    for t0, dt, kind, lab in rows:
        if kind == "operatör":
            ax.annotate("yeşil buton", (t0 + dt / 2, lanes.index(kind) + 0.42),
                        ha="center", fontsize=6.5, color=colors[kind])
        elif lab.startswith("DOUT"):
            # ON üstte, OFF altta: yan yana gelince etiketler üst üste biniyordu
            on = lab.endswith("ON")
            ax.annotate("ON" if on else "OFF",
                        (t0 + dt / 2, lanes.index(kind) + (0.42 if on else -0.78)),
                        ha="center", fontsize=6.5, color=colors[kind])
            n_do += 1
    ax.set_yticks(range(len(lanes)))
    ax.set_yticklabels(lanes, fontsize=8)
    ax.set_ylim(-0.7, len(lanes) - 0.3)
    ax.set_xlim(0, t * 1.01)
    ax.set_xlabel("turun başından itibaren tahmini süre [s]", fontsize=9)
    ax.grid(axis="x", alpha=0.25, lw=0.5)
    for sp in ("top", "right", "left"):
        ax.spines[sp].set_visible(False)
    ax.set_title(f"Tek turun zaman çizgisi — tahmini toplam {t:.0f} s "
                 "(operatör beklemeleri 3 s varsayıldı)\n"
                 "hareket süreleri en yavaş eklemin yolu / (limit x hız ölçeği) ile kestirildi",
                 fontsize=10, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_hrc_timeline.png"), bbox_inches="tight")
    plt.close(fig)
    return t


# --------------------------------------------------------------------------- #
# FİGÜR 6 — vida yaklaşımı: xTop -> xOpt
# --------------------------------------------------------------------------- #
def fig_screw_approach(cell, wps, seq):
    pairs = [("1. vida", "firstTop", "firstOpt"), ("2. vida", "secondTop", "secondOpt"),
             ("3. vida", "thirdTop", "thirdOpt"), ("4. vida", "fourthTop", "fourthOpt")]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.8, 3.6), dpi=200)
    names, dz, dxyz = [], [], []
    for label, top, opt in pairs:
        a, b = tcp_of(cell, wps[top]), tcp_of(cell, wps[opt])
        names.append(label)
        dz.append((b[2] - a[2]) * 1000.0)
        dxyz.append(np.linalg.norm(b - a) * 1000.0)
    x = np.arange(len(names))
    ax1.bar(x - 0.19, dz, 0.38, color=ACCENT, label="Δz (dikey)")
    ax1.bar(x + 0.19, dxyz, 0.38, color=WARM, label="|Δp| (toplam)")
    ax1.axhline(0, color=INK, lw=0.8)
    ax1.set_xticks(x)
    ax1.set_xticklabels(names, fontsize=8)
    ax1.set_ylabel("vidalama ucu yer değiştirmesi [mm]", fontsize=9)
    ax1.legend(fontsize=7.5, frameon=False)
    ax1.grid(axis="y", alpha=0.25, lw=0.5)
    ax1.set_title("xTop → xOpt dalış derinliği", fontsize=9, color=INK)

    cmap = plt.get_cmap("tab10")
    width = 0.8 / 6
    for k in range(1, 7):
        vals = [math.degrees(wps[opt][k] - wps[top][k]) for _, top, opt in pairs]
        ax2.bar(x + (k - 3.5) * width, vals, width, color=cmap((k - 1) % 10),
                label=JOINT_SHORT[k])
    ax2.axhline(0, color=INK, lw=0.8)
    ax2.set_xticks(x)
    ax2.set_xticklabels(names, fontsize=8)
    ax2.set_ylabel("eklem değişimi [°]", fontsize=9)
    ax2.legend(ncol=3, fontsize=7, frameon=False)
    ax2.grid(axis="y", alpha=0.25, lw=0.5)
    ax2.set_title("aynı geçişin eklem uzayındaki karşılığı", fontsize=9, color=INK)
    fig.suptitle("Vidalama geçişi: her vidada alet ne kadar ilerliyor", fontsize=10, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_hrc_screw_approach.png"), bbox_inches="tight")
    plt.close(fig)
    return names, dz, dxyz


# --------------------------------------------------------------------------- #
# FİGÜR 7 — GPIO: pin haritası ve el sıkışma (handshake)
# --------------------------------------------------------------------------- #
def fig_gpio(cell, wps, seq):
    fig = plt.figure(figsize=(7.9, 4.6), dpi=200)
    gs = fig.add_gridspec(2, 1, height_ratios=[1.0, 1.15], hspace=0.45)

    # --- üst: pin haritası ---
    ax = fig.add_subplot(gs[0])
    ax.axis("off")
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 3.1)
    boxes = [
        (0.2, "UR10e kontrol kutusu\nio_and_status_controller", "#e7f5ff", ACCENT),
        (3.9, "ROS 2\nset_io servisi / io_states konusu", "#fff4e6", WARM),
        (7.4, "senaryo düğümü\nhuman_robot_collaboration", "#ebfbee", GREEN),
    ]
    for x0, label, fc, ec in boxes:
        ax.add_patch(FancyBboxPatch((x0, 1.15), 2.4, 0.95,
                                    boxstyle="round,pad=0.06,rounding_size=0.12",
                                    facecolor=fc, edgecolor=ec, lw=1.2))
        ax.text(x0 + 1.2, 1.62, label, ha="center", va="center", fontsize=7.5, color=INK)
    for x0, x1 in ((2.6, 3.9), (6.3, 7.4)):
        ax.add_patch(FancyArrowPatch((x0, 1.62), (x1, 1.62), arrowstyle="<->",
                                     mutation_scale=11, color=GREY, lw=1.0))
    rows = [
        (0.2, 0.80, "ÇIKIŞ DOUT0", "vidalama SIKMA — senaryoda kullanılıyor", WARM),
        (0.2, 0.38, "ÇIKIŞ DOUT1", "vidalama SÖKME — rezerve", GREY),
        (5.3, 0.80, "GİRİŞ DIN7", "YEŞİL buton — senaryoda kullanılıyor", GREEN),
        (5.3, 0.38, "GİRİŞ DIN6/DIN5", "KIRMIZI / BEYAZ buton — rezerve", GREY),
    ]
    for x0, yy, pin, note, color in rows:
        ax.text(x0, yy, pin, fontsize=7.2, color=color, weight="bold", family="monospace")
        ax.text(x0 + 1.55, yy, note, fontsize=7.2, color=INK)
    ax.set_title("GPIO haritası (find_io_pins.py ile IFARLAB hücresinde ölçüldü)",
                 fontsize=9.5, color=INK)

    # --- alt: bir vidanın zamanlama diyagramı ---
    ax2 = fig.add_subplot(gs[1])
    T = 12.0
    t_press, t_on, t_down0, t_down1, t_wait, t_off, t_up = 2.0, 2.4, 2.6, 6.0, 7.5, 7.8, 11.2
    ax2.set_xlim(0, T)
    ax2.set_ylim(-0.4, 3.6)
    ax2.set_yticks([0, 1.2, 2.4])
    ax2.set_yticklabels(["ucun yüksekliği\n(xTop → xOpt)", "DOUT0\nvidalama motoru",
                         "DIN7\nyeşil buton"], fontsize=7.5)
    ax2.set_xlabel("zaman [s]  (operatörün butona basmasından itibaren)", fontsize=8.5)
    ax2.grid(axis="x", alpha=0.25, lw=0.5)
    for sp in ("top", "right", "left"):
        ax2.spines[sp].set_visible(False)

    def sig(y, xs, ys, color):
        ax2.step(xs, [y + 0.62 * v for v in ys], where="post", color=color, lw=1.6)

    ax2.set_ylim(-0.5, 3.9)
    sig(2.4, [0, t_press, t_press + 0.35, T], [0, 1, 0, 0], GREEN)
    sig(1.2, [0, t_on, t_off, T], [0, 1, 0, 0], WARM)
    ax2.plot([0, t_down0, t_down1, t_wait, t_up, T],
             [0.62, 0.62, 0.0, 0.0, 0.62, 0.62], color=ACCENT, lw=1.6)
    notes = [
        (t_press, 3.35, "yükselen kenar\n(önce bırakılması beklenir)", GREEN),
        (t_on, 1.92, "set_io(DOUT0, ON)", WARM),
        ((t_down0 + t_down1) / 2, 0.72, "hız ölçeği 0.01\nile dalış", ACCENT),
        (t_wait - 0.1, 0.12, "1.5 s oturma", INK),
        (t_off, 1.92, "set_io(DOUT0, OFF)\n(geri çıkmadan ÖNCE)", WARM),
    ]
    for x, y, txt, color in notes:
        ax2.annotate(txt, (x, y), fontsize=6.8, color=color, ha="center",
                     va="bottom" if y > 1 else "top")
    ax2.set_title("Bir vidanın el sıkışma sırası: operatör onayı → motor → yavaş dalış → motor kapalı → geri çekilme",
                  fontsize=9.5, color=INK)
    fig.savefig(os.path.join(HERE, "fig_hrc_gpio.png"), bbox_inches="tight")
    plt.close(fig)


# --------------------------------------------------------------------------- #
# FİGÜR 8 — attach edilen vidalama aleti (planning scene)
# --------------------------------------------------------------------------- #
def fig_attach(cell, wps, seq):
    q = wps["holdScrewer"]
    poses = cell.fk(joint_values(q, gripper=0.026))
    parts = []
    for link in GRIPPER_LINKS + ("ur10e_wrist_3_link", "ur10e_tool0", "ur10e_flange"):
        if link not in poses:
            continue
        for T_local, kind, spec in cell.visuals(link):
            if kind != "mesh":
                continue
            got = mesh_tris(spec[0], spec[1], 6000)
            if got is None:
                continue
            V, F = got
            T = poses[link] @ T_local
            parts.append(((T[:3, :3] @ V.T).T + T[:3, 3], F,
                          "#7a8290" if "gripper" in link else "#c8ccd4", 1.0))

    # attach edilen silindir: add_collision_cylinder(height=0.02, radius=0.002,
    # position=SCREWDRIVER_IN_GRIPPER, quat_xyzw=..., frame_id=gripper_base_link)
    quat = np.array([-0.679113, -0.196990, 0.196990, 0.679113])
    x, y, z, w = quat
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    Tc = np.eye(4)
    Tc[:3, :3] = R
    Tc[:3, 3] = SCREWDRIVER_IN_GRIPPER
    Tw = poses[TOOL_LINK] @ Tc
    V, F = cyl_tris(0.002, 0.02, n=24)
    parts.append(((Tw[:3, :3] @ V.T).T + Tw[:3, 3], F, WARM, 1.0))

    fig, axes = plt.subplots(1, 2, figsize=(7.8, 3.8), dpi=200)
    for ax, (elev, azim, t) in zip(axes, [(12, 200, "yandan"), (60, 250, "eğik")]):
        basis = camera(elev, azim)
        pts = draw_tris(ax, parts, basis)
        cxy, _ = project(np.array([Tw[:3, 3]]), basis)
        ax.annotate("attach edilen silindir\nh=20 mm, r=2 mm, ağırlık 2.4 kg",
                    cxy[0], textcoords="offset points", xytext=(16, -26), fontsize=7,
                    color=WARM, arrowprops=dict(arrowstyle="->", color=WARM, lw=0.9))
        fit(ax, pts)
        ax.set_title(t, fontsize=9, color=INK)
    fig.suptitle("Vidalama aletinin planning scene temsili: gripper kapandıktan sonra\n"
                 "ur10e_gripper_base_link'e attach edilen silindir (touch_links ile 6 komşu link muaf)",
                 fontsize=9.5, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_hrc_attach.png"), bbox_inches="tight")
    plt.close(fig)


# --------------------------------------------------------------------------- #
# FİGÜR 9 — segment maliyetleri
# --------------------------------------------------------------------------- #
def fig_segment_cost(cell, wps, seq):
    moves = [s for s in seq_steps(seq) if s["kind"] == "move"]
    names, cost, dur, slow = [], [], [], []
    for a, b in zip(moves[:-1], moves[1:]):
        scale = b["speed"] or 0.1
        dq = np.abs(np.asarray(b["q"]) - np.asarray(a["q"]))
        names.append(f"{name_of(wps, a['q'])} → {name_of(wps, b['q'])}")
        cost.append(float(np.degrees(dq[1:]).sum()))
        dur.append(seg_duration(a["q"], b["q"], scale))
        slow.append(b["speed"] is not None)
    order = np.arange(len(names))
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(8.2, 5.4), dpi=200, sharey=True)
    colors = [WARM if s else ACCENT for s in slow]
    ax.barh(order, cost, color=colors)
    ax.set_xlabel("eklem uzayında yol [° toplam]", fontsize=9)
    ax.invert_yaxis()
    ax.set_yticks(order)
    ax.set_yticklabels(names, fontsize=5.8)
    ax.grid(axis="x", alpha=0.25, lw=0.5)
    ax2.barh(order, dur, color=colors)
    ax2.set_xlabel("tahmini süre [s]", fontsize=9)
    ax2.grid(axis="x", alpha=0.25, lw=0.5)
    fig.suptitle("Her geçişin eklem-uzayı yolu ve tahmini süresi\n"
                 "kırmızı: hız ölçeği 0.01 (vidalama) · mavi: 0.1 (seyir) — "
                 "aynı yol 10 kat uzun sürüyor", fontsize=10, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_hrc_segment_cost.png"), bbox_inches="tight")
    plt.close(fig)
    return names, cost, dur, slow



# --------------------------------------------------------------------------- #
# FİGÜR 10 — çarpışma payları (ölçülen)
#   Kolun çarpışma mesh'leri ile hücredeki DURAĞAN gövdeler arasındaki en küçük
#   mesafe. Payı değil, gerçek geometriyi ölçer: open3d'nin RaycastingScene'i
#   üçgen ağına işaretli olmayan tam mesafeyi verir.
# --------------------------------------------------------------------------- #
CLEAR_ARM = ("ur10e_gripper_base_link", "ur10e_gripper_upper_1", "ur10e_gripper_lower_1",
             "ur10e_wrist_3_link", "ur10e_wrist_2_link", "ur10e_forearm_link")


def _world_VF(cell, link, poses, tag="collision", budget=30000):
    Vs, Fs, off = [], [], 0
    for v in cell.links[link].findall(tag):
        m = v.find("geometry/mesh")
        if m is None:
            continue
        scale = [float(x) for x in m.get("scale", "1 1 1").split()]
        got = mesh_tris(m.get("filename"), scale, budget)
        if got is None:
            continue
        V, F = got
        xyz, rpy = _origin(v)
        T = poses[link] @ xf(xyz, rpy)
        Vs.append((T[:3, :3] @ V.T).T + T[:3, 3])
        Fs.append(np.asarray(F) + off)
        off += len(V)
    if not Vs:
        return None
    return np.vstack(Vs), np.vstack(Fs)


def clearance(cell, wps, qname, obstacle, grip=0.026):
    """(mesafe [m], en yakın kol linki). Payların HİÇBİRİ uygulanmaz."""
    import open3d as o3d
    poses = cell.fk(joint_values(wps[qname], grip))
    ob = _world_VF(cell, obstacle, poses)
    if ob is None:
        return None, None
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh(
        o3d.core.Tensor(ob[0], o3d.core.float32), o3d.core.Tensor(ob[1], o3d.core.int32)))
    best = (1e9, None)
    for L in CLEAR_ARM:
        vf = _world_VF(cell, L, poses)
        if vf is None:
            continue
        d = float(scene.compute_distance(
            o3d.core.Tensor(vf[0].astype(np.float32))).numpy().min())
        if d < best[0]:
            best = (d, L)
    return best


CLEARANCE_CASES = [
    ("holdScrewer", "ur10e_alumunium_table", "aleti kavrarken\ntezgâh"),
    ("tookScrew", "ur10e_screw_feeder", "vida alırken\nbesleyici"),
    ("outTookScrew", "ur10e_screw_feeder", "besleyiciden\nçıkarken"),
    ("thirdOpt", "ur10e_alumunium_table", "3. vida\ntezgâh"),
    ("firstOpt", "ur10e_alumunium_table", "1. vida\ntezgâh"),
    ("secondOpt", "ur10e_alumunium_table", "2. vida\ntezgâh"),
    ("fourthOpt", "ur10e_alumunium_table", "4. vida\ntezgâh"),
    ("holdScrewer", "human_link", "aleti kavrarken\noperatör mankeni"),
]


def measure_clearances(cell, wps):
    out = []
    for q, ob, label in CLEARANCE_CASES:
        d, link = clearance(cell, wps, q, ob)
        out.append((label.replace("\\n", " "), q, ob, d * 1000.0, link))
    return out


def fig_clearance(cell, wps, seq):
    rows = []
    for q, ob, label in CLEARANCE_CASES:
        d, link = clearance(cell, wps, q, ob)
        rows.append((label, d * 1000.0, link, ob))
    rows.sort(key=lambda r: r[1])
    fig, ax = plt.subplots(figsize=(7.8, 3.9), dpi=200)
    y = np.arange(len(rows))
    colors = [WARM if r[1] < 10 else ("#f59f00" if r[1] < 50 else ACCENT) for r in rows]
    ax.barh(y, [r[1] for r in rows], color=colors)
    ax.set_yticks(y)
    ax.set_yticklabels([r[0] for r in rows], fontsize=7.5)
    ax.invert_yaxis()
    ax.set_xscale("log")
    ax.set_xlabel("en küçük mesafe [mm, log ölçek]", fontsize=9)
    ax.grid(axis="x", alpha=0.25, lw=0.5)
    for yy, r in zip(y, rows):
        ax.annotate(f"{r[1]:.1f} mm — {r[2].replace('ur10e_', '')}", (r[1], yy),
                    textcoords="offset points", xytext=(6, -3), fontsize=7, color=INK)
    ax.set_xlim(1, 3000)
    ax.set_title("Ölçülen çarpışma payları: kol çarpışma mesh'i ↔ duran gövde\n"
                 "(URDF geometrisi, padding UYGULANMADAN; operatör mankeni SRDF'te "
                 "zaten devre dışı)", fontsize=9.5, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_hrc_clearance.png"), bbox_inches="tight")
    plt.close(fig)
    return rows


FIGURES = {
    "cell": fig_cell,
    "postures": fig_postures,
    "tcp": fig_tcp_path,
    "joints": fig_joint_profile,
    "timeline": fig_timeline,
    "screw": fig_screw_approach,
    "gpio": fig_gpio,
    "attach": fig_attach,
    "segments": fig_segment_cost,
    "clearance": fig_clearance,
}


def main(argv):
    wanted = [a for a in argv if a in FIGURES] or list(FIGURES)
    cell = Cell(urdf_path())
    _, wps, seq = scenario_data()
    for key in wanted:
        print(f"  {key} ...", flush=True)
        FIGURES[key](cell, wps, seq)
    print("bitti ->", HERE)


if __name__ == "__main__":
    main(sys.argv[1:])
