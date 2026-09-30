#!/usr/bin/env python3
"""HARMONY validasyon defect'leri icin eklem acisi ureteci.

cleaning_mission_runner.py, defect'lere sabit (hardcode) eklem acilariyla gider
(sahada tekrarlanabilirlik icin). Bu script o acilari uretir: her defect icin
kamera (ur10e_depth_optical_frame) defect + (dx, 0, dz) noktasina konumlanir ve
optik ekseni kapiya (-X) bakacak sekilde yonlendirilir; ardindan sayisal IK ile
7 eklemlik (lineer eksen + UR10e) cozum bulunur.

Kullanim:
    xacro src/Universal_Robots_ROS2_Tutorials/my_robot_cell/my_robot_cell_control/\\
        urdf/whole_cell_hw.urdf.xacro control_group:=ur ur_type:=ur10e \\
        use_fake_hardware:=true > /tmp/whole_cell_hw.urdf
    python3 generate_defect_configs.py /tmp/whole_cell_hw.urdf

Cikti dogrudan cleaning_mission_runner.py icindeki DEFECT_JOINTS tablosuna
kopyalanabilir. Uretilen acilar SAHADA DOGRULANMALIDIR: bu script yalnizca
kapi meshlerine olan mesafeyi kontrol eder, hucrenin geri kalani (sasi, masa,
kablo kanali, bariyerler) icin MoveIt carpisma kontrolu gerekir.
"""

from __future__ import annotations

import math
import struct
import sys
from typing import List, Optional, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
from urdf_parser_py.urdf import URDF

# --------------------------------------------------------------------------
# Yapilandirma
# --------------------------------------------------------------------------
CAM_LINK = "ur10e_depth_optical_frame"
TOOL_LINK = "ur10e_tool0"

DX = 0.15   # kapidan lineer eksene dogru pay (+X)
DZ = 0.15   # yukari dogru pay (+Z)
TILT_DEG = 0.0  # 0 = optik eksen yatay (-X). >0 ise kamera asagi egilir.

# HARMONY validasyon senaryosunun 4 yapay defect'i (world frame, metre)
DEFECTS = [
    ("DEF-01", "Protrusion", (-1.100, 0.800, 1.520)),
    ("DEF-02", "Protrusion", (-1.100, 0.300, 1.530)),
    ("DEF-03", "Dent",       (-0.918, 0.274, 1.040)),
    ("DEF-04", "Dent",       (-0.892, 1.680, 1.000)),
]

JOINTS = [
    "ur10e_base_to_robot_mount",
    "ur10e_shoulder_pan_joint",
    "ur10e_shoulder_lift_joint",
    "ur10e_elbow_joint",
    "ur10e_wrist_1_joint",
    "ur10e_wrist_2_joint",
    "ur10e_wrist_3_joint",
]

# Dogrulanmis sensing waypoint'leri IK tohumu olarak kullanilir.
_r = math.radians
SEEDS = [
    [1.0, _r(86.40), _r(-44.74), _r(98.25), _r(-233.91), _r(-90), _r(90)],
    [1.0, _r(86.37), _r(-66.12), _r(45.49), _r(-164.0), _r(-90), _r(90)],
    [1.0, _r(94.91), _r(-111.64), _r(-37.60), _r(-28.84), _r(-90), _r(90)],
    [1.0, _r(94.84), _r(-105.96), _r(-90.55), _r(-18.31), _r(-90), _r(90)],
    [1.0, _r(109.13), _r(-57.99), _r(-83.46), _r(-40.12), _r(-90), _r(90)],
    [1.0, _r(108.91), _r(-46.56), _r(-115.55), _r(-19.46), _r(-90), _r(90)],
    [1.0, _r(54), _r(-92), _r(-156), _r(65), _r(-36), _r(93)],
    [1.0, 1.554, -2.377, -1.837, 1.072, -1.486, _r(90)],
]

CLEARANCE_LINKS = [
    "ur10e_upper_arm_link", "ur10e_forearm_link", "ur10e_wrist_1_link",
    "ur10e_wrist_2_link", "ur10e_wrist_3_link", TOOL_LINK, CAM_LINK,
]

DOOR_MESHES = [
    "/home/cem/colcon_ws/src/Universal_Robots_ROS2_Description/meshes/ur10e/collision/windowlessdoor.stl",
    "/home/cem/colcon_ws/src/Universal_Robots_ROS2_Description/meshes/ur10e/collision/windowdoor.stl",
]


# --------------------------------------------------------------------------
# Kinematik
# --------------------------------------------------------------------------
def rpy_to_mat(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def axis_angle_to_mat(axis, theta: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def homogeneous(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = p
    return M


class Model:
    """URDF uzerinden ileri kinematik ve eklem limitleri."""

    def __init__(self, path: str):
        self.robot = URDF.from_xml_file(path)
        self.jmap = {j.name: j for j in self.robot.joints}
        self.parent_of = {j.child: j for j in self.robot.joints}

    def chain(self, target_link: str, root: str = "world"):
        out = []
        link = target_link
        while link != root:
            j = self.parent_of.get(link)
            if j is None:
                raise RuntimeError(f"{link} icin parent joint bulunamadi")
            out.append(j)
            link = j.parent
        return list(reversed(out))

    def fk(self, target_link: str, q, root: str = "world") -> np.ndarray:
        qd = dict(zip(JOINTS, q))
        M = np.eye(4)
        for j in self.chain(target_link, root):
            o = j.origin
            xyz = np.array(o.xyz if o and o.xyz else [0, 0, 0], dtype=float)
            rpy = np.array(o.rpy if o and o.rpy else [0, 0, 0], dtype=float)
            M = M @ homogeneous(rpy_to_mat(*rpy), xyz)
            if j.type in ("revolute", "continuous"):
                M = M @ homogeneous(axis_angle_to_mat(j.axis, qd.get(j.name, 0.0)), np.zeros(3))
            elif j.type == "prismatic":
                a = np.asarray(j.axis, dtype=float)
                a = a / np.linalg.norm(a)
                M = M @ homogeneous(np.eye(3), a * qd.get(j.name, 0.0))
        return M

    def limits(self) -> Tuple[np.ndarray, np.ndarray]:
        lo, hi = [], []
        for n in JOINTS:
            j = self.jmap[n]
            if j.limit is not None and j.type != "continuous":
                lo.append(j.limit.lower)
                hi.append(j.limit.upper)
            else:
                lo.append(-2 * math.pi)
                hi.append(2 * math.pi)
        return np.array(lo), np.array(hi)


def load_stl_vertices(path: str) -> np.ndarray:
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        a = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
        return a[:, 12:48].copy().view(np.float32).reshape(-1, 3)


def look_at_rotation(forward) -> np.ndarray:
    """Optik frame donusu: +Z = bakis yonu, +Y = goruntu asagi, +X = goruntu sag."""
    f = np.asarray(forward, dtype=float)
    f = f / np.linalg.norm(f)
    right = np.cross(f, np.array([0.0, 0.0, 1.0]))
    right = right / np.linalg.norm(right)
    down = np.cross(f, right)
    return np.column_stack([right, down, f])


def solve_ik(model: Model, link: str, p_target, R_target, seed, lo, hi, rail_hint):
    def residual(q):
        M = model.fk(link, q)
        e_pos = M[:3, 3] - p_target
        e_rot = Rot.from_matrix(R_target.T @ M[:3, :3]).as_rotvec()
        e_reg = 0.02 * (np.asarray(q) - np.asarray(seed))
        e_rail = np.array([0.05 * (q[0] - rail_hint)])
        return np.concatenate([e_pos, 0.5 * e_rot, e_reg, e_rail])

    res = least_squares(residual, seed, bounds=(lo, hi), xtol=1e-12, ftol=1e-12, max_nfev=4000)
    M = model.fk(link, res.x)
    err_pos = float(np.linalg.norm(M[:3, 3] - p_target))
    err_rot = float(np.linalg.norm(Rot.from_matrix(R_target.T @ M[:3, :3]).as_rotvec()))
    return res.x, err_pos, err_rot


def main(urdf_path: str) -> int:
    model = Model(urdf_path)
    lo, hi = model.limits()
    door = np.vstack([load_stl_vertices(p) for p in DOOR_MESHES])

    tilt = math.radians(TILT_DEG)
    R_target = look_at_rotation([-math.cos(tilt), 0.0, -math.sin(tilt)])
    quat = Rot.from_matrix(R_target).as_quat()  # x, y, z, w

    print(f"# offsets: dx={DX} dz={DZ} tilt={TILT_DEG} deg")
    print(f"# kamera optik +Z (bakis yonu) = {np.round(R_target[:, 2], 4).tolist()}")
    print(f"# CAMERA_QUAT_XYZW = {[round(float(v), 6) for v in quat]}")
    print()

    rows = []
    for name, dtype, defect in DEFECTS:
        p_defect = np.array(defect, dtype=float)
        p_target = p_defect + np.array([DX, 0.0, DZ])
        rail_hint = float(np.clip(defect[1], lo[0], hi[0]))

        best: Optional[tuple] = None
        for seed in SEEDS:
            s = np.clip(np.array(seed, dtype=float), lo, hi)
            s[0] = rail_hint
            q, ep, er = solve_ik(model, CAM_LINK, p_target, R_target, s, lo, hi, rail_hint)
            score = ep + 0.05 * er
            if best is None or score < best[0]:
                best = (score, q, ep, er)

        _, q, err_pos, err_rot = best
        clearance = min(
            (float(np.sqrt(((door - model.fk(l, q)[:3, 3]) ** 2).sum(1)).min()), l)
            for l in CLEARANCE_LINKS
        )
        cam = model.fk(CAM_LINK, q)
        rows.append((name, dtype, defect, q, err_pos, err_rot, clearance, cam))

    print("DEFECT_JOINTS = {")
    for name, dtype, defect, q, err_pos, err_rot, clearance, cam in rows:
        print(f'    # {name} ({dtype}) defect={defect}')
        print(f'    #   hedef kamera={tuple(round(float(v), 3) for v in np.array(defect) + [DX, 0, DZ])}'
              f'  ulasilan={tuple(round(float(v), 3) for v in cam[:3, 3])}')
        print(f'    #   pos_err={err_pos * 1000:.1f} mm, rot_err={math.degrees(err_rot):.2f} deg, '
              f'kapi mesafesi={clearance[0] * 100:.1f} cm ({clearance[1]})')
        print(f'    "{name}": [' + ", ".join(f"{v:.6f}" for v in q) + "],")
    print("}")
    print()

    bad = [r for r in rows if r[4] > 0.01 or r[6][0] < 0.10]
    if bad:
        print("# UYARI: asagidaki defect'ler hedefe tam ulasilamadi veya kapiya < 10 cm yaklasti:")
        for name, _, _, _, ep, _, cl, _ in bad:
            print(f"#   {name}: pos_err={ep * 1000:.1f} mm, kapi mesafesi={cl[0] * 100:.1f} cm")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/whole_cell_hw.urdf"))
