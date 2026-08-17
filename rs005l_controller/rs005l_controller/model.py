#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math

###############################################################################
# Geometry (kept from your file)
###############################################################################

NUM_JOINTS = 6

# DH parameters: (theta_offset_deg, d, a, alpha_deg)
DH_PARAMS = [
    (0,    0.285, 0,   90),
    (0,    0.105, 0,  -90),
    (0,    0.380, 0,   90),
    (0,    0.080, 0,  -90),
    (0,    0.143, 0,   90),
    (0,    0.267, 0,  -90)
]

def dh_transform(theta_deg, d, a, alpha_deg):
    """Create a 4x4 DH transformation (angles in degrees)."""
    theta = math.radians(theta_deg)
    alpha = math.radians(alpha_deg)
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ ct,    -st*ca,  st*sa,  a*ct ],
        [ st,     ct*ca, -ct*sa,  a*st ],
        [  0,        sa,     ca,     d ],
        [  0,         0,      0,     1 ]
    ])

def forward_kinematics(joint_angles_deg):
    """Return end-effector [x,y,z]. joint_angles_deg is a 6-element array in degrees."""
    T = np.eye(4)
    for i, (theta0, d, a, alpha) in enumerate(DH_PARAMS):
        total_theta = theta0 + float(joint_angles_deg[i])
        T_i = dh_transform(total_theta, d, a, alpha)
        T = T @ T_i
    return T[:3, 3]

###############################################################################
# Analytic kinematics (frames, Jacobians)
###############################################################################

def _fk_all(joint_angles_deg):
    """
    Returns lists:
      T0i:   list of 4x4 transforms ^0T_i (i=0..6), with T0i[0]=I
      oi:    list of origins o_i in R^3
      zi:    list of z axes z_i (in base frame) for each joint axis (i=0..5)
    """
    T = np.eye(4)
    T0i = [T.copy()]
    oi  = [T[:3, 3].copy()]
    zi  = []

    for j, (theta0, d, a, alpha) in enumerate(DH_PARAMS):
        total_theta = theta0 + float(joint_angles_deg[j])
        T = T @ dh_transform(total_theta, d, a, alpha)
        T0i.append(T.copy())
        oi.append(T[:3, 3].copy())
        # z_{j} is the previous joint axis (current transform's z after applying up to joint j)
        # For revolute joints, axis is z of frame j in base:
        z_prev = T0i[-2][:3, 2]
        zi.append(z_prev.copy())

    return T0i, oi, zi  # len(T0i)=7 (0..6), len(oi)=7, len(zi)=6

def jacobian_analytic_ee(joint_angles_deg):
    """
    6x6 spatial Jacobian for end-effector (stacked [Jv; Jw]).
    """
    T0i, oi, zi = _fk_all(joint_angles_deg)
    on = oi[-1]
    Jv = np.zeros((3, NUM_JOINTS))
    Jw = np.zeros((3, NUM_JOINTS))
    for k in range(NUM_JOINTS):
        z = zi[k]
        ok = oi[k]
        Jv[:, k] = np.cross(z, (on - ok))
        Jw[:, k] = z
    return np.vstack((Jv, Jw))

###############################################################################
# Approximate inertial parameters (table)
###############################################################################
# Kept from your code and extended to a table structure.
# If you have CAD/URDF inertias, replace these with true COM and I matrices.
link_masses  = np.array([10.395, 8.788, 7.575, 2.679, 1.028, 1.000])
link_lengths = [0.3, 0.3, 0.3, 0.2, 0.1, 0.1]
I_diag       = [0.07, 0.05, 0.05, 0.02, 0.01, 0.01]
GRAVITY      = 9.81

# Parameter table (COM is along local z by half length as a pragmatic default)
LINK_PARAMS = []
for i in range(NUM_JOINTS):
    m  = float(link_masses[i])
    L  = float(link_lengths[i])
    I3 = np.diag([I_diag[i], I_diag[i], I_diag[i]])  # placeholder; replace with CAD values if available
    LINK_PARAMS.append({
        "mass": m,
        "com": np.array([0.0, 0.0, L*0.5]),  # local COM
        "I": I3                               # about COM in local link frame
    })

###############################################################################
# COM poses and COM-Jacobians
###############################################################################

def _com_transform(joint_angles_deg, i_link):
    """
    ^0T_com(i): base->COM_i transform (use link i frame and a pure translation to its COM)
    """
    T0i, _, _ = _fk_all(joint_angles_deg)
    Ti = T0i[i_link+1]            # ^0T_i (after applying joint i)
    ci = LINK_PARAMS[i_link]["com"]
    Tci = Ti @ np.array([[1,0,0,ci[0]],
                         [0,1,0,ci[1]],
                         [0,0,1,ci[2]],
                         [0,0,0,1]])
    return Tci

def _com_jacobians(joint_angles_deg, i_link):
    """
    Returns (Jv_i, Jw_i) for COM of link i in base frame.
    """
    T0i, oi, zi = _fk_all(joint_angles_deg)
    Tci = _com_transform(joint_angles_deg, i_link)
    oci = Tci[:3, 3]
    Jv = np.zeros((3, NUM_JOINTS))
    Jw = np.zeros((3, NUM_JOINTS))
    for k in range(NUM_JOINTS):
        z = zi[k]
        ok = oi[k]
        Jv[:, k] = np.cross(z, (oci - ok))
        Jw[:, k] = z
    return Jv, Jw, Tci[:3, :3]  # rotation of COM

###############################################################################
# Dynamics: M(q), g(q), C(q,qdot), friction, potential
###############################################################################

def mass_matrix(q_deg):
    """
    Joint-space inertia matrix using composite rigid-body style sum over link COM Jacobians.
    """
    M = np.zeros((NUM_JOINTS, NUM_JOINTS))
    for i in range(NUM_JOINTS):
        mi = LINK_PARAMS[i]["mass"]
        Ii = LINK_PARAMS[i]["I"]
        Jv, Jw, R0i = _com_jacobians(q_deg, i)
        M += Jv.T @ (mi * np.eye(3)) @ Jv + Jw.T @ (R0i @ Ii @ R0i.T) @ Jw
    return M

def gravity_vector(q_deg, gvec=np.array([0.0, 0.0, -GRAVITY])):
    """
    g(q) = Σ Jv_i(q)^T m_i g
    """
    g = np.zeros(NUM_JOINTS)
    for i in range(NUM_JOINTS):
        mi = LINK_PARAMS[i]["mass"]
        Jv, _, _ = _com_jacobians(q_deg, i)
        g += Jv.T @ (mi * gvec)
    return g

def potential_U(q_deg, gvec=np.array([0.0, 0.0, -GRAVITY])):
    """
    U(q) = Σ m_i * g^T * r_com_i(q)
    """
    U = 0.0
    for i in range(NUM_JOINTS):
        mi = LINK_PARAMS[i]["mass"]
        Tci = _com_transform(q_deg, i)
        r = Tci[:3, 3]
        U += mi * gvec.dot(r)
    return float(-U)  # classical choice: potential is -m g^T r (so that g = ∂U/∂q)

def friction_torque(qdot_deg, kv=None, kc=None, eps=1e-3):
    """
    Simple viscous + Coulomb friction per joint:
      tau_f = Kv * qdot + Kc * tanh(qdot/eps)
    """
    if kv is None: kv = np.array([0.2,0.2,0.15,0.08,0.05,0.05])
    if kc is None: kc = np.array([0.4,0.3,0.3,0.15,0.10,0.10])
    qdot = np.deg2rad(np.asarray(qdot_deg))
    return kv * qdot + kc * np.tanh(qdot/eps)

def coriolis_matrix(q_deg, qdot_deg, h=1e-6):
    """
    Numerical Christoffel construction:
      C_{ij} = 0.5 * Σ_k ( ∂M_{ij}/∂q_k + ∂M_{ik}/∂q_j - ∂M_{jk}/∂q_i ) * qdot_k
    We return the full C(q, qdot) matrix (not just b=C*qdot), so validation can use ||Mdot-2C||_F.
    """
    q = np.asarray(q_deg, dtype=float)
    qdot = np.asarray(qdot_deg, dtype=float)
    M0 = mass_matrix(q)
    # partial derivatives of M wrt each q_k by central differences
    dM = []
    for k in range(NUM_JOINTS):
        dq = np.zeros(NUM_JOINTS); dq[k] = h * (1.0)   # degrees step
        Mp = mass_matrix(q + dq)
        Mm = mass_matrix(q - dq)
        dM.append((Mp - Mm) / (2*h))
    # assemble C
    C = np.zeros_like(M0)
    for i in range(NUM_JOINTS):
        for j in range(NUM_JOINTS):
            s = 0.0
            for k in range(NUM_JOINTS):
                s += (dM[k][i, j] + dM[j][i, k] - dM[i][j, k]) * qdot[k]
            C[i, j] = 0.5 * s
    return C

def b_term(q_deg, qdot_deg):
    """Convenience: b(q,qdot) = C(q,qdot) @ qdot"""
    C = coriolis_matrix(q_deg, qdot_deg)
    return C @ np.asarray(qdot_deg, dtype=float)

def compute_jacobian(joint_angles_deg, delta=1e-5):
    """Legacy 3x6 numeric Jacobian of EE position (kept for compatibility)."""
    base_pos = forward_kinematics(joint_angles_deg)
    J = np.zeros((3, NUM_JOINTS))
    for j in range(NUM_JOINTS):
        perturbed = np.array(joint_angles_deg, dtype=float)
        perturbed[j] += delta
        pos_pert = forward_kinematics(perturbed)
        J[:, j] = (pos_pert - base_pos) / delta
    return J

def inverse_kinematics_xyz(target_pos, init_q_deg, max_iter=100, alpha=0.01, tol=1e-4):
    """Simple iterative IK (unchanged)."""
    q = np.array(init_q_deg, dtype=float)
    for _ in range(max_iter):
        current_pos = forward_kinematics(q)
        error = target_pos - current_pos
        if np.linalg.norm(error) < tol:
            break
        J = compute_jacobian(q)
        J_inv = np.linalg.pinv(J, rcond=1e-3)
        q += alpha * (J_inv @ error)
    return q
