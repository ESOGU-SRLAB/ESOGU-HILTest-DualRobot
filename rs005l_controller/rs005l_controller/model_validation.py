#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from rs005l_controller.model import (
    NUM_JOINTS, GRAVITY,
    forward_kinematics,
    mass_matrix, gravity_vector, potential_U, coriolis_matrix
)

# --- Checks required by the assignment ---
# 1) SPD: lambda_min(M) > 1e-6
def test_spd(q_deg):
    M = mass_matrix(q_deg)
    lam = np.linalg.eigvalsh(M)
    return bool(np.min(lam) > 1e-6), lam

# 2) || Mdot - 2C ||_F < 1e-6
def test_skew_symmetry(q_deg, qdot_deg, dt=1e-3):
    M1 = mass_matrix(q_deg)
    q2 = np.asarray(q_deg) + np.asarray(qdot_deg) * dt
    M2 = mass_matrix(q2)
    Mdot = (M2 - M1) / dt
    C = coriolis_matrix(q_deg, qdot_deg)
    return np.linalg.norm(Mdot - 2.0 * C, ord='fro')

# 3) || g(q) - grad U(q) ||_2 < 1e-5
def test_gravity_gradient(q_deg, h=1e-5):
    g = gravity_vector(q_deg)
    gradU = np.zeros(NUM_JOINTS)
    U0 = potential_U(q_deg)
    for i in range(NUM_JOINTS):
        dq = np.zeros(NUM_JOINTS); dq[i] = h
        gradU[i] = (potential_U(np.asarray(q_deg)+dq) - U0) / h
    return np.linalg.norm(g - gradU)

if __name__ == "__main__":
    np.random.seed(0)
    q  = np.random.uniform(-45, 45, size=(NUM_JOINTS,))
    dq = np.random.uniform(-5, 5, size=(NUM_JOINTS,))

    spd_ok, eigs = test_spd(q)
    skew = test_skew_symmetry(q, dq)
    grav_err = test_gravity_gradient(q)

    print(f"SPD ok? {spd_ok} | lambda_min={np.min(eigs):.3e}")
    print(f"|| Mdot - 2C ||_F = {skew:.3e}  (target < 1e-6)")
    print(f"|| g - gradU ||_2 = {grav_err:.3e}  (target < 1e-5)")
