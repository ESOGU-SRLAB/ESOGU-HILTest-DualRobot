#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Model Validation for RS005L + AGV system
Author: Cem Süha Yılmaz (2025)
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from model import forward_kinematics, inverse_kinematics_xyz, mass_matrix, coriolis_matrix, gravity_vector

# ======================================================
# 1. Load Logged Data
# ======================================================
df = pd.read_csv("rs_log.csv")
joint_cols = ['q1','q2','q3','q4','q5','q6']
tcp_cols   = ['x','y','z']

q_rad = df[joint_cols].values
q_deg = np.degrees(q_rad)
xyz_real = df[tcp_cols].values

# ======================================================
# 2. Forward Kinematics Validation
# ======================================================
xyz_model = np.array([forward_kinematics(qi) for qi in q_deg])
error_fk = np.linalg.norm(xyz_model - xyz_real, axis=1)

print(f"\n=== Forward Kinematics Validation ===")
print(f"Mean position error: {np.mean(error_fk)*1000:.2f} mm")
print(f"Max position error : {np.max(error_fk)*1000:.2f} mm")

plt.figure()
plt.plot(error_fk*1000)
plt.title("Forward Kinematics Error (mm)")
plt.xlabel("Sample")
plt.ylabel("Error [mm]")
plt.savefig("fk_error.png", dpi=150)

# ======================================================
# 3. Inverse Kinematics Validation
# ======================================================
xyz_sample = xyz_real[::50]  # her 50. örnekten birini al
q_ik = np.array([inverse_kinematics_xyz(p, q_deg[0]) for p in xyz_sample])
xyz_reconstructed = np.array([forward_kinematics(q) for q in q_ik])
error_ik = np.linalg.norm(xyz_reconstructed - xyz_sample, axis=1)

print(f"\n=== Inverse Kinematics Validation ===")
print(f"Mean reconstruction error: {np.mean(error_ik)*1000:.2f} mm")
print(f"Max reconstruction error : {np.max(error_ik)*1000:.2f} mm")

plt.figure()
plt.plot(error_ik*1000, 'r')
plt.title("Inverse Kinematics Reconstruction Error (mm)")
plt.xlabel("Sample (every 50th)")
plt.ylabel("Error [mm]")
plt.savefig("ik_error.png", dpi=150)

# ======================================================
# 4. Dynamics Validation
# ======================================================
# Eğer simülasyonda torque (tau) verisi kaydedilmişse ekle
if 'tau1' in df.columns:
    tau_meas = df[['tau1','tau2','tau3','tau4','tau5','tau6']].values
else:
    tau_meas = np.zeros_like(q_rad)

dq_deg = np.gradient(q_deg, axis=0) / np.gradient(df['t_sec'].values)[:,None]
ddq_deg = np.gradient(dq_deg, axis=0) / np.gradient(df['t_sec'].values)[:,None]

tau_model = []
for q, dq, ddq in zip(q_deg, dq_deg, ddq_deg):
    M = mass_matrix(q)
    C = coriolis_matrix(q, dq)
    g = gravity_vector(q)
    tau = M @ ddq + C @ dq + g
    tau_model.append(tau)
tau_model = np.array(tau_model)

error_dyn = np.linalg.norm(tau_meas - tau_model, axis=1)

print(f"\n=== Dynamic Model Validation ===")
print(f"Mean torque error: {np.mean(error_dyn):.2f} Nm")
print(f"Max torque error : {np.max(error_dyn):.2f} Nm")

plt.figure()
plt.plot(error_dyn)
plt.title("Dynamic Model Torque Error [Nm]")
plt.xlabel("Sample")
plt.ylabel("||tau_meas - tau_model||")
plt.savefig("dyn_error.png", dpi=150)

# ======================================================
# 5. Summary Save
# ======================================================
summary = {
    "mean_fk_mm": np.mean(error_fk)*1000,
    "max_fk_mm": np.max(error_fk)*1000,
    "mean_ik_mm": np.mean(error_ik)*1000,
    "max_ik_mm": np.max(error_ik)*1000,
    "mean_tau_err": np.mean(error_dyn),
    "max_tau_err": np.max(error_dyn)
}
pd.DataFrame([summary]).to_csv("validation_summary.csv", index=False)
print("\nValidation summary saved to validation_summary.csv")
