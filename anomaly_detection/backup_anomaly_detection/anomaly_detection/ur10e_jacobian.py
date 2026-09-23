"""
UR10e Analytical Jacobian (Geometric Jacobian)
Based on official UR10e DH parameters from Universal Robots.

Convention: Modified DH (Craig)
"""

import numpy as np
from typing import Tuple

# ────────────── UR10e DH Parameters (official) ──────────────
# Joint | a [m]     | d [m]    | alpha [rad]
DH_A     = np.array([0.0,     -0.6127,  -0.57155,  0.0,      0.0,      0.0     ])
DH_D     = np.array([0.1807,   0.0,      0.0,      0.17415,  0.11985,  0.11655 ])
DH_ALPHA = np.array([np.pi/2,  0.0,      0.0,      np.pi/2, -np.pi/2,  0.0     ])


def _dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Standard DH homogeneous transformation matrix."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d     ],
        [0,   0,        0,       1     ],
    ])


def forward_kinematics_all(q: np.ndarray) -> list:
    """
    Compute the homogeneous transformation matrix for each frame.
    
    Parameters
    ----------
    q : (6,) joint angles in radians
    
    Returns
    -------
    T_list : list of 7 (4x4) matrices, T_list[0] = I (base), T_list[i] = T_0_i
    """
    T = np.eye(4)
    T_list = [T.copy()]
    for i in range(6):
        Ti = _dh_matrix(q[i], DH_D[i], DH_A[i], DH_ALPHA[i])
        T = T @ Ti
        T_list.append(T.copy())
    return T_list


def geometric_jacobian(q: np.ndarray) -> np.ndarray:
    """
    Compute the 6x6 geometric Jacobian for UR10e.
    
    J = [Jv]   where Jv_i = z_{i-1} x (o_n - o_{i-1})   (linear)
        [Jw]         Jw_i = z_{i-1}                        (angular)
    
    All joints are revolute.
    
    Parameters
    ----------
    q : (6,) joint angles in radians
    
    Returns
    -------
    J : (6, 6) geometric Jacobian in base frame
    """
    T_list = forward_kinematics_all(q)
    o_n = T_list[6][:3, 3]  # end-effector position
    
    J = np.zeros((6, 6))
    for i in range(6):
        z = T_list[i][:3, 2]          # z-axis of frame i (revolute)
        o = T_list[i][:3, 3]          # origin of frame i
        J[:3, i] = np.cross(z, o_n - o)   # linear velocity
        J[3:, i] = z                        # angular velocity
    return J


def jacobian_transpose_force_mapping(q: np.ndarray, wrench_tcp: np.ndarray) -> np.ndarray:
    """
    Map a 6D wrench at TCP to joint torques via J^T.
    
    τ_ext = J(q)^T · F_tcp
    
    Parameters
    ----------
    q          : (6,) joint angles [rad]
    wrench_tcp : (6,) [fx, fy, fz, tx, ty, tz] in base frame
    
    Returns
    -------
    tau_ext : (6,) estimated joint torques from external force
    """
    J = geometric_jacobian(q)
    return J.T @ wrench_tcp


# ────────────── Quick self-test ──────────────
if __name__ == "__main__":
    # Home position (all zeros)
    q_home = np.zeros(6)
    J = geometric_jacobian(q_home)
    print("Jacobian at home position:")
    print(np.round(J, 4))
    
    T_all = forward_kinematics_all(q_home)
    print(f"\nTCP position at home: {np.round(T_all[6][:3, 3], 4)} m")
    
    # Test: 1N force in z -> joint torques
    F_test = np.array([0, 0, 1.0, 0, 0, 0])
    tau = jacobian_transpose_force_mapping(q_home, F_test)
    print(f"\nJoint torques from 1N Fz: {np.round(tau, 4)} Nm")
