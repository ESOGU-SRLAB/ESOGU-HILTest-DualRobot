"""
features.py
===========
Çevrimiçi öznitelik motoru — çevrimdışı hattın (generate_residuals.py) BİREBİR aynısı.

Neden ayrı bir modül
--------------------
Anomali tespitinde en sinsi hata, eğitim verisini üreten hesabın canlı hesaptan
milimetrik farklı olmasıdır: model o farkı anomali sanar ve eşik anlamını yitirir.
Bu modül tek doğruluk kaynağıdır; `verify_online_features.py` her değişiklikten
sonra çevrimdışı parquet'e karşı sayısal olarak doğrular (hedef: maks fark < 1e-9).

Hesap zinciri (245.pdf Denklem 1–3)
-----------------------------------
    τ_ölç   = i [A] · nm_per_amp            ← akım→tork (quasi-statik yerçekimi kalibrasyonu)
    q̈       = SG türevi (merkezli, 51/3)     ← 25 örnek gecikme yaratır, bilerek
    τ_model = FMU ters dinamik (q, q̇, q̈)
    r_top   = τ_ölç − τ_model − b
    r_dis   = J(q)ᵀ · (R₀₆(q) · F_KTS)       ← wrench tool0'dan tabana döndürülür
    r_ic    = r_top − r_dis

Gecikme
-------
Merkezli Savitzky-Golay penceresi q̈'yi ancak pencerenin ORTASINDA verir. Bu yüzden
üretilen öznitelik daima `sg_window//2` = 25 örnek (50 ms) geriden gelir. Nedensel
(tek yönlü) bir türev bu gecikmeyi sıfırlardı ama gürültüyü belirgin artırır ve
eğitimde kullanılan operatörden farklı olurdu — o yüzden gecikme kabul ediliyor.
"""

from __future__ import annotations

from collections import deque

import numpy as np
from scipy.signal import savgol_coeffs

# UR10e resmi DH parametreleri (standart DH). generate_residuals.py ile aynı tablo.
DH_A = np.array([0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0])
DH_D = np.array([0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655])
DH_ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])

JOINT_SUFFIX = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def jacobian_and_rotation(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Tek poz için (taban çerçevesi geometrik Jacobian 6×6, R₀₆ 3×3)."""
    T = np.eye(4)
    zs = np.empty((6, 3))
    os_ = np.empty((6, 3))
    for i in range(6):
        zs[i] = T[:3, 2]
        os_[i] = T[:3, 3]
        ct, st = np.cos(q[i]), np.sin(q[i])
        ca, sa = np.cos(DH_ALPHA[i]), np.sin(DH_ALPHA[i])
        Ti = np.array([
            [ct, -st * ca,  st * sa, DH_A[i] * ct],
            [st,  ct * ca, -ct * sa, DH_A[i] * st],
            [0.0, sa,       ca,      DH_D[i]],
            [0.0, 0.0,      0.0,     1.0],
        ])
        T = T @ Ti
    o_n = T[:3, 3]
    J = np.empty((6, 6))
    for i in range(6):
        J[:3, i] = np.cross(zs[i], o_n - os_[i])
        J[3:, i] = zs[i]
    return J, T[:3, :3]


class OnlineFeatureExtractor:
    """
    500 Hz örnekleri alır, `sg_window//2` gecikmeyle 12 kanal kalıntı + 24 kanal ham
    öznitelik üretir.

    Parametreler
    ------------
    solver       : ur10_solver_py.InverseDynamicsSolverUR10 örneği (FMU çekirdeği)
    nm_per_amp   : (6,) akım→tork katsayıları [Nm/A]  — current_to_torque.json
    offset_b     : (6,) τ_ölç − τ_model sabit sapması [Nm] — residual_calibration*.json
    fts_frame    : "tool" (UR sürücüsünün yayınladığı, doğru olan) veya "base"
    """

    def __init__(self, solver, nm_per_amp, offset_b=None, dt=0.002,
                 sg_window=51, sg_poly=3, fts_frame="tool"):
        if sg_window % 2 == 0:
            raise ValueError("sg_window tek sayı olmalı")
        self.solver = solver
        self.nm_per_amp = np.asarray(nm_per_amp, dtype=np.float64).reshape(6)
        self.b = (np.zeros(6) if offset_b is None
                  else np.asarray(offset_b, dtype=np.float64).reshape(6))
        self.dt = float(dt)
        self.sg_window = int(sg_window)
        self.half = self.sg_window // 2
        self.fts_frame = fts_frame
        # Merkezli SG türev çekirdeği: q̈ = coeffs · q̇[pencere]  (doğrudan iç çarpım)
        self.sg = savgol_coeffs(sg_window, sg_poly, deriv=1, delta=dt, use="dot")
        self._buf: deque = deque(maxlen=self.sg_window)

    # ── durum ──
    @property
    def ready(self) -> bool:
        return len(self._buf) == self.sg_window

    @property
    def lag_samples(self) -> int:
        return self.half

    @property
    def lag_seconds(self) -> float:
        return self.half * self.dt

    def reset(self) -> None:
        self._buf.clear()

    # ── ana giriş ──
    def push(self, q, qd, effort_amps, wrench) -> dict | None:
        """
        Bir 500 Hz örneği ekler. Tampon dolmadıysa None; dolduysa pencerenin
        ORTASINDAKİ (yani `half` örnek geriden) örneğin öznitelikleri.
        """
        self._buf.append((np.asarray(q, np.float64).copy(),
                          np.asarray(qd, np.float64).copy(),
                          np.asarray(effort_amps, np.float64).copy(),
                          np.asarray(wrench, np.float64).copy()))
        if len(self._buf) < self.sg_window:
            return None
        return self._compute()

    def _compute(self) -> dict:
        QD = np.stack([s[1] for s in self._buf])          # (W, 6)
        qdd = self.sg @ QD                                 # merkez örneğin q̈'si
        q, qd, amps, wr = self._buf[self.half]

        tau = amps * self.nm_per_amp                       # [Nm]
        tau_model = np.asarray(self.solver.getTorques(list(q), list(qd), list(qdd)),
                               dtype=np.float64).ravel()

        J, R06 = jacobian_and_rotation(q)
        w = wr.copy()
        if self.fts_frame == "tool":
            w[:3] = R06 @ wr[:3]
            w[3:] = R06 @ wr[3:]
        r_ext = J.T @ w
        r_tot = tau - tau_model - self.b
        r_int = r_tot - r_ext

        return {
            "q": q, "qd": qd, "qdd": qdd, "tau": tau, "tau_model": tau_model,
            "wrench": wr, "r_total": r_tot, "r_ext": r_ext, "r_int": r_int,
            # Model girdileri — models.py'deki RESIDUAL_COLS / RAW_COLS sırasıyla
            "residual_vec": np.concatenate([r_int, r_ext]),           # 12
            "raw_vec": np.concatenate([q, qd, tau, wr]),              # 24
        }
