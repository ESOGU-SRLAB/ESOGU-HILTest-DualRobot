"""
features.py
===========
Çevrimiçi öznitelik motoru — KTS'siz. 500 Hz örnekten 6 kanallı kalıntı
(r_tot) ve 16 kanallı ham (q_2..5, q̇_1..6, τ_1..6) öznitelik üretir.

v3'ten v4'e üç köklü değişiklik (hepsi ölçülerek, `anomaly_detection_v2/
dataset_prep/`de üretilen veri ve deneylerle karara bağlandı):

  1. KTS ÇIKARILDI. `ur10e_tcp_fts_sensor` fiziksel bir kuvvet sensörü DEĞİL —
     UR sürücüsü onu RTDE'nin `actual_TCP_force` alanından dolduruyor
     (kontrolcünün, AYARLI payload'a göre hesapladığı bir tahmin). Payload
     ayarı gerçek takımla uyuşmazsa (görev değişince hep uyuşmuyor) bu "kuvvet"
     durağan pozda bile onlarca N sahte sapma veriyor. Eski r_int/r_ext
     ayrıştırması (12 kanal) bu yüzden kimliklenemez; tek kalıntı kanalı var:
         r_tot = τ_ölç − τ_model − (sürtünme + ofset + yük düzeltmesi)
  2. YÜK DÜZELTMESİ SENARYO BAŞINA VE POZA BAĞLI. FMU sabit bir yük varsayıyor;
     gerçek takım (HRC/PICKPLACE/MULTIROBOT/UR10E_INSPECTION arası) farklı.
     Sabit bir ofset bunu genellemedi (val/test'te iki kat sapma bıraktı);
     kütle + ağırlık merkezi terimi (m[senaryo]·A(q) + u[senaryo]·B(q)) hem
     train hem val/test'te kalıntıyı küçülttü — ayrıntı: residual_calibration.json.
  3. HAM MODEL q_1 VE q_6'YI GÖRMÜYOR. İkisi de yerçekimi torkuna girmiyor
     (dikey eksen / sınırsız dönen bilek), yalnızca "kol o an nereye bakıyor"
     bilgisini taşıyor — arıza sinyali değil. Bunları tutmak, ham modelin daha
     önce hiç görmediği bir poza giden bir turu BAŞTAN SONA "anomali" saymasına
     yol açıyordu (%20-30 pencere).

Bu modül tek doğruluk kaynağıdır: `dataset_prep/features_v2.py` (bu dosyanın
kökeni) `verify_online_offline.py` ile eğitim tablosuna karşı sayısal olarak
doğrulandı (fark < 1e-6 Nm).
"""

from __future__ import annotations

from collections import deque

import numpy as np
from scipy.signal import savgol_coeffs

JOINT_SUFFIX = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# Ham modele giren eklemler (0 tabanlı indeks): q_1 (shoulder_pan) ve q_6
# (wrist_3) DIŞARIDA - bkz. modül docstring'i madde 3.
RAW_Q_IDX = [1, 2, 3, 4]

# UR10e resmi DH parametreleri (standart DH).
DH_A = np.array([0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0])
DH_D = np.array([0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655])
DH_ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])


def jacobian_and_rotation(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Tek poz için (taban çerçevesi geometrik Jacobian 6x6, flanşın taban
    çerçevesine göre dönüklüğü R 3x3). Standart DH ileri kinematik."""
    T = np.eye(4)
    zs = np.empty((6, 3))
    os_ = np.empty((6, 3))
    for i in range(6):
        zs[i] = T[:3, 2]
        os_[i] = T[:3, 3]
        ct, st = np.cos(q[i]), np.sin(q[i])
        ca, sa = np.cos(DH_ALPHA[i]), np.sin(DH_ALPHA[i])
        T = T @ np.array([
            [ct, -st * ca,  st * sa, DH_A[i] * ct],
            [st,  ct * ca, -ct * sa, DH_A[i] * st],
            [0.0, sa,       ca,      DH_D[i]],
            [0.0, 0.0,      0.0,     1.0],
        ])
    o_n = T[:3, 3]
    J = np.empty((6, 6))
    for i in range(6):
        J[:3, i] = np.cross(zs[i], o_n - os_[i])
        J[3:, i] = zs[i]
    return J, T[:3, :3]


class OnlineFeatureExtractor:
    """
    500 Hz örnekleri alır, `sg_window//2` gecikmeyle 6 kanal kalıntı + 16 kanal
    ham öznitelik üretir.

    Parametreler
    ------------
    solver         : ur10_solver_py.InverseDynamicsSolverUR10 örneği (FMU çekirdeği)
    nm_per_amp     : (6,) akım→tork katsayıları [Nm/A] — current_to_torque.json["nm_per_amp"]
    calibration    : residual_calibration.json içeriği (Fc, Fv, eps, g, b_by_use_case, payload_by_use_case)
    use_case       : hangi görev çalışıyor — HRC | MULTIROBOT_INSPECTION | PICKPLACE | UR10E_INSPECTION.
                     Yük/ofset düzeltmesi buna göre seçilir; YANLIŞ verilirse kalıntı
                     yüzlerce Nm sapabilir (bkz. residual_calibration.json'daki senaryo farkları).
    """

    def __init__(self, solver, nm_per_amp, calibration: dict, use_case: str,
                 dt=0.002, sg_window=51, sg_poly=3):
        if sg_window % 2 == 0:
            raise ValueError("sg_window tek sayı olmalı")
        if use_case not in calibration["b_by_use_case"]:
            raise ValueError(
                f"bilinmeyen use_case {use_case!r}; geçerli: "
                f"{sorted(calibration['b_by_use_case'])}")
        self.solver = solver
        self.nm_per_amp = np.asarray(nm_per_amp, dtype=np.float64).reshape(6)
        self.dt = float(dt)
        self.sg_window = int(sg_window)
        self.half = self.sg_window // 2
        self.sg = savgol_coeffs(sg_window, sg_poly, deriv=1, delta=dt, use="dot")
        self._buf: deque = deque(maxlen=self.sg_window)

        self.use_case = use_case
        self.eps = float(calibration["eps"])
        self.g = float(calibration["g"])
        self.fc = np.asarray(calibration["Fc"], dtype=np.float64)
        self.fv = np.asarray(calibration["Fv"], dtype=np.float64)
        self.b = np.asarray(calibration["b_by_use_case"][use_case], dtype=np.float64)
        pl = calibration["payload_by_use_case"][use_case]
        self.m = float(pl["m_kg"])
        self.u = np.asarray(pl["u_kg_m"], dtype=np.float64)

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

    def payload_terms(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """A (6,), B (6,3): ekstra yükün eklem torkuna katkısı τ_i = m·A_i + u·B_i.
        m·A_i: yükün KENDİSİ (kütle merkezi flanşta varsayılsın); u·B_i: ağırlık
        merkezinin flanştan ofseti (u = m·c). residual_calibration.json ile
        BİREBİR aynı formül — ayrıntı ve türetme orada."""
        J, R = jacobian_and_rotation(q)
        A = self.g * J[2, :]
        z = J[3:, :].T
        v = self.g * np.stack([-z[:, 1], z[:, 0], np.zeros(6)], axis=1)
        return A, v @ R

    # ── ana giriş ──
    def push(self, q, qd, effort_amps) -> dict | None:
        """
        Bir 500 Hz örneği ekler. Tampon dolmadıysa None; dolduysa pencerenin
        ORTASINDAKİ (yani `half` örnek geriden) örneğin öznitelikleri.
        """
        self._buf.append((np.asarray(q, np.float64).copy(),
                          np.asarray(qd, np.float64).copy(),
                          np.asarray(effort_amps, np.float64).copy()))
        if len(self._buf) < self.sg_window:
            return None
        return self._compute()

    def _compute(self) -> dict:
        QD = np.stack([s[1] for s in self._buf])          # (W, 6)
        qdd = self.sg @ QD                                  # merkez örneğin q̈'si
        q, qd, amps = self._buf[self.half]

        tau = amps * self.nm_per_amp                       # [Nm]
        tau_model = np.asarray(self.solver.getTorques(list(q), list(qd), list(qdd)),
                               dtype=np.float64).ravel()

        A, B = self.payload_terms(q)
        correction = (self.fc * np.tanh(qd / self.eps) + self.fv * qd + self.b
                     + self.m * A + B @ self.u)
        r_tot = tau - tau_model - correction

        raw_vec = np.concatenate([q[RAW_Q_IDX], qd, tau])   # 4 + 6 + 6 = 16

        return {
            "q": q, "qd": qd, "qdd": qdd, "tau": tau, "tau_model": tau_model,
            "r_tot": r_tot,
            "residual_vec": r_tot,          # 6 — models.py RESIDUAL_COLS sırasıyla
            "raw_vec": raw_vec,             # 16 — models.py RAW_COLS sırasıyla
        }
