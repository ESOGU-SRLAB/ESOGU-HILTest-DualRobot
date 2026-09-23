#!/usr/bin/env python3
"""
generate_residuals.py
=====================
Hibrit artık parquet'ini SIFIRDAN, çalışan ters dinamik modeliyle üretir.

    r_top(t) = τ_ölç(t) − τ̂_model(q, q̇, q̈)      ← FMU / C++ solver
    r_dis(t) = J(q)ᵀ · F_KTS(t)                    ← Jacobian aktarımı
    r_ic(t)  = r_top(t) − r_dis(t)

Eskisinden farkları
-------------------
* Girdi olarak 3,7 GB CSV yerine `ur10e_raw_features.parquet` okunur — q, q̇, τ ve
  KTS zaten orada, satır sırası birebir aynı. Hem çok hızlı hem hizalama garantili.
* q̈ türevi tüm dizi üzerinde tek seferde alınır; eski kod 20k'lık chunk'lar arasında
  50 örneklik tampon taşıyordu, chunk sınırlarında küçük artefaktlar oluşuyordu.
* Jacobian vektörleştirildi (eskisi 1,1 milyon kez Python döngüsü çeviriyordu).
  Doğruluğu, senin `ur10e_jacobian.py` dosyandaki referans fonksiyona karşı
  her koşuda otomatik doğrulanır.
* **τ_model sıfır çıkarsa program durur.** Eski hat sessizce sıfır yazıp geçiyordu;
  mevcut parquet'in bozuk olmasının sebebi tam olarak buydu.

Kullanım
--------
    python3 generate_residuals.py --backend so
    python3 generate_residuals.py --backend fmu --limit-rows 20000 --out check.parquet
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path.cwd()))
from fmu_backend import make_backend, assert_nonzero, SolverUnavailable  # noqa: E402

JN = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

# Senin ur10e_jacobian.py dosyandakiyle BİREBİR aynı DH tablosu.
DH_A = np.array([0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0])
DH_D = np.array([0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655])
DH_ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])


def jacobians_and_rotation(Q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(N,6) eklem açısı → (taban çerçevesinde geometrik Jacobian, R₀₆ dönme matrisi)."""
    N = len(Q)
    T = np.zeros((N, 4, 4))
    T[:, 0, 0] = T[:, 1, 1] = T[:, 2, 2] = T[:, 3, 3] = 1.0
    zs = np.empty((N, 6, 3))
    os_ = np.empty((N, 6, 3))
    for i in range(6):
        zs[:, i] = T[:, :3, 2]
        os_[:, i] = T[:, :3, 3]
        ct, st = np.cos(Q[:, i]), np.sin(Q[:, i])
        ca, sa = np.cos(DH_ALPHA[i]), np.sin(DH_ALPHA[i])
        Ti = np.zeros((N, 4, 4))
        Ti[:, 0, 0] = ct;      Ti[:, 0, 1] = -st * ca; Ti[:, 0, 2] = st * sa
        Ti[:, 0, 3] = DH_A[i] * ct
        Ti[:, 1, 0] = st;      Ti[:, 1, 1] = ct * ca;  Ti[:, 1, 2] = -ct * sa
        Ti[:, 1, 3] = DH_A[i] * st
        Ti[:, 2, 1] = sa;      Ti[:, 2, 2] = ca;       Ti[:, 2, 3] = DH_D[i]
        Ti[:, 3, 3] = 1.0
        T = T @ Ti
    o_n = T[:, :3, 3]
    J = np.empty((N, 6, 6))
    for i in range(6):
        J[:, :3, i] = np.cross(zs[:, i], o_n - os_[:, i])
        J[:, 3:, i] = zs[:, i]
    return J, T[:, :3, :3]


def jacobians(Q: np.ndarray) -> np.ndarray:
    return jacobians_and_rotation(Q)[0]


def pq_columns(path: str) -> list[str]:
    """Parquet şemasındaki kolon adları (dosyayı okumadan)."""
    import pyarrow.parquet as pq
    return list(pq.ParquetFile(path).schema.names)


def run_bounds(run_id: np.ndarray | None, n: int) -> list[tuple[int, int]]:
    """run_id kolonundan (başlangıç, uzunluk) listesi. Yoksa tek bir koşu."""
    if run_id is None:
        return [(0, n)]
    edges = np.flatnonzero(np.diff(run_id) != 0) + 1
    starts = np.concatenate([[0], edges])
    ends = np.concatenate([edges, [n]])
    return [(int(a), int(b - a)) for a, b in zip(starts, ends)]


def verify_jacobian(Q: np.ndarray) -> None:
    """Vektörleştirilmiş Jacobian'ı kullanıcının referans uygulamasına karşı doğrula."""
    try:
        from ur10e_jacobian import geometric_jacobian
    except ImportError:
        print("  ⚠ ur10e_jacobian.py bulunamadı — Jacobian çapraz kontrolü atlandı.")
        return
    idx = np.linspace(0, len(Q) - 1, 200).astype(int)
    mine = jacobians(Q[idx])
    ref = np.stack([geometric_jacobian(Q[i]) for i in idx])
    d = float(np.abs(mine - ref).max())
    print(f"  Jacobian çapraz kontrolü (ur10e_jacobian.py): maks fark {d:.3e} "
          f"{'✅' if d < 1e-9 else '❌'}")
    if d >= 1e-9:
        raise RuntimeError("Vektörleştirilmiş Jacobian referansla uyuşmuyor.")


def calibrate(tau_meas: np.ndarray, tau_model: np.ndarray, mode: str = "affine",
              ratio: float = 0.8, iters: int = 3) -> list[dict]:
    """
    Eklem başına  τ_ölç ≈ a·τ_model + b  uydurur.

    Neden gerekli: `dynamic_joint_states`'teki `effort` alanı Nm DEĞİL, motor AKIMI.
    UR'ın kendi ROS 2 sürücü belgesi (ur_robot_driver → Controllers) aynen şöyle diyor:

        "The effort field contains the currents reported by the joints
         and not the actual efforts in a physical sense."

    Fiziksel sağlama: UR10e'nin kolu yatayken shoulder_lift etrafındaki yerçekimi
    torku, resmi ur_description kütleleriyle ~121 Nm. Ölçülen effort'un tepesi ~10 —
    Nm olsaydı robot kendi kolunu kaldıramazdı. τ_model'in tepesi ise ~112 Nm, tutuyor.

    FMU ise Nm üretiyor. Doğrudan çıkarma elmayla armudu
    çıkarmak olur; ölçeklenmemiş model artığa hâkim olur ve artık küçüleceğine büyür.
    `a` eklemin etkin tork sabiti/redüktör oranını, `b` akım ofsetini soğurur.

    Uydurma yalnızca ilk `ratio` orandaki veride yapılır (eğitim böleni) ve
    3σ dışındaki noktalar iteratif olarak atılır ki anomaliler katsayıyı bozmasın.
    """
    n = max(int(len(tau_meas) * ratio), 1000)
    out = []
    for j in range(6):
        x, y = tau_model[:n, j], tau_meas[:n, j]
        m = np.ones(n, dtype=bool)
        a, b = 1.0, 0.0
        if mode == "offset":
            # Birim dışarıdan (quasi-statik yerçekimi) belirlendi; ölçeği yeniden
            # uydurmak modeli gürültüye büyütür. a sabit 1, yalnızca sabit sapma alınır.
            b = float(np.median(y - x))
            r_all = tau_meas[:, j] - (x_all := tau_model[:, j]) - b
            vy = float(tau_meas[:, j].var())
            out.append({
                "joint": JN[j], "a": 1.0, "b": b,
                "r2": float(1 - r_all.var() / vy) if vy > 0 else 0.0,
                "corr": float(np.corrcoef(tau_meas[:, j], x_all)[0, 1]),
                "std_tau_meas": float(tau_meas[:, j].std()),
                "std_tau_model": float(x_all.std()),
                "std_resid_naive": float((tau_meas[:, j] - x_all).std()),
                "std_resid_calibrated": float(r_all.std()),
                "inliers_frac": 1.0,
            })
            continue
        for _ in range(iters if mode != "none" else 0):
            if mode == "affine":
                M = np.vstack([x[m], np.ones(int(m.sum()))]).T
                sol, *_ = np.linalg.lstsq(M, y[m], rcond=None)
                a, b = float(sol[0]), float(sol[1])
            else:                                   # "scale": ofset yok
                a = float(x[m] @ y[m] / (x[m] @ x[m] + 1e-12)); b = 0.0
            r = y - (a * x + b)
            sd = float(r[m].std())
            if sd < 1e-12:
                break
            m = np.abs(r - float(r[m].mean())) < 3 * sd
        r_all = tau_meas[:, j] - (a * tau_model[:, j] + b)
        r_naive = tau_meas[:, j] - tau_model[:, j]
        vy = float(tau_meas[:, j].var())
        out.append({
            "joint": JN[j], "a": a, "b": b,
            "r2": float(1 - r_all.var() / vy) if vy > 0 else 0.0,
            "corr": float(np.corrcoef(tau_meas[:, j], tau_model[:, j])[0, 1]),
            "std_tau_meas": float(tau_meas[:, j].std()),
            "std_tau_model": float(tau_model[:, j].std()),
            "std_resid_naive": float(r_naive.std()),
            "std_resid_calibrated": float(r_all.std()),
            "inliers_frac": float(m.mean()),
        })
    return out


# ───────────────────────── sürtünme ─────────────────────────

def friction_basis(qd: np.ndarray, eps: float) -> np.ndarray:
    """
    (N,) hız → (N, 2) taban [tanh(q̇/eps), q̇]  yani [Coulomb, viskoz].

    sign(q̇) yerine tanh(q̇/eps): işaret fonksiyonu sıfır geçişinde süreksizdir ve
    bu robot düşük hızda çok zaman geçirir. Keskin sign ile kalıntıya her yön
    değişiminde bir basamak enjekte ederdik — tam da anomali sandığımız şeyi.
    """
    return np.stack([np.tanh(qd / eps), qd], axis=1)


def fit_friction(r_total: np.ndarray, QD: np.ndarray, mask: np.ndarray,
                 vmin: float, eps: float, joints: list[str],
                 clip_sigma: float = 3.0, iters: int = 3) -> list[dict]:
    """
    Eklem başına τ_f = Fc·tanh(q̇/eps) + Fv·q̇ katsayılarını sağlam biçimde uydurur.

    `mask` YALNIZ eğitim koşularının geçerli örneklerini işaretler. Katsayılar test
    verisini görürse sürtünme düzeltmesinin kendisi bir sızıntı kanalı olur.

    Sağlamlık: 3σ'yı aşan artıklar atılıp yeniden uydurulur. Kalibrasyon adımıyla
    aynı disiplin — tek bir çarpışma anı Fc'yi kaydırmasın diye.

    UYARI (raporlanmalı): bu regresyon hıza bağlı HER model hatasını soğurur,
    yalnız sürtünmeyi değil. Uygun bir sürtünme modeli olduğunu iddia etmiyoruz;
    ölçtüğümüz şey, kalıntının hıza bağlı ne kadarının açıklanabildiğidir.
    """
    out = []
    for j in range(6):
        m = mask & (np.abs(QD[:, j]) > vmin)
        y = r_total[:, j]
        X = friction_basis(QD[:, j], eps)
        n0 = int(m.sum())
        keep = m.copy()
        coef = np.zeros(2)
        for _ in range(iters):
            if keep.sum() < 100:
                break
            coef, *_ = np.linalg.lstsq(X[keep], y[keep], rcond=None)
            res = y - X @ coef
            sd = res[keep].std()
            if sd < 1e-12:
                break
            keep = m & (np.abs(res - res[keep].mean()) < clip_sigma * sd)
        pred = X @ coef
        ss_res = float(((y[m] - pred[m]) ** 2).sum())
        ss_tot = float(((y[m] - y[m].mean()) ** 2).sum())
        out.append({
            "joint": joints[j], "Fc": float(coef[0]), "Fv": float(coef[1]),
            "n_moving": n0, "n_used": int(keep.sum()),
            "r2": float(1.0 - ss_res / ss_tot) if ss_tot > 0 else 0.0,
            "std_before": float(y[m].std()), "std_after": float((y - pred)[m].std()),
        })
    return out


def fmt(s: float) -> str:
    return f"{s:.0f}s" if s < 60 else (f"{s/60:.1f}dk" if s < 3600 else f"{s/3600:.2f}sa")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default="ur10e_raw_features.parquet")
    ap.add_argument("--out", default="ur10e_hybrid_residual.parquet")
    ap.add_argument("--backend", choices=["so", "fmu"], default="so")
    ap.add_argument("--resources", default="resources")
    ap.add_argument("--fmu", default="UR10e_InverseDynamics.fmu")
    ap.add_argument("--sg-window", type=int, default=51)
    ap.add_argument("--sg-poly", type=int, default=3)
    ap.add_argument("--chunk", type=int, default=100_000, help="Jacobian parça boyutu")
    ap.add_argument("--calibrate", choices=["offset", "affine", "scale", "none"], default="affine",
                    help="τ_ölç ile τ_model arasındaki birim/ölçek uyumu "
                         "(effort alanı Nm değil motor akımı geliyor)")
    ap.add_argument("--calib-ratio", type=float, default=0.8,
                    help="kalibrasyonun uydurulacağı ilk veri oranı")
    ap.add_argument("--min-r2", type=float, default=0.05,
                    help="bu R²'nin altında kalan eklemlerde model katkısı sıfırlanır "
                         "(a=0). Yerçekimi yükü taşımayan eklemlerde a gürültüden "
                         "uydurulur, hatta işareti ters çıkabilir.")
    ap.add_argument("--calib-out", default="residual_calibration.json")
    ap.add_argument("--calibration-in", default=None,
                    help="kalibrasyon katsayılarını YENİDEN UYDURMA, bu dosyadan oku. "
                         "Arıza enjekte edilmiş veride şart: offset modunda b = "
                         "ortalama(τ_ölç − τ_model) tüm veriden hesaplanır, enjekte "
                         "edilen arıza b'yi kaydırır ve DOKUNULMAMIŞ koşuların "
                         "kalıntısı da değişir. Düğüm de zaten sabit bir b kullanıyor.")
    ap.add_argument("--units", choices=["current", "nm"], default="current",
                    help="artıkların birimi. current (varsayılan) = ölçüm uzayı, güvenli. "
                         "nm = 1/a ile bölme; yerçekimi yükü olmayan eklemlerde a belirsiz "
                         "olduğu için bu kanalları patlatabilir.")
    ap.add_argument("--fts-frame", choices=["tool", "base"], default="tool",
                    help="KTS wrench'inin geldiği çerçeve. UR sürücüsü e-Series'te "
                         "actual_TCP_force'u TCP çerçevesine döndürür ve frame_id=tool0 "
                         "yayınlar → 'tool' (varsayılan, doğru olan). 'base' eski/hatalı "
                         "davranışı korur (bildiri erratum'u için).")
    ap.add_argument("--friction", choices=["none", "fit", "file"], default="none",
                    help="FMU'da sürtünme modeli YOK (500 pozda sürtünme vektörü "
                         "birebir sıfır ölçüldü). 'fit' Coulomb+viskoz terimi YALNIZ "
                         "eğitim koşularından uydurur ve r_top'tan çıkarır; 'file' "
                         "önceden uydurulmuş katsayıları okur. FMU'nun kendisine "
                         "dokunulmaz — terim çözücünün DIŞINDA, kalıntı tanımındadır.")
    ap.add_argument("--friction-model", default="friction_model.json",
                    help="--friction fit ile yazılır, --friction file ile okunur")
    ap.add_argument("--splits", default="splits.json",
                    help="make_splits.py çıktısı; sürtünme YALNIZ eğitim koşularından "
                         "uydurulur, yoksa katsayılar test verisini görür")
    ap.add_argument("--friction-vmin", type=float, default=0.02,
                    help="uydurmaya giren en düşük |q̇| [rad/s]. Duruşta sign(q̇) "
                         "tanımsız ve Coulomb terimi ölçülemez.")
    ap.add_argument("--friction-eps", type=float, default=0.02,
                    help="tanh(q̇/eps) yumuşatma genişliği [rad/s]. sign(q̇) sıfır "
                         "geçişinde süreksiz ve robot düşük hızda çok zaman geçiriyor; "
                         "keskin sign kalıntıya gürültü enjekte eder.")
    ap.add_argument("--limit-rows", type=int, default=None)
    args = ap.parse_args()

    t_wall = time.time()
    print("=" * 70)
    print("HİBRİT ARTIK ÜRETİMİ")
    print("=" * 70)
    print(f"  girdi   : {args.raw}")
    print(f"  çıktı   : {args.out}")
    print(f"  arka uç : {args.backend}")

    # ── veri ──
    cols = (["t"] + [f"q_{j}" for j in range(1, 7)] + [f"qd_{j}" for j in range(1, 7)]
            + [f"tau_{j}" for j in range(1, 7)] + ["fx", "fy", "fz", "tx", "ty", "tz"])
    have = set(pq_columns(args.raw))
    extra = [c for c in ("run_id", "interp") if c in have]
    df = pd.read_parquet(args.raw, columns=cols + extra)
    if args.limit_rows:
        df = df.iloc[:args.limit_rows]
    t = df["t"].to_numpy(np.float64)
    Q = df[[f"q_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    QD = df[[f"qd_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    TAU = df[[f"tau_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    FTS = df[["fx", "fy", "fz", "tx", "ty", "tz"]].to_numpy(np.float64)
    RUN = df["run_id"].to_numpy(np.int32) if "run_id" in extra else None
    INTERP = df["interp"].to_numpy(bool) if "interp" in extra else None
    del df
    N = len(t)
    dts = np.diff(t); dts = dts[dts > 0]
    dt = float(np.median(dts)) if dts.size else 0.002
    runs = run_bounds(RUN, N)
    print(f"  {N:,} örnek   dt = {dt*1000:.3f} ms  ({1/dt:.1f} Hz)   ({fmt(time.time()-t_wall)})")
    if RUN is None:
        print(f"  ⚠ 'run_id' kolonu yok → tüm dizi TEK kesintisiz koşu varsayılıyor.")
        print(f"    Kaynak veri kopukluk içeriyorsa q̈ bozulur; prepare_dataset.py çalıştır.")
    else:
        print(f"  {len(runs):,} kesintisiz koşu (medyan {int(np.median([L for _, L in runs])):,} örnek)")

    # ── q̈ ──
    print(f"\n[1] q̈ — Savitzky-Golay (pencere {args.sg_window}, derece {args.sg_poly})")
    half = args.sg_window // 2
    QDD = np.zeros_like(QD)
    VALID = np.zeros(N, bool)
    short = 0
    for a_, L in runs:
        sl = slice(a_, a_ + L)
        if L < args.sg_window:
            short += L
            continue
        dtr = np.diff(t[sl])
        dtr = float(np.median(dtr[dtr > 0])) if (dtr > 0).any() else dt
        QDD[sl] = savgol_filter(QD[sl], window_length=args.sg_window,
                                polyorder=args.sg_poly, deriv=1, delta=dtr,
                                axis=0, mode="interp")
        VALID[a_ + half:a_ + L - half] = True      # filtre kenar etkisini dışla
    if short:
        print(f"  ⚠ {short:,} örnek {args.sg_window}'den kısa koşularda → geçersiz işaretlendi")
    print(f"  geçerli örnek: {int(VALID.sum()):,} (%{100*VALID.mean():.1f}) "
          f"— kenar payı koşu başına ±{half}")
    print(f"  q̈ std (geçerli, eklem başına) = {np.round(QDD[VALID].std(axis=0), 4)}")

    # ── τ_model ──
    print(f"\n[2] τ_model — ters dinamik ({args.backend})")
    try:
        be = make_backend(args.backend, args.resources, args.fmu)
    except SolverUnavailable as e:
        print(f"\n❌ {e}\n", file=sys.stderr)
        return 2

    t0 = time.time()

    def prog(k, n):
        if k == 0:
            return
        el = time.time() - t0
        eta = el / k * (n - k)
        print(f"\r  {k:>10,}/{n:,}  ({100*k/n:5.1f}%)  geçen {fmt(el)}  kalan ~{fmt(eta)}   ",
              end="", flush=True)

    TAU_M = be.torques(Q, QD, QDD, progress=prog)
    be.close()
    print()
    assert_nonzero(TAU_M, "(τ_model üretimi)")
    print(f"  ✅ τ_model üretildi ({fmt(time.time()-t0)})")
    print(f"  τ_model std (geçerli, eklem başına) = {np.round(TAU_M[VALID].std(axis=0), 4)}")

    # ── artıklar ──
    print("\n[3] Artıklar")
    verify_jacobian(Q)

    # Kalibrasyon yalnızca geçerli (filtre kenarı olmayan) örneklerden uydurulur.
    vi = np.flatnonzero(VALID)
    cal = calibrate(TAU[vi], TAU_M[vi], mode=args.calibrate, ratio=args.calib_ratio)
    print(f"\n  Ölçek kalibrasyonu (mod: {args.calibrate}) — τ_ölç ≈ a·τ_model + b")
    print(f"  {'eklem':<15}{'a':>10}{'b':>9}{'korel':>8}{'R²':>7}"
          f"{'artık (ham)':>13}{'artık (kal.)':>14}{'kazanç':>9}")
    print("  " + "-" * 85)
    for c in cal:
        g = c["std_resid_naive"] / max(c["std_resid_calibrated"], 1e-12)
        print(f"  {c['joint']:<15}{c['a']:>10.4f}{c['b']:>9.3f}{c['corr']:>8.3f}"
              f"{c['r2']:>7.3f}{c['std_resid_naive']:>13.3f}"
              f"{c['std_resid_calibrated']:>14.3f}{g:>8.1f}x")
    # İKİ AYRI KATSAYI — karıştırılmamalı:
    #
    #   A_model : τ_model'i ölçüm uzayına taşır. Modelin hiçbir şey açıklamadığı
    #             eklemlerde (R² ≈ 0) a gürültüden uyduruluyor — wrist_3'te işareti
    #             bile ters çıkıyor. Oralarda 0'a çekiyoruz: ters dinamik o eklemde
    #             bilgi vermiyor, artık ölçümün kendisi olsun. Dürüst olan bu.
    #
    #   A_unit  : Nm → akım birim dönüşümü. r_ext = J(q)ᵀF fiziksel bir Nm büyüklüğü
    #             ve HER eklemde geçerli; bunu sıfırlarsak o eklemde dış etkileşim
    #             sinyalini tamamen kaybederiz. R²'si düşük eklemler için aynı boyut
    #             ailesindeki (τ_max'ı eşit) güvenilir eklemin katsayısı kullanılır.
    TAU_MAX = [330, 330, 150, 56, 56, 56]      # UR10e eklem tork sınırları
    if args.calibrate == "offset":
        # Birim zaten dışarıdan sabitlendi (calibrate_current_to_torque.py → Nm).
        # Ne birim dönüşümü ne de model sıfırlaması gerekir: modelin az açıkladığı
        # eklemde küçük bir τ_model'i çıkarmak zararsız, artık ölçümün kendisi kalır.
        A_unit = np.ones(6)
        for c in cal:
            c["a_unit"] = 1.0
        print("  (mod: offset → birim ölçeği dışarıdan sabit, model sıfırlaması yok)")
    else:
        A_unit = np.array([c["a"] for c in cal], dtype=float)
        fam: dict[int, list[int]] = {}
        for j, tm in enumerate(TAU_MAX):
            fam.setdefault(tm, []).append(j)
        for tm, members in fam.items():
            good = [j for j in members if cal[j]["r2"] >= args.min_r2]
            if not good:
                continue
            best = max(good, key=lambda j: cal[j]["r2"])
            for j in members:
                if cal[j]["r2"] < args.min_r2:
                    A_unit[j] = cal[best]["a"]
                    cal[j]["a_unit_from"] = cal[best]["joint"]

        for j, c in enumerate(cal):
            c["a_unit"] = float(A_unit[j])
            if c["r2"] < args.min_r2:
                c["a_model_raw"] = c["a"]
                c["a"], c["b"] = 0.0, float(TAU[:, j].mean())
                c["model_zeroed"] = True
        zz = [c["joint"] for c in cal if c.get("model_zeroed")]
        if zz:
            print(f"  ⚠ R² < {args.min_r2}: {', '.join(zz)} → model katkısı sıfırlandı "
                  f"(artık = ölçüm − ortalama); birim dönüşümü aile katsayısından alındı")
    A = np.array([c["a"] for c in cal])          # model katkısı
    B = np.array([c["b"] for c in cal])
    if args.calibration_in:
        cj = json.loads(Path(args.calibration_in).read_text(encoding="utf-8"))
        A = np.array(cj["a_model"], dtype=float)
        B = np.array(cj["b"], dtype=float)
        A_unit = np.array(cj["a"], dtype=float)
        for j, c in enumerate(cal):
            c["a"], c["b"], c["a_unit"] = float(A[j]), float(B[j]), float(A_unit[j])
            c["frozen_from"] = str(args.calibration_in)
        print(f"  kalibrasyon DONDURULDU ← {args.calibration_in}")
        print(f"    b = {np.round(B, 4).tolist()}")
    Path(args.calib_out).write_text(json.dumps(
        {"mode": args.calibrate, "ratio": args.calib_ratio,
         "residual_units": args.units,
         "note": "tau_meas(akim) ~ a*tau_model(Nm) + b. Artiklar olcum uzayinda; "
                 "Nm'ye cevirmek icin a'ya bol (dusuk R2'li eklemlerde guvenilmez). "
                 "r_ext de a ile carpilarak ayni uzaya tasindi.",
         "a": [c["a_unit"] for c in cal],          # birim dönüşümü (arıza genlikleri için)
         "a_model": [c["a"] for c in cal],         # model katkısı (düşük R²'de 0)
         "b": [c["b"] for c in cal],
         "r2": [c["r2"] for c in cal],
         "nm_per_unit": [1.0 / c["a"] if abs(c["a"]) > 1e-9 else None for c in cal],
         "per_joint": cal},
        indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"  → {args.calib_out}  (ROS 2 node'u aynı katsayıları kullanacak)")

    # ── Birim tutarlılığı ──────────────────────────────────────────────
    # τ_ölç amper, τ_model Nm, J(q)ᵀ·F ise Nm. Üçünü tek uzayda buluşturmak şart,
    # yoksa r_ic = r_top − r_dis yine elmayla armut çıkarır.
    #
    # Modeli ÖLÇÜM uzayına taşıyoruz (a ile çarpma), ölçümü modele değil (a'ya bölme):
    # yerçekimi yükü taşımayan eklemlerde a belirsiz (R²≈0) ve ona bölmek o kanalı
    # patlatıyor (ör. shoulder_pan'da 1/a ≈ 96 → sahte 92 Nm).
    # İki gösterim zaten kanal başına sabit çarpanla denk: r_A = a · r_Nm.
    #
    # ÇERÇEVE (denetim bulgusu F3): UR sürücüsü actual_TCP_force'u RTDE'den taban
    # çerçevesinde okur, sonra transformForceTorque() ile TCP çerçevesine döndürür
    # (hardware_interface.cpp:1074) ve mesajı frame_id=tool0 ile yayınlar. Jacobian
    # ise taban çerçevesinde. Çarpımdan önce wrench'i tabana geri döndürmek ŞART;
    # döndürmezsek üretilen r_dis, olması gerekenle ilişkisiz çıkıyor (korel ≈ 0).
    R_TOT = TAU - (A * TAU_M + B)

    # ── sürtünme (FMU'nun DIŞINDA) ──────────────────────────────────────
    # Denetim bulgusu: 500 rastgele pozda çözücünün sürtünme vektörü birebir sıfır.
    # Gerçek redüktörlü bir robotta sürtünme torkun büyük kısmıdır — özellikle
    # yerçekimi ve atalet torklarının küçük olduğu bileklerde, ki orada r_ic
    # pratikte ham ölçüme dönüşüyor.
    #
    # FMU'ya DOKUNMUYORUZ (gerçek robotla doğrulandı). Terim çözücünün içine değil,
    # kalıntı TANIMININ dışına giriyor:
    #     r_top = τ_ölç − τ̂_model − τ̂_f(q̇)
    # Çözücünün tek satırı değişmiyor, doğrulaması geçerli kalıyor.
    FRIC = None
    fr_rows = None
    if args.friction != "none":
        if args.friction == "fit":
            tr_runs = set(json.loads(Path(args.splits).read_text(encoding="utf-8"))["train"])
            if RUN is None:
                print("\n❌ --friction fit için run_id gerekli.", file=sys.stderr)
                return 2
            fit_mask = VALID & np.isin(RUN, list(tr_runs))
            print(f"\n[sürtünme] Coulomb+viskoz, YALNIZ {len(tr_runs):,} eğitim koşusundan "
                  f"({int(fit_mask.sum()):,} geçerli örnek)")
            fr_rows = fit_friction(R_TOT, QD, fit_mask, args.friction_vmin,
                                   args.friction_eps, JN)
            Path(args.friction_model).write_text(json.dumps(
                {"model": "Fc*tanh(qd/eps) + Fv*qd", "eps": args.friction_eps,
                 "vmin": args.friction_vmin, "units": args.units,
                 "fitted_on": "train runs only", "n_train_runs": len(tr_runs),
                 "splits": str(args.splits),
                 "caveat": "Regresyon hiza bagli HER model hatasini sogurur, yalniz "
                           "surtunmeyi degil. R2 ornek-ici degil, egitim kosularindan; "
                           "ayrilmis kosulardaki degeri evaluate_fusion.py raporlar.",
                 "Fc": [r["Fc"] for r in fr_rows], "Fv": [r["Fv"] for r in fr_rows],
                 "per_joint": fr_rows},
                indent=2, ensure_ascii=False), encoding="utf-8")
            print(f"  → {args.friction_model}  (çevrimiçi motor aynı katsayıları okuyacak)")
        else:
            fj = json.loads(Path(args.friction_model).read_text(encoding="utf-8"))
            fr_rows = fj.get("per_joint")
            args.friction_eps = float(fj["eps"])
            print(f"\n[sürtünme] katsayılar {args.friction_model} dosyasından okundu "
                  f"(eps={args.friction_eps})")

        Fc = np.array([r["Fc"] for r in fr_rows])
        Fv = np.array([r["Fv"] for r in fr_rows])
        FRIC = np.tanh(QD / args.friction_eps) * Fc + QD * Fv
        print(f"  {'eklem':<16}{'Fc':>10}{'Fv':>10}{'R²':>8}"
              f"{'std önce':>11}{'std sonra':>11}{'düşüş':>8}")
        print("  " + "-" * 74)
        for j, r in enumerate(fr_rows):
            drop = 100 * (1 - r["std_after"] / max(r["std_before"], 1e-12))
            print(f"  {r['joint']:<16}{r['Fc']:>10.3f}{r['Fv']:>10.3f}{r['r2']:>8.3f}"
                  f"{r['std_before']:>11.3f}{r['std_after']:>11.3f}{drop:>7.0f}%")
        R_TOT = R_TOT - FRIC

    R_EXT = np.empty((N, 6))
    t0 = time.time()
    for a_ in range(0, N, args.chunk):
        b_ = min(a_ + args.chunk, N)
        J, R06 = jacobians_and_rotation(Q[a_:b_])
        W = FTS[a_:b_]
        if args.fts_frame == "tool":
            W = np.concatenate([np.einsum("nij,nj->ni", R06, W[:, :3]),
                                np.einsum("nij,nj->ni", R06, W[:, 3:])], axis=1)
        R_EXT[a_:b_] = np.einsum("nji,nj->ni", J, W)
        del J, R06, W
    R_EXT *= A_unit                 # Nm → ölçüm (akım) uzayı (A_model DEĞİL)
    R_INT = R_TOT - R_EXT

    if args.units == "nm" and args.calibrate != "offset":
        bad = [c["joint"] for c in cal if c["r2"] < 0.3]
        if bad:
            print(f"  ⚠ --units nm: {', '.join(bad)} için a belirsiz (R²<0,3); "
                  f"bu kanallar güvenilmez.")
        R_TOT, R_EXT, R_INT = R_TOT / A_unit, R_EXT / A_unit, R_INT / A_unit
    print(f"  Jacobian aktarımı tamam ({fmt(time.time()-t0)})")

    print(f"\n  {'eklem':<16}{'τ_ölç std':>12}{'τ_model std':>13}{'r_top std':>11}"
          f"{'r_dis std':>11}{'r_ic std':>11}")
    print("  " + "-" * 74)
    for j in range(6):
        print(f"  {JN[j]:<16}{TAU[vi, j].std():>12.4f}{TAU_M[vi, j].std():>13.4f}"
              f"{R_TOT[vi, j].std():>11.4f}{R_EXT[vi, j].std():>11.4f}{R_INT[vi, j].std():>11.4f}")

    if np.allclose(R_TOT, TAU, atol=1e-9):
        print("\n❌ r_top ile τ_ölç aynı → τ_model sıfır. Durduruluyor.", file=sys.stderr)
        return 2

    # ── yaz ──
    out = {"t": t}
    for j in range(6):
        out[f"r_total_{j+1}"] = R_TOT[:, j]
        out[f"r_ext_{j+1}"] = R_EXT[:, j]
        out[f"r_int_{j+1}"] = R_INT[:, j]
    out["r_total_norm"] = np.linalg.norm(R_TOT, axis=1)
    out["r_ext_norm"] = np.linalg.norm(R_EXT, axis=1)
    out["r_int_norm"] = np.linalg.norm(R_INT, axis=1)
    for j in range(6):
        out[f"tau_model_{j+1}"] = TAU_M[:, j]        # ham model çıktısı (denetlenebilirlik)
        if FRIC is not None:
            out[f"tau_fric_{j+1}"] = FRIC[:, j]      # çıkarılan sürtünme (denetlenebilirlik)
        out[f"tau_model_cal_{j+1}"] = A[j] * TAU_M[:, j] + B[j]   # kalibre edilmiş
    # Ham kanallar da aynı dosyaya taşınır: iki model TEK parquet'ten beslenince
    # pencere kümeleri (koşu sınırı + geçerlilik maskesi) birebir aynı olur ve
    # skor düzeyinde birleşim hizalanma garantisi kazanır.
    for j in range(6):
        out[f"q_{j+1}"] = Q[:, j]
        out[f"qd_{j+1}"] = QD[:, j]
        out[f"tau_{j+1}"] = TAU[:, j]
    for k, nm in enumerate(["fx", "fy", "fz", "tx", "ty", "tz"]):
        out[nm] = FTS[:, k]
    out["valid"] = VALID                              # SG kenar payı dışındaki örnekler
    if RUN is not None:
        out["run_id"] = RUN
    if INTERP is not None:
        out["interp"] = INTERP
    pd.DataFrame(out).to_parquet(args.out, index=False)

    sz = Path(args.out).stat().st_size / 1e6
    print(f"\nTAMAM → {args.out}  ({N:,} satır, {sz:.1f} MB, {fmt(time.time()-t_wall)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
