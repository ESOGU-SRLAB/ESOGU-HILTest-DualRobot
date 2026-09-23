"""
stage_b_prepare.py
===================
AŞAMA B: Aşama A'nın çıktısından (`stage_a_clean.csv`), iki modelin
(kalıntı LSTM-AE + ham LSTM-AE) eğitiminde kullanılacak son öznitelik
tablosunu üretir. Çıktı, HER ÖRNEK için bir satır olan geniş bir CSV
(`stage_b_features.csv`) - pencere DEĞİL. Pencereleri (100 örnek, stride 25)
eğitim script'i, `run_id` ve `t` sütunlarını kullanarak kendisi kuracak.

BU SCRIPT ŞU AN ÇALIŞTIRILAMAZ:
`current_to_torque_v2.json` henüz yok - Pazartesi RTDE kalibrasyonundan
sonra üretilecek. Script o dosyayı bulamayınca AÇIK bir hata verip duracak
(yanlış/eski bir kalibrasyonla sessizce yanlış sonuç üretmek yerine).
Şimdilik amaç: kalibrasyon gelince TEK SATIR değiştirmeden çalışacak hale
getirmek.

Hesap zinciri (245.pdf Denklem 1-3; `anomaly_detection/anomaly_detection/
features.py` ile AYNI fizik - orada 500 Hz canlı akış için, burada 5,6
milyon satırlık toplu/çevrimdışı veri için):

    τ_ölç   = effort_amps * nm_per_amp        <- akım->tork (Pazartesi kalibre edilecek)
    q̈       = merkezli Savitzky-Golay türevi (51 örnek, run_id sınırını HİÇ geçmez)
    τ_model = FMU ters dinamik (anomaly_detection/resources - DOKUNULMAMIŞ,
              gerçek robotla doğrulanmış aynı .so çözücü)
    r_top   = τ_ölç - τ_model
    r_dis   = J(q)^T . (R06(q) . F_KTS)        <- dışsal kalıntı (KTS, taban çerçevesine döndürülmüş)
    r_ic    = r_top - r_dis                    <- içsel kalıntı

NOT - eksik olan iki kalibrasyon: eski hatta (`245.pdf`, `README.md`) bu
zincire ayrıca bir sabit ofset (`b`, eklem başına Nm) ve bir sürtünme
düzeltmesi (`Fc, Fv`) ekleniyordu (residual_calibration_fric.json,
friction_model.json). Bunlar bu script'te YOK - Monday'in nm_per_amp
kalibrasyonuyla iş bitmiyor, bu script'in çıktısı üzerinde AYRI bir
kalibrasyon adımı daha gerekecek (eski hattaki sıra: akım->tork, sonra
sürtünme, sonra ofset). Şimdilik r_int/r_ext "ham" - b=0, sürtünme yok.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

# ─────────────────────────── AYARLAR ───────────────────────────

HERE = Path(__file__).resolve().parent
INPUT_CSV = HERE / "data" / "stage_a_clean.csv"
OUTPUT_CSV = HERE / "data" / "stage_b_features.csv"

# Pazartesi'nin çıktısı buraya gelecek: her eklem için {a, c} -> amps ≈ a*tau + c
# ters çevrilip nm_per_amp = 1/a olarak mı, yoksa doğrudan nm_per_amp olarak mı
# kaydedileceğine kalibrasyon script'ini yazarken karar vereceğiz. Şimdilik
# eski dosyayla aynı şemayı (current_to_torque.json - "nm_per_amp" anahtarı,
# 6 elemanlı liste) bekliyoruz.
#
# ANOMALY_V2_CALIB_JSON ortam değişkeniyle geçici olarak başka bir dosya
# gösterilebilir - örn. calibrate_current_to_torque.py'nin ürettiği
# SMOKETEST dosyasıyla "mekanizma çalışıyor mu" diye hızlı deneme yapmak için.
# Böylece "gerçek" yol (Pazartesi kalibrasyonu) ile "deneme" yolu hiç
# karışmaz, ikisi de aynı script'i kullanır.
CURRENT_TO_TORQUE_JSON = Path(os.environ.get(
    "ANOMALY_V2_CALIB_JSON", str(HERE / "data" / "current_to_torque_v2.json")))

# ANOMALY_V2_MAX_RUNS ortam değişkeni ile "yalnız ilk N oturumu işle" denebilir
# - tam koşu 10-30 dakika sürebildiği için, hızlı bir "çalışıyor mu" denemesi
# için 0 (sınırsız) yerine küçük bir sayı (örn. 1) vermek işe yarar.
MAX_RUNS = int(os.environ.get("ANOMALY_V2_MAX_RUNS", "0"))

# Doğrulanmış FMU ters dinamik çözücüsü - v1 paketindeki `resources/`
# dizininden AYNEN kullanılıyor, kopyalanmıyor (README: "Değiştirilmemiştir -
# gerçek robotla doğrulanmış hâlidir").
RESOURCES_DIR = HERE.parent.parent / "anomaly_detection" / "resources"

# 245.pdf ile birebir aynı: T=100 örnek pencere (eğitim script'i kuracak),
# SG türev penceresi 51 örnek (features.py ile AYNI olmalı - farklı olursa
# çevrimiçi/çevrimdışı öznitelikler milimetrik kayar, bu en sinsi hata türü).
SG_WINDOW = 51
SG_POLY = 3
SG_HALF = SG_WINDOW // 2  # 25 - her oturumun İKİ ucundan da bu kadar örnek feda edilir
DT_NOMINAL = 0.002        # 500 Hz varsayımı - online extractor da SABİT dt kullanıyor

# Bir oturumun gerçekten kullanılabilir olması için: qdd hesaplamak üzere
# iki uçtan SG_HALF'er örnek gidiyor, GERİYE KALANIN da en az bir AE
# penceresi (100 örnek) kadar olması lazım.
WINDOW_SIZE = 100
MIN_RUN_LENGTH_STAGE_B = WINDOW_SIZE + 2 * SG_HALF  # 150

# Split (train/val/test) oranları - örnek SAYISINA göre dengelenecek
# (Aşama A'da gördük: 139 oturum çok çarpık uzunlukta, oturum SAYISINA göre
# bölersek tek bir dev oturum tek bir bölüme düşüp o bölümü domine edebilir).
SPLIT_FRACTIONS = {"train": 0.70, "val": 0.15, "test": 0.15}

JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# UR10e resmi DH parametreleri (standart DH) - features.py ile BİREBİR aynı
# tablo. Bunu ayrı bir yerden kopyalamak yerine tek doğruluk kaynağından
# almak daha güvenli olurdu ama v2'nin v1'e Python-import bağımlılığı
# olmasını istemedik (paketler bağımsız kalsın); bu yüzden burada elle
# tekrar yazılıyor - DEĞİŞTİRİLİRSE İKİ YERDE DE değiştirilmeli.
DH_A = np.array([0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0])
DH_D = np.array([0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655])
DH_ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])


def jacobian_and_rotation(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Tek bir poz (q, 6 elemanlı) için taban çerçevesi geometrik Jacobian
    (6x6) ve uç çerçevenin taban çerçevesine göre dönüklüğü (R06, 3x3).
    features.py'deki fonksiyonla birebir aynı - standart DH ileri kinematik.
    """
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


def load_nm_per_amp() -> np.ndarray:
    if not CURRENT_TO_TORQUE_JSON.exists():
        raise FileNotFoundError(
            f"\n\n  {CURRENT_TO_TORQUE_JSON} henüz yok.\n"
            f"  Bu, Pazartesi'den sonra RTDE kaydından (target_moment vs actual_current\n"
            f"  regresyonu) üretilecek. O script hazır olmadan Aşama B çalışamaz -\n"
            f"  bilerek burada duruyoruz, eski/yanlış bir kalibrasyonla sessizce\n"
            f"  devam etmiyoruz.\n")
    d = json.loads(CURRENT_TO_TORQUE_JSON.read_text(encoding="utf-8"))
    return np.asarray(d["nm_per_amp"], dtype=np.float64).reshape(6)


def load_fmu_solver():
    """Doğrulanmış .so çözücüsünü yükler (anomaly_detection/resources).
    fmu_backend.py'deki SoBackend ile aynı mantık - RPC/FMU katmanı atlanıp
    C++ çekirdeğine doğrudan erişiliyor (~1000x daha hızlı, toplu iş için şart)."""
    rd = str(RESOURCES_DIR.resolve())
    if rd not in sys.path:
        sys.path.insert(0, rd)
    try:
        import ur10_solver_py
    except ImportError as e:
        raise RuntimeError(
            f"`ur10_solver_py` import edilemedi: {e}\n"
            f"  arama yolu: {rd}\n"
            f"  Bu derleme Linux x86-64 / Python 3.10 için - WSL2 Ubuntu 22.04 altında "
            f"çalıştırdığından emin ol.") from e
    return ur10_solver_py.InverseDynamicsSolverUR10()


def compute_run_features(g: pd.DataFrame, nm_per_amp: np.ndarray, solver) -> pd.DataFrame | None:
    """Tek bir run_id'ye ait, zaten kronolojik sıradaki satırlar için
    öznitelikleri hesaplar. Run çok kısaysa None döner (dışarıda atlanır).
    """
    n = len(g)
    if n < MIN_RUN_LENGTH_STAGE_B:
        return None

    q = g[[f"q_{i}" for i in range(1, 7)]].to_numpy(dtype=np.float64)
    qd = g[[f"qd_{i}" for i in range(1, 7)]].to_numpy(dtype=np.float64)
    amps = g[[f"effort_amps_{i}" for i in range(1, 7)]].to_numpy(dtype=np.float64)
    wrench = g[["wrench_fx", "wrench_fy", "wrench_fz",
                "wrench_tx", "wrench_ty", "wrench_tz"]].to_numpy(dtype=np.float64)
    t = g["t"].to_numpy(dtype=np.float64)

    # q̈: merkezli Savitzky-Golay türevi, her eklem sütunu için ayrı ayrı.
    # savgol_filter kenarlarda ('interp' modu) polinomla EKSTRAPOLASYON
    # yaparak sahte değer üretir - bunu İSTEMİYORUZ, bu yüzden sonucu
    # hesapladıktan sonra iki uçtan SG_HALF'er örneği KESİYORUZ (aşağıda).
    qdd_full = savgol_filter(qd, window_length=SG_WINDOW, polyorder=SG_POLY,
                              deriv=1, delta=DT_NOMINAL, axis=0, mode="interp")

    # Yalnız gerçekten SG_WINDOW genişliğinde tam komşuluğu olan iç kısmı tut.
    sl = slice(SG_HALF, n - SG_HALF)
    q, qd, amps, wrench, t = q[sl], qd[sl], amps[sl], wrench[sl], t[sl]
    qdd = qdd_full[sl]
    n_inner = len(t)

    tau = amps * nm_per_amp  # [Nm] - Denklem 1'in akım->tork kısmı

    # FMU çağrısı satır satır (çözücü tek bir poz alıyor) - ~1000x hızlı .so
    # yolu olsa da milyonlarca satırda biraz zaman alır, normaldir.
    tau_model = np.empty((n_inner, 6), dtype=np.float64)
    r_ext = np.empty((n_inner, 6), dtype=np.float64)
    for i in range(n_inner):
        tau_model[i] = solver.getTorques(list(q[i]), list(qd[i]), list(qdd[i]))
        J, R06 = jacobian_and_rotation(q[i])
        w = wrench[i].copy()
        w[:3] = R06 @ wrench[i, :3]
        w[3:] = R06 @ wrench[i, 3:]
        r_ext[i] = J.T @ w

    r_top = tau - tau_model              # b (ofset) ve sürtünme HENÜZ YOK
    r_int = r_top - r_ext

    out = pd.DataFrame({
        "run_id": g["run_id"].iloc[sl.start:sl.stop].to_numpy(),
        "t": t,
    })
    for i in range(6):
        out[f"q_{i+1}"] = q[:, i]
        out[f"qd_{i+1}"] = qd[:, i]
        out[f"tau_{i+1}"] = tau[:, i]
    for i, axis in enumerate(["fx", "fy", "fz", "tx", "ty", "tz"]):
        out[f"wrench_{axis}"] = wrench[:, i]
    for i in range(6):
        out[f"r_int_{i+1}"] = r_int[:, i]
        out[f"r_ext_{i+1}"] = r_ext[:, i]
    return out


def assign_splits(run_sample_counts: dict[int, int]) -> dict[int, str]:
    """Her run_id'yi train/val/test'e atar. Amaç: run'ları BÖLMEDEN (bir run
    tamamen tek bir bölüme gider - erken bulduğumuz sızıntı hatasını
    tekrarlamamak için), ama toplam ÖRNEK sayısını hedef oranlara olabildiğince
    yaklaştırmak. Klasik "en büyüğü önce yerleştir, en boş kovaya koy" açgözlü
    algoritması (bin-packing) - kusursuz değil ama basit ve anlaşılır.
    """
    targets = {k: v for k, v in SPLIT_FRACTIONS.items()}
    used = {k: 0 for k in targets}
    total = sum(run_sample_counts.values())
    assignment: dict[int, str] = {}
    # Büyükten küçüğe sırala: en büyük run'lar dengeyi en çok bozan, o yüzden
    # önce onları yerleştirmek daha iyi bir denge verir.
    for run_id, n in sorted(run_sample_counts.items(), key=lambda kv: -kv[1]):
        # Her split için "kullanım oranı / hedef oran" hesapla, en düşük
        # olan (yani hedefinin en altında kalan) split'e ekle.
        def slack(split):
            return (used[split] / total) / targets[split] if targets[split] > 0 else float("inf")
        best = min(targets, key=slack)
        assignment[run_id] = best
        used[best] += n
    return assignment


def main() -> None:
    if not INPUT_CSV.exists():
        raise FileNotFoundError(f"{INPUT_CSV} yok - önce stage_a_prepare.py çalıştırılmalı.")

    nm_per_amp = load_nm_per_amp()   # Pazartesi'ye kadar burada FileNotFoundError ile durur
    solver = load_fmu_solver()

    print(f"Okunuyor: {INPUT_CSV} ...")
    df = pd.read_csv(INPUT_CSV)
    print(f"  {len(df):,} satır, {df['run_id'].nunique()} oturum.")

    feature_frames = []
    n_dropped_runs = 0
    n_processed_runs = 0
    for run_id, g in df.groupby("run_id", sort=False):
        if MAX_RUNS and n_processed_runs >= MAX_RUNS:
            print(f"  ANOMALY_V2_MAX_RUNS={MAX_RUNS} - kalan oturumlar atlanıyor "
                  f"(bu bir hızlı deneme, tam koşu değil).")
            break
        feats = compute_run_features(g.reset_index(drop=True), nm_per_amp, solver)
        n_processed_runs += 1
        if feats is None:
            n_dropped_runs += 1
            continue
        feature_frames.append(feats)

    print(f"  {n_dropped_runs} oturum, Aşama B'nin minimum uzunluğundan "
          f"({MIN_RUN_LENGTH_STAGE_B} örnek) kısa olduğu için atıldı.")

    result = pd.concat(feature_frames, ignore_index=True)

    # split ataması: her run_id'nin toplam örnek sayısına göre.
    run_counts = result.groupby("run_id").size().to_dict()
    split_map = assign_splits(run_counts)
    result["split"] = result["run_id"].map(split_map)

    # MAX_RUNS ile sınırlı bir deneme koşusuysa, çıktıyı "gerçek" dosyanın
    # üzerine YAZMIYORUZ - karışmasın diye ayrı bir isimle kaydediyoruz.
    out_path = (OUTPUT_CSV.with_name(OUTPUT_CSV.stem + "_DEMO.csv")
                if MAX_RUNS else OUTPUT_CSV)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    result.to_csv(out_path, index=False, float_format="%.6f")
    print(f"\nYazıldı: {out_path}  ({len(result):,} satır)")

    for split_name in SPLIT_FRACTIONS:
        n = int((result["split"] == split_name).sum())
        print(f"  {split_name:>5}: {n:,} satır (%{100*n/len(result):.1f})")


if __name__ == "__main__":
    main()
