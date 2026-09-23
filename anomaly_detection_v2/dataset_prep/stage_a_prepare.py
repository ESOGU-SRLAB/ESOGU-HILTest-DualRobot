"""
stage_a_prepare.py
===================
AŞAMA A: ham `ros-joint-states.csv` dosyasını modellerin kullanabileceği
temiz, küçük bir ara dosyaya çevirir.

Bu script'in yaptığı şey, sırayla:
  1. Devasa dosyadan (124 sütun) yalnızca işimize yarayacak ~27 sütunu okur.
  2. Gerçek zaman damgasına (`header.stamp`) göre satırları KRONOLOJİK SIRAYA
     dizer. (Ham dosyanın satır sırası zaman sırası DEĞİL - bunu birlikte
     keşfettik: dosya, aylara yayılmış birçok ayrı kaydın karışık şekilde
     dökülmüş hali. Sıralamadan önce hiçbir "art arda iki örnek" varsayımı
     güvenilir değil.)
  3. Robotun "temiz/normal" çalışmadığı satırları atar (koruyucu dur, arıza,
     acil durdurma - `safety_mode` alanından anlaşılıyor).
  4. Birebir aynı zaman damgasına sahip mükerrer satırları temizler (bunların
     gerçekten birebir aynı veri taşıdığını daha önce ayrıca doğrulamıştık).
  5. Sütunları anlaşılır isimlerle (q_1..q_6, qd_1..qd_6, ...) yeniden adlandırıp
     küçük bir CSV'ye yazar.
  6. En sonda, artık GÜVENİLİR olan (sıralanmış, temizlenmiş) zaman damgaları
     üzerinden gerçek bir "boşluk" (gap) istatistiği basar. Bunu BİRLİKTE
     yorumlayıp bir sonraki adımda (oturum/run_id ayırma) kullanacağımız eşiği
     seçeceğiz - şu an script bu kararı KENDİ BAŞINA vermiyor, bilinçli olarak.

Nasıl çalıştırılır
-------------------
    cd ~/colcon_ws/src/anomaly_detection_v2/dataset_prep
    python3 stage_a_prepare.py

Gereksinim: `pandas` ve `numpy` kurulu olmalı (`pip install pandas numpy`
ya da `sudo apt install python3-pandas python3-numpy`).

Süre/bellek notu: girdi dosyası ~6 GB, 5,9 milyon satır. Yalnızca ihtiyacımız
olan sütunları okuduğumuz için birkaç dakika sürmesi ve birkaç GB RAM
kullanması beklenir - normaldir, kod takılı kalmadı demektir.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

# ─────────────────────────── AYARLAR ───────────────────────────

# Bu script'in bulunduğu dizine göre yol veriyoruz, böylece nereden
# çalıştırırsan çalıştır aynı dosyaları bulur.
HERE = Path(__file__).resolve().parent
INPUT_CSV = HERE.parent.parent / "anomaly_detection" / "ros-joint-states.csv"
OUTPUT_CSV = HERE / "data" / "stage_a_clean.csv"

# UR10e'nin 6 kol ekleminin ADI ve FMU'nun beklediği SIRA (q1..q6).
# Bu sıra keyfi değil: `resources/model.py` içindeki FMU çözücüsü tau1..tau6
# çıktısını tam bu eklem sırasıyla üretiyor (bkz. features.py, DH tablosu da
# aynı sırayla indeksli). Bu sırayı DEĞİŞTİRMEMEK gerekiyor, yoksa ileride
# FMU'ya yanlış eklemin açısı "shoulder_pan" diye gidebilir.
JOINT_NAMES = [
    "shoulder_pan_joint",   # q1 - taban dönüşü (dikey eksen, yerçekimi torku ~0)
    "shoulder_lift_joint",  # q2 - omuz kaldırma (yerçekimi baskın eklem)
    "elbow_joint",          # q3 - dirsek
    "wrist_1_joint",        # q4
    "wrist_2_joint",        # q5
    "wrist_3_joint",        # q6
]
JOINT_PREFIX = "ur10e_"

# UR safety_mode enum değeri: 1 = NORMAL. (3=PROTECTIVE_STOP, 6=SYSTEM_E-STOP,
# 7=ROBOT_E-STOP, 9=FAULT - hepsini attık, yalnızca 1'i tutuyoruz.)
SAFETY_MODE_NORMAL = 1.0

# Oturum (run_id) kırma eşiği. İlk çalıştırmada çıkan gerçek Δt dağılımına
# birlikte baktık: p90 = 2 ms (tamamen normal), ama p90'dan p99'a 2 ms'den
# 258 ms'ye sıçrıyor - yani iki ayrı nüfus var: sıradan küçük paket
# gecikmeleri ve gerçek duraksamalar (görev arası mola, oturum sınırı).
# 1 saniyenin ALTINDAKİ boşlukları "aynı oturumun içi" sayıp run'ı
# kırmıyoruz; yalnızca 1 saniyeyi AŞAN boşlukta yeni run_id başlıyor.
#
# Neden çok daha küçük bir eşik seçmedik: 10 ms eşiğinde ortalama run
# uzunluğu ~26 örneğe düşüyor - bu, AE penceresinden (100 örnek, 0,2 sn)
# bile kısa, yani veri setinin neredeyse tamamı kullanılamaz hale gelirdi.
# Run içinde kalan (1 sn'nin altındaki) tekil boşuklar, Aşama B'de pencere
# bazında ayrıca kontrol edilip kirli pencereler tek tek elenecek - run_id
# burada yalnız kaba/görev düzeyinde bir ayrım.
RUN_BREAK_SECONDS = 1.0

# AE penceresi 100 örnek (245.pdf, T=100). Bir oturum 100 örnekten kısaysa
# tek bir pencere bile üretemez - Aşama B'de zaten kullanılamayacak, o yüzden
# "sadece işimize yarayan veriyi yaz" prensibiyle burada baştan eleniyor.
# (Not: Aşama B'de SG türevi için pencerenin uçlarında ek 25'er örnek daha
# gerekecek, oradaki gerçek eşik biraz daha katı olacak - burası kaba filtre.)
MIN_USABLE_RUN_LENGTH = 100


def main() -> None:
    if not INPUT_CSV.exists():
        raise FileNotFoundError(
            f"Girdi dosyası bulunamadı: {INPUT_CSV}\n"
            f"  Bu script'in `anomaly_detection_v2/dataset_prep/` altında "
            f"olduğunu ve `anomaly_detection/ros-joint-states.csv`'nin onun "
            f"iki üst dizininin yanında durduğunu varsayıyor.")

    # ── 1. Yalnızca ihtiyacımız olan sütunları oku ──────────────────────
    # Ham dosyada 124 sütun var (GPIO bitleri, register'lar, ray, gripper,
    # yazılım versiyonu vb. - hiçbiri anomali tespitine girmiyor). `usecols`
    # ile pandas'a "diskten okurken bile bu sütunlara hiç bakma" diyoruz;
    # bu hem çok daha hızlı hem çok daha az bellek kullanır.
    joint_cols = []
    for j in JOINT_NAMES:
        joint_cols += [f"{JOINT_PREFIX}{j}.position",
                       f"{JOINT_PREFIX}{j}.velocity",
                       f"{JOINT_PREFIX}{j}.effort"]

    wrench_cols = [f"{JOINT_PREFIX}tcp_fts_sensor.{axis}.{comp}"
                   for axis in ("force", "torque") for comp in ("x", "y", "z")]

    use_cols = ["header.stamp.sec", "header.stamp.nanosec",
                f"{JOINT_PREFIX}gpio.safety_mode"] + joint_cols + wrench_cols

    print(f"Okunuyor: {INPUT_CSV} (~6 GB, birkaç dakika sürebilir)...")
    # dtype'ı float32 veriyoruz: sensör verisi için float64'ün fazladan
    # hassasiyeti gereksiz, float32 belleği yarıya indirir. header.stamp.sec
    # tam sayı olduğu için Int64 (büyük harf - NaN'a izin veren pandas tipi,
    # olası eksik satırlarda hata vermez) kullanıyoruz.
    dtype_map = {c: np.float32 for c in (joint_cols + wrench_cols +
                                          [f"{JOINT_PREFIX}gpio.safety_mode"])}
    dtype_map["header.stamp.sec"] = "Int64"
    dtype_map["header.stamp.nanosec"] = "Int64"

    df = pd.read_csv(INPUT_CSV, usecols=use_cols, dtype=dtype_map)
    n_read = len(df)
    print(f"  {n_read:,} satır okundu.")

    # ── 2. Tek bir zaman sütunu üret: t = sec + nanosec/1e9 ─────────────
    # İki ayrı tam sayı sütunu yerine tek bir float saniye değeri, hem daha
    # kolay sıralanır hem de fark almak (t[i] - t[i-1]) çok daha okunaklı.
    df["t"] = df["header.stamp.sec"].astype(np.float64) + \
        df["header.stamp.nanosec"].astype(np.float64) / 1e9
    df = df.drop(columns=["header.stamp.sec", "header.stamp.nanosec"])

    # ── 3. Kronolojik sıraya diz ─────────────────────────────────────────
    # BUNU EN ÖNCE yapıyoruz çünkü aşağıdaki her adım (filtre, dedup, boşluk
    # istatistiği) sıra zaman sırası OLDUĞUNDA anlamlı. `kind="mergesort"`
    # kararlı bir sıralama algoritması - eşit `t` değerine sahip satırların
    # ORİJİNAL göreli sırasını bozmaz (mükerrer temizliğinde "ilkini tut"
    # dediğimizde hangi satırın "ilk" olduğu tutarlı olsun diye).
    df = df.sort_values("t", kind="mergesort").reset_index(drop=True)

    # ── 4. Temiz-olmayan satırları at ────────────────────────────────────
    safety_col = f"{JOINT_PREFIX}gpio.safety_mode"
    mask_normal = df[safety_col] == SAFETY_MODE_NORMAL
    n_dropped_unsafe = int((~mask_normal).sum())
    df = df.loc[mask_normal].drop(columns=[safety_col]).reset_index(drop=True)
    print(f"  {n_dropped_unsafe:,} satır atıldı (safety_mode != NORMAL, "
          f"%{100*n_dropped_unsafe/n_read:.2f}).")

    # ── 5. Mükerrer zaman damgalarını temizle ───────────────────────────
    # Aynı `t` değerine sahip ardışık satırların gerçekten birebir aynı
    # sensör verisini taşıdığını daha önce ayrı bir script'le doğrulamıştık
    # (83.996 mükerrer çiftin TAMAMI birebir aynıydı). Bu yüzden "ilkini
    # tut, gerisini at" güvenli.
    n_before_dedup = len(df)
    df = df.drop_duplicates(subset="t", keep="first").reset_index(drop=True)
    n_dropped_dup = n_before_dedup - len(df)
    print(f"  {n_dropped_dup:,} mükerrer satır atıldı.")

    # ── 6. Sütunları anlaşılır isimlerle yeniden adlandır ────────────────
    rename_map = {}
    for i, j in enumerate(JOINT_NAMES, start=1):
        rename_map[f"{JOINT_PREFIX}{j}.position"] = f"q_{i}"
        rename_map[f"{JOINT_PREFIX}{j}.velocity"] = f"qd_{i}"
        rename_map[f"{JOINT_PREFIX}{j}.effort"] = f"effort_amps_{i}"
    for axis in ("force", "torque"):
        short = "f" if axis == "force" else "t"
        for comp in ("x", "y", "z"):
            rename_map[f"{JOINT_PREFIX}tcp_fts_sensor.{axis}.{comp}"] = f"wrench_{short}{comp}"
    df = df.rename(columns=rename_map)

    # ── 7. Oturum kimliği (run_id) ata ───────────────────────────────────
    # `t` adım 3'te zaten kronolojik sıraya dizildi. İki ardışık örnek
    # arasındaki fark (Δt) RUN_BREAK_SECONDS'ı aşıyorsa, oradan itibaren
    # YENİ bir oturum başlıyor sayıyoruz.
    #   np.diff(t)      -> art arda farklar [t1-t0, t2-t1, ...]  (N-1 eleman)
    #   is_break        -> her farkın eşiği aşıp aşmadığı (True/False)
    #   np.cumsum(...)  -> her True'dan SONRA sayaç bir artar; yani
    #                      is_break=[F,F,T,F,T] -> cumsum=[0,0,1,1,2]
    #   run_id'yi 0'dan başlatmak için başa bir 0 ekliyoruz (ilk örnek
    #   her zaman run_id=0 ile başlar).
    t_arr = df["t"].to_numpy()
    dt = np.diff(t_arr)
    is_break = dt > RUN_BREAK_SECONDS
    run_id = np.concatenate([[0], np.cumsum(is_break)])
    df.insert(0, "run_id", run_id)

    # ── 7b. Çok kısa oturumları (< MIN_USABLE_RUN_LENGTH) at ─────────────
    # run_lengths[k] = run_id'si k olan oturumun kaç örnekten oluştuğu.
    # run_lengths[run_id] ifadesi, her SATIR için "bu satırın ait olduğu
    # oturum toplam kaç örnek" değerini üretir (numpy "fancy indexing").
    run_lengths_pre = np.bincount(run_id)
    n_runs_pre = len(run_lengths_pre)
    n_short_pre = int((run_lengths_pre < MIN_USABLE_RUN_LENGTH).sum())
    keep_mask = run_lengths_pre[run_id] >= MIN_USABLE_RUN_LENGTH
    n_dropped_short = int((~keep_mask).sum())
    df = df.loc[keep_mask].reset_index(drop=True)
    print(f"  {n_dropped_short:,} satır atıldı ({n_short_pre:,} kısa oturum, "
          f"her biri {MIN_USABLE_RUN_LENGTH} örnekten az).")

    # Sütun sırasını da temiz ve öngörülebilir yapalım.
    final_cols = (["run_id", "t"]
                  + [f"q_{i}" for i in range(1, 7)]
                  + [f"qd_{i}" for i in range(1, 7)]
                  + [f"effort_amps_{i}" for i in range(1, 7)]
                  + ["wrench_fx", "wrench_fy", "wrench_fz",
                     "wrench_tx", "wrench_ty", "wrench_tz"])
    df = df[final_cols]

    # ── 8. Yaz ────────────────────────────────────────────────────────────
    OUTPUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    # float_format ile ondalık basamak sayısını sınırlıyoruz (float64'ün
    # tüm gürültülü basamaklarını yazmak yerine) - dosya boyutunu küçültür,
    # sensör hassasiyetinin çok üzerinde bir hassasiyet.
    df.to_csv(OUTPUT_CSV, index=False, float_format="%.6f")
    size_mb = OUTPUT_CSV.stat().st_size / 1e6
    print(f"\nYazıldı: {OUTPUT_CSV}  ({len(df):,} satır, {size_mb:.1f} MB)")

    # ── 9. Tanılama: Δt dağılımı + oturum uzunlukları ────────────────────
    print("\n=== Gerçek Δt dağılımı (sıralanmış + filtrelenmiş veri) ===")
    print(f"  toplam adım sayısı : {len(dt):,}")
    print(f"  medyan Δt          : {np.median(dt)*1000:.3f} ms")
    print(f"  p90 / p99 / p99.9  : {np.percentile(dt,90)*1000:.3f} / "
          f"{np.percentile(dt,99)*1000:.3f} / {np.percentile(dt,99.9)*1000:.3f} ms")
    for thr, label in [(0.004, "4 ms"), (0.010, "10 ms"), (0.050, "50 ms"),
                        (0.100, "100 ms"), (0.25, "0.25 s"), (1.0, "1 s"),
                        (5.0, "5 s"), (60.0, "1 dk"), (300.0, "5 dk")]:
        n_over = int((dt > thr).sum())
        print(f"  Δt > {label:>6}: {n_over:,} kez")

    # run_lengths_pre, filtre uygulanmadan ÖNCEKİ oturum uzunlukları
    # (adım 7b'de zaten hesaplandı) - burada onu özetliyoruz.
    usable_lengths = run_lengths_pre[run_lengths_pre >= MIN_USABLE_RUN_LENGTH]
    print(f"\n=== run_id istatistiği (kırılma eşiği = {RUN_BREAK_SECONDS:.0f} s) ===")
    print(f"  filtre ÖNCESİ oturum sayısı        : {n_runs_pre:,}")
    print(f"  bunlardan {MIN_USABLE_RUN_LENGTH} örnekten kısa (atılan): {n_short_pre:,} "
          f"(%{100*n_short_pre/n_runs_pre:.1f})")
    print(f"  KULLANILABİLİR oturum sayısı       : {len(usable_lengths):,}")
    print(f"  en kısa / medyan / en uzun kullanılabilir oturum : "
          f"{usable_lengths.min():,} / {int(np.median(usable_lengths)):,} / "
          f"{usable_lengths.max():,} örnek")
    print(f"  nihai dosyadaki toplam örnek       : {len(df):,}")


if __name__ == "__main__":
    main()
