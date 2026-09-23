# 245.pdf'in birebir yeniden üretimi — çalıştırma kılavuzu

Bildiri: **"FMU Tabanlı Kalıntı Ayrıştırma ve İkili LSTM Özkodlayıcı Birleşimi ile
İşbirlikçi Robotlarda Anomali Tespiti"** (SIU 2026, IEEE 979-8-3195-1046-4/26)

> ⚠️ `AnomalyDetection_v1/siu2026_paper/main.tex` **başka bir çalışma** (tek model,
> AUC 0.988 / F1 0.840). `245.pdf` ondan sonra gelen, **iki modelin skor düzeyinde
> birleşimi** üzerine olan sürüm. Bu kılavuz `245.pdf`'i esas alır.

Tüm scriptler `AnomalyDetection_Live/` altında, **`AnomalyDetection_v1/` içinden**
çalıştırılır.

---

## Bildirideki iki model

| | **Kalıntı** (residual) | **Ham** (raw) |
|---|---|---|
| Girdi | 12 kanal: r_ic,1–6 + r_dis,1–6 | 24 kanal: q, q̇, τ, KTS (her biri 6) |
| Pencere / adım | 100 / 25 (%75 örtüşme) | 100 / 25 |
| Gizli / saklı | 128 / 32 | 256 / 64 |
| **Parametre** | **478.892** | **1.907.032** |
| Bildiride yakınsama | 129. dönem, val_loss 0,181 | 87. dönem, val_loss 0,320 |
| Eşik (P97) | θ = 0,420 | θ = 0,854 |
| AUC / PR-AUC / BestF1 | 0,908 / 0,695 / 0,692 | 0,952 / 0,761 / 0,698 |

**Ortak eğitim yapılandırması:** Adam (lr=1e-3, β₁=0,9, β₂=0,999) · yığın 256 ·
sönümleme %15 · gradyan kırpma max_norm=1,0 · ReduceLROnPlateau (faktör 0,5, sabır 8) ·
erken durdurma sabır 25, maksimum 300 dönem · MSE kaybı · eşik = doğrulama yeniden
yapılanma hatalarının **97. persentili**.

**Birleşim:** min–max normalleştirilmiş skorların ağırlıklı ortalaması
`S_bir = w_kal·S̄_kal + w_ham·S̄_ham`, w_kal ∈ [0,1] adım 0,05.
Bildiride en iyi: w_kal = 0,95 → AUC 0,980 · PR-AUC 0,905 · BestF1 0,859.

### Doğruladığım şeyler

Kodu yazmadan önce bildirinin kurulumunu tersine çözüp sayısal olarak teyit ettim:

```
parametre sayısı   residual  D=12 H=128 Z=32 →   478.892   bildiri   478.892  ✅
                   raw       D=24 H=256 Z=64 → 1.907.032   bildiri 1.907.032  ✅

pencere (N=1.124.432, W=100, S=25)          =    44.974
  4 senaryo × 44.974                         =   179.896   bildiri   179.896  ✅
  eğitim   (%80 örnek → pencere)             =    35.978   bildiri    35.978  ✅
  doğrulama                                  =     8.992   bildiri     8.992  ✅
  arıza penceresi (4 senaryo, %10 örtüşme)   =    14.402   bildiri    14.402  ✅
```

Yani mimari, bölme kuralı, pencereleme ve arıza maskeleri bildiriyle **birebir**.

---

## Gerekli paketler

```bash
pip install torch --index-url https://download.pytorch.org/whl/cu124   # RTX 4000 Ada
pip install pandas pyarrow numpy scipy scikit-learn matplotlib onnx
```

---

## 0) Mimariyi doğrula (5 saniye)

```bash
python ..\AnomalyDetection_Live\models.py
```

İki modelin parametre sayısını hesaplayıp bildirinin değerleriyle karşılaştırır.
İkisi de ✅ değilse devam etme.

---

## 1) Ham 24 kanallı özellikleri çıkar

```bash
cd AnomalyDetection_v1

python ..\AnomalyDetection_Live\extract_raw_features.py --max-chunks 3 --out raw_smoke.parquet
python ..\AnomalyDetection_Live\extract_raw_features.py --out ur10e_raw_features.parquet
```

Parquet 27 kolon içerir (q, q̇, τ, KTS + TCP konum); model bunlardan **24'ünü**
kullanır — TCP konumu bildirinin ham tanımında yok, ileride lazım olur diye
dosyada duruyor.

Chunk boyutu ve chunk-içi zaman sıralaması `generate_hybrid_residual.py` ile birebir
aynı → iki parquet satır-satır hizalı, `t` kolonları çakışıyor. **Birleşim bunu
gerektiriyor**: iki modelin skorları aynı pencerelere denk gelmeli.

Regexleri 50.000 gerçek satırda doğruladım: 0 hata.

---

## 2) İki modeli eğit

```bash
python ..\AnomalyDetection_Live\train_ae.py --mode residual ^
       --parquet ur10e_hybrid_residual.parquet --model-dir residual_ae_model --amp

python ..\AnomalyDetection_Live\train_ae.py --mode raw ^
       --parquet ur10e_raw_features.parquet --model-dir raw_ae_model --amp
```

FMU'yu yeniden koşturmaya gerek yok — `ur10e_hybrid_residual.parquet` (207 MB) elde.
Mevcut `lstm_ae_model/` klasörüne dokunulmuyor.

Eğitim sırasında parametre sayısı, pencere sayıları ve son değerler **bildirinin
değerleriyle yan yana** yazdırılır; sapma anında görülür.

Her dizine çıkanlar: `best_model.pt` · `model.onnx` · `metadata.json`
(mean, std, threshold, feature_cols, pencere/adım, loss+lr geçmişi) ·
`training_loss.png` · `val_window_errors.npy`

---

## 3) Birleşim değerlendirmesi — bildirinin asıl sonucu

```bash
python ..\AnomalyDetection_Live\evaluate_fusion.py ^
       --residual-dir residual_ae_model --raw-dir raw_ae_model --out fusion_results
```

Bu tek komut bildirinin **tüm** sonuç bölümünü üretir:

* Dört sentetik arıza senaryosunu iki temsil uzayına kendi genlikleriyle enjekte eder
* 4 × 44.974 = 179.896 test penceresini iki modelle skorlar
* **Tablo II** — genel performans (AUC / PR-AUC / BestF1), bildirinin değerleri yanında
* **Tablo III** — arıza tipine göre BestF1 (kalıntı / ham / birleşim)
* **Ağırlık taraması** — w_kal ∈ [0, 1] adım 0,05, en iyi noktayı bulur
* **MAX** ve **OR** birleşim stratejileri
* **Referans yöntemler** — Kalıntı Norm Eşiği, Isolation Forest (kontaminasyon 0,08),
  One-Class SVM (RBF, ν=0,05)
* **Tamamlayıcılık analizi** — arıza pencerelerinin yüzde kaçı yalnızca kalıntı /
  yalnızca ham model tarafından yakalanıyor (bildiri: %24,1 ve %25,9)

Çıktılar: `fusion_results.json` · `table2_overall.csv` · `table3_per_fault.csv` ·
`fig2_roc.png` · `fig3_complementarity.png` · `fig3_temporal.png` ·
`fig4_weight_sweep.png` · `scores.npz`

Hızlı duman testi: `--limit-rows 200000` ekle.

### Dört arıza senaryosu (bildiri II.D)

| senaryo | tip | aralık | ham genlik | kalıntı genlik | hedef |
|---|---|---|---|---|---|
| Motor kayması | doğrusal rampa | %30–38 | 15 Nm | 25 Nm | Eklem 3 (dirsek) torku |
| Çarpışma | Gauss darbe | merkez %65 ± %4 | 30 N | 40 N | tüm KTS kanalları |
| Gizyazar hatası | basamak | %92'den itibaren | 1,5 rad | 8 Nm | Eklem 5 (bilek 2) pozisyonu |
| Sensör gürültüsü | Gauss gürültü | %15–23 | 3,5 N | 0,08 Nm | tüm KTS kanalları |

Sensör gürültüsü kasıtlı olarak yalnızca ham sinyali belirgin etkileyecek genlikte —
tamamlayıcılığı sınamak için. Pencere etiketi: örneklerin ≥ %10'u arıza maskesiyle
örtüşüyorsa anomali.

`python ..\AnomalyDetection_Live\inject_faults.py` ile senaryoların pencere
sayılarını tek başına doğrulayabilirsin (14.402 çıkmalı).

---

## 4) Ek teşhis (bildiride yok, ROS 2 için gerekli)

```bash
python ..\AnomalyDetection_Live\evaluate_ae.py --mode residual --model-dir residual_ae_model
python ..\AnomalyDetection_Live\evaluate_ae.py --mode raw      --model-dir raw_ae_model
python ..\AnomalyDetection_Live\compare_models.py --raw raw_ae_model --hybrid residual_ae_model
```

Bunlar bildirinin protokolü **değil** — varyans tabanlı hızlı bir bakış artı canlı
sistem için gereken üç ölçüm:

* **Çıkarım gecikmesi** — batch 1/8/64 için ms/çağrı ve pencere/s, sonra "500 Hz
  akışta stride=25 ile saniyede 20 pencere gerekiyor, kapasite payın K×". ROS 2
  node'unun stride'ını buna göre seçeceğiz.
* **CUSUM katmanı** — pencere-tabanlı AE'nin göremediği yavaş sürüklenme için.
* **Anomali sınıflandırma** — `‖r_int‖ > 2‖r_ext‖ → internal` vb.

---

## Sonraki adım — ROS 2 Humble

`dynamic_joint_states` tek topic olduğu için mimari sade:

```
/dynamic_joint_states  (control_msgs/DynamicJointState, 500 Hz)
        │  tek mesajda: 6 eklem × (position, velocity, effort)
        │               + ur10e_tcp_fts_sensor (force.*/torque.*)
        ▼
   parse (isim→indeks önbelleği, tek seferlik)
        ├── ham dalı     : 24 kanal → pencere 100 → normalize → AE(256/64) → S_ham
        └── kalıntı dalı : savgol(51,3) tamponu → q̈
                           FMU/solver → τ_model → r_top
                           J(q)ᵀ·KTS → r_dis ,  r_ic = r_top − r_dis
                           12 kanal → pencere 100 → normalize → AE(128/32) → S_kal
        ▼
   min–max normalizasyon → S_bir = 0,95·S̄_kal + 0,05·S̄_ham
        ▼
/ur10e/anomaly  (S_kal, S_ham, S_bir, eşikler, is_anomaly, tip, kanal katkıları)
```

İki tasarım kararı:

1. **Merkezli savgol korunuyor.** Offline hat q̈'yı `savgol_filter(qd, 51, 3, deriv=1)`
   ile alıyor; bu 25 örnek (**50 ms**) ileriye bakıyor. Canlıda 51 örneklik tampon
   tutup artığı pencerenin **ortası** için yayınlayacağız: 50 ms sabit gecikme
   karşılığında offline hattın birebir aynısı, train/serve farkı sıfır.
2. **Skor pencere seviyesinde.** Eğitim eşiği pencere-başına MSE dağılımından
   geliyor; canlı node da pencere-başına MSE üretecek, eşik doğrudan geçerli.

Ters dinamik için iki arka uç: `resources/ur10_solver_py...so`'yu doğrudan import
(ROS 2 Humble = Ubuntu 22.04 = Python 3.10, `.so` tam uyumlu) veya FMU üzerinden.
Doğrudan import gRPC katmanını atladığı için 500 Hz'de tek uygulanabilir seçenek.

**Min–max normalizasyon canlıda dikkat ister:** offline'da tüm test kümesi üzerinden
hesaplanıyor, canlıda geleceği bilemeyiz. Node, eğitim/doğrulama skorlarından
sabitlenmiş min–max katsayılarını `metadata.json`'dan kullanacak — bunu füzyon
değerlendirmesi sırasında kaydediyorum.
