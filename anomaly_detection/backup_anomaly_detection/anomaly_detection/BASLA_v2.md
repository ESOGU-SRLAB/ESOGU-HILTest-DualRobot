# Ubuntu — güncellenmiş akış (v2)

Bu paket, ilk koşudan sonra bulunan **birim hatasını** düzeltiyor. Tüm scriptler
tazelendi; `~/siu2026` içine açıp üzerine yaz.

---

## Ne değişti, neden

İlk koşu baştan sona çalıştı ama iki katmanlı bir birim sorunu vardı.

**1. `effort` alanı Nm değil, motor akımı.** UR'ın kendi sürücü belgesi:
*"The effort field contains the currents reported by the joints and not the actual
efforts in a physical sense."* Fiziksel sağlaması da net: UR10e kolu yatayken
shoulder_lift'te ~121 Nm gerekiyor, effort'un tepesi ise ~10 — Nm olsaydı robot
kolunu kaldıramazdı. Ölçülen ölçek farkı 10,8× (korelasyon 0,959).

**2. `r_ext = J(q)ᵀ·F_KTS` zaten Nm.** Yani `r_int = r_top − r_ext` çıkarması da
karışık birimdeydi.

Sonuç: `r_int` küçüleceğine büyümüştü ve motor kayması enjeksiyonu 8,4σ'dan 1,7σ'ya
düşmüştü — Tablo III'te `motor_kaymasi` F1'inin 0,239'a çökmesinin sebebi buydu.

**Düzeltme:** her şey Nm'ye çekiliyor. `convert_effort_to_nm.py` ölçüm kanallarını
akımdan Nm'ye çeviriyor, sonra hattın tamamı tek birimde çalışıyor.

Bu **doğruluğu tek başına değiştirmiyor** (AE kanal başına z-skor normalize ediyor,
sabit çarpan soğuruluyor) ama artık ayrıştırması ve arıza genlikleri ancak tek birimde
anlamlı — asıl kazanç orada.

---

## Ön koşul

`~/siu2026` içinde şunların durduğunu varsayıyorum (ilk koşudan kalanlar):

```
ur10e_raw_features.parquet       resources/        binaries/
UR10e_InverseDynamics.fmu        modelDescription.xml      ur10e_jacobian.py
```

`ros-joint-states.csv`'ye artık ihtiyaç yok — ham özellikler zaten çıkarılmıştı.
Yoksa önce `python3 extract_raw_features.py --out ur10e_raw_features.parquet`.

---

## Akış

```bash
cd ~/siu2026
```

### 1 — effort → Nm  (~30 sn)

```bash
python3 convert_effort_to_nm.py --out ur10e_raw_features_nm.parquet
```

Kaynak CSV'ye ve mevcut parquet'e dokunmaz, yeni dosya yazar. Katsayılar script
içinde gömülü (senin verinden ölçüldü), ayrıca çıktıda fiziksel sağlama var:
shoulder_lift tepe torku 60–250 Nm aralığında olmalı (beklenen ~121 Nm).

### 2 — artıklar, Nm uzayında  (~5–10 dk)

```bash
source ~/venv310/bin/activate          # python3 zaten 3.10 ise atla

python3 generate_residuals.py --raw ur10e_raw_features_nm.parquet --units nm \
        --out ur10e_hybrid_residual.parquet --backend so

python3 verify_fmu_residual.py --residual ur10e_hybrid_residual.parquet \
        --raw ur10e_raw_features_nm.parquet

deactivate
```

> **🔑 Bakacağın satır:** kalibrasyon tablosunda **shoulder_lift ve elbow için
> `a ≈ 1,0000`**. Dönüşüm doğruysa bu böyle çıkar — kendi kendini doğrulayan kontrol.
> Sapıyorsa dur, devam etme.

Düşük R²'li üç eklemde (shoulder_pan, wrist_2, wrist_3) model hiçbir şey açıklamıyor;
script onları otomatik sıfırlayıp uyarı basacak. Beklenen davranış.

### 3 — iki modeli eğit  (~50 dk)

```bash
python3 models.py      # parametre kontrolü: 478.892 ve 1.907.032 → ikisi de ✅

python3 train_ae.py --mode residual --parquet ur10e_hybrid_residual.parquet \
        --model-dir residual_ae_model --amp --num-workers 0

python3 train_ae.py --mode raw --parquet ur10e_raw_features_nm.parquet \
        --model-dir raw_ae_model --amp --num-workers 0
```

**Ham modeli de yeniden eğitmek zorunlu** — girdi ölçeği değişti, eski
`metadata.json`'daki mean/std amper cinsindendi. Sonuçları pratikte aynı çıkacak
ama tutarlılık şart.

### 4 — birleşim değerlendirmesi  (~10 dk)

```bash
python3 evaluate_fusion.py --residual-dir residual_ae_model --raw-dir raw_ae_model \
        --out fusion_results
```

Başlarken `residual_units = "nm"` gördüğü için arıza genliklerini olduğu gibi
uygulayacak — bildirinin 25 Nm / 8 Nm / 40 N değerleri doğrudan geçerli.

### 5 — ek teşhis (isteğe bağlı, ROS 2 için gerekli)

```bash
python3 evaluate_ae.py --mode residual --model-dir residual_ae_model
python3 evaluate_ae.py --mode raw      --model-dir raw_ae_model
python3 compare_models.py --raw raw_ae_model --hybrid residual_ae_model
```

Çıkarım gecikmesi, CUSUM ve anomali sınıflandırma buradan geliyor.

---

## Tek blokta

```bash
cd ~/siu2026
python3 convert_effort_to_nm.py --out ur10e_raw_features_nm.parquet

source ~/venv310/bin/activate
python3 generate_residuals.py --raw ur10e_raw_features_nm.parquet --units nm --out ur10e_hybrid_residual.parquet --backend so
python3 verify_fmu_residual.py --residual ur10e_hybrid_residual.parquet --raw ur10e_raw_features_nm.parquet
deactivate

python3 train_ae.py --mode residual --parquet ur10e_hybrid_residual.parquet --model-dir residual_ae_model --amp --num-workers 0
python3 train_ae.py --mode raw --parquet ur10e_raw_features_nm.parquet --model-dir raw_ae_model --amp --num-workers 0
python3 evaluate_fusion.py --residual-dir residual_ae_model --raw-dir raw_ae_model --out fusion_results
```

Toplam ~1 saat 10 dakika.

---

## İlk koşuda ne almıştık (karşılaştırma için)

| Model | AUC | PR-AUC | BestF1 |
|---|---:|---:|---:|
| Kalıntı LSTM Özk. | 0,855 | 0,569 | 0,628 |
| Ham LSTM Özk. | 0,943 | 0,735 | 0,661 |
| Bir. A-Ort (0,90/0,10) | 0,955 | 0,784 | 0,811 |
| Bir. MAX | 0,974 | 0,862 | 0,803 |

Arıza tipine göre: motor kayması **0,239** · çarpışma 0,999 · gizyazar 0,982 ·
sensör gürültüsü 0,255 (kalıntı sütunu).

Bu koşuda kalıntı modelinin ve özellikle `motor_kaymasi` satırının toparlamasını
bekliyorum. Ham model ve MAX birleşimi zaten bildiriye yakındı, onlar fazla
değişmemeli.

---

## Bu pakette ne var

| dosya | durum |
|---|---|
| `convert_effort_to_nm.py` | **YENİ** — akım → Nm dönüşümü |
| `generate_residuals.py` | değişti — kalibrasyon, `--units`, `--min-r2`, A_model/A_unit ayrımı |
| `inject_faults.py` | değişti — arıza genliklerinde birim ölçekleme |
| `evaluate_fusion.py` | değişti — kalibrasyon bağlandı, Tablo III global normalizasyon, float64 pencere öznitelikleri |
| `evaluate_ae.py` | değişti — `--mode residual` desteği |
| `verify_fmu_residual.py` | değişti — 0/0 açığı kapatıldı (her şey sıfırken yanlışlıkla ✅ veriyordu) |
| `train_ae.py` · `models.py` · `extract_raw_features.py` · `fmu_backend.py` · `check_fmu.py` · `compare_models.py` | değişmedi, tazelik için dahil |
| `belgeler/SONUCLAR_v1.md` | ilk koşunun sonuçları + kök neden analizi |
| `belgeler/FMU_DOGRULAMA_NOTU.md` | FMU doğrulaması neden ölçek farkını yakalayamadı |
| `belgeler/UYUMLULUK_DENETIMI.md` | 245.pdf ile neyin kanıtlandığı / neyin çıkarım olduğu |
