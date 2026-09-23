# İlk tam koşu — sonuçlar ve bulunan sorun

Ubuntu'daki koşu baştan sona çalıştı: FMU örneklendi, `so` ve `fmu` arka uçları
üretildi, 1.124.432 satır işlendi, iki model eğitildi, birleşim değerlendirmesi
tamamlandı. **Hat çalışıyor.** Ama bir birim hatası çıktı.

---

## Alınan sonuçlar (Tablo II)

| Model | AUC | PR-AUC | BestF1 | bildiri |
|---|---:|---:|---:|---|
| Kalıntı LSTM Özk. | 0,855 | 0,569 | 0,628 | 0,908 / 0,695 / 0,692 |
| Ham LSTM Özk. | **0,943** | 0,735 | 0,661 | 0,952 / 0,761 / 0,698 |
| Bir. A-Ort (0,90/0,10) | 0,955 | 0,784 | **0,811** | — |
| Bir. MAX | **0,974** | 0,862 | 0,803 | 0,976 / 0,889 / 0,824 |
| Bir. OR | 0,974 | 0,862 | 0,803 | 0,976 / 0,889 / 0,824 |
| Art. Norm Eşik | 0,561 | 0,104 | 0,181 | 0,849 / 0,805 / 0,707 |
| Isolation Forest | 0,617 | 0,109 | 0,221 | 0,688 / 0,459 / 0,547 |
| One-Class SVM | 0,785 | 0,565 | 0,617 | 0,815 / 0,816 / 0,760 |

**Yapı doğru yeniden üretti:** birleşim iki tekil modeli de geçiyor, MAX/OR bildiriyle
neredeyse birebir (0,974 vs 0,976), tamamlayıcılık %23,9 / %34,1 (bildiri %24,1 / %25,9),
OR geri çağırma 0,841 (bildiri 0,854). Ham model 0,943 ile bildirinin 0,952'sine çok yakın.

**Kalıntı modeli ise düştü** (0,855 vs 0,908) — sebebi aşağıda.

### Arıza tipine göre (Tablo III)

| Arıza | Kalıntı | Ham | Birleşim | bildiri |
|---|---:|---:|---:|---|
| Motor kayması | **0,239** | 0,582 | 0,390 | 0,597 / 0,605 / 0,588 |
| Çarpışma | 0,999 | 0,997 | 0,999 | 0,916 / 0,933 / 0,920 |
| Gizyazar hatası | 0,982 | 0,492 | 0,981 | 0,986 / 0,530 / 0,977 |
| Sensör gürültüsü | 0,255 | 1,000 | 0,987 | 0,272 / 1,000 / 0,992 |

Çarpışma ve gizyazar bildiriyi **geçiyor**. Motor kayması ise 0,597 → 0,239'a çökmüş.
Bu, hatanın nerede olduğunu gösteren ipucuydu.

---

## Kök neden: `effort` alanı Nm değil, motor akımı

`dynamic_joint_states`'teki `effort`, UR sürücüsünün yazdığı **motor akımı**.
FMU ise **Nm** üretiyor. Hat bu ikisini doğrudan çıkarıyordu — elmayla armut.

Ölçüm (1,1 milyon örnek üzerinde, τ_ölç ile τ_model regresyonu):

| eklem | korel | eğim a | R² | yorum |
|---|---:|---:|---:|---|
| shoulder_pan | 0,050 | 0,0087 | 0,002 | düşey eksen → yerçekimi torku yok, beklenen |
| **shoulder_lift** | **0,959** | **0,0923** | **0,918** | model varyansın %92'sini açıklıyor |
| **elbow** | **0,948** | **0,1099** | **0,899** | %90 |
| wrist_1 | 0,418 | 0,1347 | 0,175 | zayıf yük |
| wrist_2 | 0,060 | 0,1615 | 0,003 | sürtünme/gürültü baskın |
| wrist_3 | −0,009 | −0,1016 | 0,000 | aynı |

**Senin FMU modelin doğru çalışıyor** — yerçekimi yükünü taşıyan iki eklemde ölçümle
korelasyonu 0,95–0,96. Eğimlerin eklem başına farklı olması (1/10,8 ve 1/9,1) bunu
doğruluyor: global bir birim hatası olsa tek katsayı çıkardı, bunlar UR'ın eklem başına
farklı redüktör/tork sabitleri.

### Etkisi

Ölçeklenmemiş çıkarma, artığı küçülteceğine **büyüttü**:

| r_int std | bozuk parquet (τ_model=0) | kalibresiz | **kalibre** |
|---|---:|---:|---:|
| shoulder_lift | 5,28 | 34,61 | **4,28** |
| elbow | 2,98 | 14,51 | **2,66** |
| ortalama | 2,27 | 9,45 | **2,05** |

Sabit genlikli arıza enjeksiyonları bu yüzden görece zayıfladı:

```
motor kayması (25 Nm, r_int_3):
  bozuk parquet : 8,4 σ
  kalibresiz    : 1,7 σ   ← F1 0,239'un sebebi
  kalibre       : 9,4 σ
```

---

## Düzeltme

`generate_residuals.py`'ye eklem başına ölçek kalibrasyonu eklendi:

```
τ_ölç ≈ a·τ_model + b        →        r_top = τ_ölç − (a·τ_model + b)
```

* `a` eklemin etkin tork sabitini/redüktör oranını, `b` akım ofsetini soğurur.
* Yalnızca ilk %80'de (eğitim böleni) uydurulur — sızıntı yok.
* 3σ dışındaki noktalar 3 tur iteratif atılır ki anomaliler katsayıyı bozmasın
  (inlier oranı %98,7–100 çıkıyor).
* Katsayılar `residual_calibration.json`'a yazılır; **ROS 2 node'u aynı katsayıları
  kullanacak.**
* `--calibrate none` ile eski davranış, `--calibrate scale` ile ofsetsiz sürüm.

Modele dokunulmadı; yapılan tek şey ölçüm ile model arasındaki birim uyumu.

---

## Yeniden çalıştırılacaklar (Ubuntu)

```bash
cd ~/siu2026
source ~/venv310/bin/activate     # python3 zaten 3.10 ise atla

python3 generate_residuals.py --backend so --out ur10e_hybrid_residual.parquet
python3 verify_fmu_residual.py --residual ur10e_hybrid_residual.parquet --raw ur10e_raw_features.parquet
deactivate

python3 train_ae.py --mode residual --parquet ur10e_hybrid_residual.parquet \
        --model-dir residual_ae_model --amp --num-workers 0

python3 evaluate_fusion.py --residual-dir residual_ae_model --raw-dir raw_ae_model --out fusion_results
```

Ham modeli **yeniden eğitmeye gerek yok** — o kalibrasyondan etkilenmiyor
(girdisi q, q̇, τ, KTS; τ_model'e dokunmuyor). Sadece kalıntı modeli yeniden eğitilecek,
~4 dakika.

### Beklentiler

* `generate_residuals` çıktısındaki kalibrasyon tablosunda shoulder_lift ve elbow
  için R² ≈ 0,92 / 0,90 ve "kazanç" sütununda ~35× / ~24× görmelisin.
* Kalıntı modelinin AUC'si 0,855'ten yukarı çıkmalı.
* `motor_kaymasi` satırı 0,239'dan toparlamalı.
* Birleşim BestF1'i 0,811'in üzerine çıkmalı.

---

## Sıradaki iz: KTS ofseti

Kalibrasyondan sonra `r_int`'e **`r_ext` hâkim** oluyor:

```
shoulder_lift:  r_top (kalibre) std ≈ 1,0     r_ext std ≈ 4,07     r_int std ≈ 4,28
```

Yani `r_int = r_top − r_ext` işlemi, "içsel arıza" kanalına KTS gürültüsünü **enjekte
ediyor**. Temas yokken `r_ext` ≈ 0 olmalıydı.

Daha önce ölçmüştüm: kuvvet/tork sensöründe tüm segmentlerde tutarlı **~5,4 N sabit
ofset** var (yük yalnızca ~0,30 kg, yani ofset gerçek sinyalden büyük). Sensör tare
edilmemiş.

Bunu düzeltmek `r_int`'i belirgin temizler ama ölçüm zincirine müdahale demek, o yüzden
kendi başıma yapmadım. İstersen `generate_residuals.py`'ye `--tare-fts` seçeneği
ekleyeyim: temassız dönemlerin ortalamasını KTS'den çıkarır, Jacobian aktarımı ondan
sonra yapılır. Canlıda karşılığı `/io_and_status_controller/zero_ftsensor` servisini
node başlangıcında çağırmak olur.
