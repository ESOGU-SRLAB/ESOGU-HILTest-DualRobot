# anomaly_detection (v4)

UR10e işbirlikçi robotu için **çevrimiçi anomali tespiti**: KTS'siz kalıntı
LSTM özkodlayıcı ile ham LSTM özkodlayıcının skor düzeyinde birleşimi. 500 Hz
eklem akışını dinler, 20 Hz'de birleşik anomali skoru üretir.

```
/joint_states (500 Hz) ─→ öznitelik motoru ─→ 6 kanal kalıntı (r_tot)  ─→ ONNX ─┐
                              (50 ms gecikme)   16 kanal ham (q₂₋₅,q̇,τ) ─→ ONNX ─┤
                                                                                 ▼
                            w_kal·z_kal + w_ham·z_ham  →  rejim eşiği  →  ~/detected
```

## v3 → v4: altı düzeltme

`anomaly_detection_v2/dataset_prep/`de ölçülerek karara bağlandı (ayrıntı ve
ölçüm script'leri orada):

1. **Veri.** `training_features.csv` — Aşama A→C'den geçmiş (kronolojik
   sıralama, `safety_mode` filtresi, 10 Hz'lik eski kayıtların atılması,
   senaryo etiketleri, tur-ayrık split). Eski hattın verisi bu adımlardan
   geçmemişti.
2. **KTS çıkarıldı.** `ur10e_tcp_fts_sensor` fiziksel sensör değil — UR
   sürücüsü onu RTDE `actual_TCP_force`'tan dolduruyor (kontrolcünün AYARLI
   payload'a göre tahmini). Payload ayarı gerçek takımla uyuşmayınca durağan
   pozda bile onlarca N sahte sapma üretiyordu. Kalıntı artık tek kanal:
   `r_tot = τ_ölç − τ_model − düzeltme` (12→6 kanal); ham modelden wrench
   tamamen çıktı (24→18).
3. **Akım→tork RTDE ile ölçüldü.** `current_to_torque.json`: J2/J3 RTDE
   `target_moment` regresyonuyla ölçüldü (eski quasi-statik yöntemle aynı
   sonuç, bağımsız doğrulama), J1/J4-J6 aile katsayısı (`trusted: false`).
4. **Sürtünme + ofset + yük SENARYO başına.** Tek global sabit ofset,
   görevler arası takım ağırlığı farkını (20+ Nm) genelleyemiyordu. Yeni
   model: `Fc·tanh(q̇/ε) + Fv·q̇ + b[senaryo] + m[senaryo]·A(q) + u[senaryo]·B(q)`
   — hem train hem val/test'te kalıntıyı küçültüyor (`residual_calibration.json`).
5. **Ham model q₁/q₆'yı görmüyor.** İkisi de yerçekimi torkuna girmiyor
   (dikey eksen / sınırsız dönen bilek); tutmak, ham modelin hiç görmediği
   bir poza giden turu baştan sona "anomali" saymasına yol açıyordu.
6. **Birleşim ağırlığı ÖLÇÜLDÜ.** `w_kal=0,90` (`fusion_v4/fusion_config.json`,
   `w_source`), önsel/bildiriden alınmış değil — `evaluate_faults.py`,
   fiziksel-tutarlı arıza enjeksiyonu, 3 normalizasyon × 3 ham-kanal
   varyantı tarandı.

v3 (KTS'li, gerçek hücrede kalibre edilmiş eski nesil) **silinmedi**,
`backup_anomaly_detection/anomaly_detection_v3_real_cell/` altında duruyor.

## Kurulum ve çalıştırma

```bash
colcon build --packages-select anomaly_detection --symlink-install
source install/setup.bash

# use_case ZORUNLU niyetinde - yük/ofset düzeltmesi buna göre seçilir.
ros2 launch anomaly_detection detector.launch.py use_case:=HRC
# diğer seçenekler: MULTIROBOT_INSPECTION | PICKPLACE | UR10E_INSPECTION
```

Robotsuz denemek için (`fault`: `yok|motor_kaymasi|carpisma|gizyazar|sensor_gurultu`):

```bash
ros2 run anomaly_detection replay_publisher --ros-args -p use_case:=HRC -p fault:=carpisma
```

Dikkat edilecek parametreler: `tf_prefix` (`/joint_states` içindeki eklem
adlarıyla eşleşmeli, varsayılan `ur10e_`), `use_case`, `quantile`
(`p97|p99|p99.9|p99.99` — eşik bunlardan seçilir, varsayılan `p99.9`).

### Arayüz

| Yön | Ad | Tip |
|---|---|---|
| abone | `/joint_states` | `sensor_msgs/JointState` — `effort` alanı **Amper** |
| yayın | `~/score` | `std_msgs/Float32` — birleşik skor |
| yayın | `~/detected` | `std_msgs/Bool` — gecikme kuralı uygulanmış alarm |
| yayın | `~/detail` | `std_msgs/Float32MultiArray` — 17 eleman |

`~/detail` sırası: `s_kal, s_ham, z_kal, z_ham, birleşik, mutlak eşik,
uyarlanabilir eşik, mutlak isabet, uyarlanabilir isabet, kalıntı isabet, ham
isabet, hareketli, q̇ tepe, taban_n, donmuş, kalıntı eşiği, ham eşiği`.

Son iki alan (`kalıntı eşiği`, `ham eşiği` — her modelin kendi metadata.json
eşiği, `quantile` parametresinden bağımsız) v4'te EKLENDİ: arayüz eskiden
bunları hard-code ediyordu (v3'ün Nm ölçeğindeki sayılarıyla), tıpkı bir
zamanlar birleşik eşiğin de hard-code edilip yanlış çizilmesi gibi — artık
`~/detail`'den canlı okunuyor.

## Yapı

| Yol | İşlev |
|---|---|
| `anomaly_detection/features.py` | Çevrimiçi öznitelik motoru: SG türevi, FMU, senaryo-bazlı sürtünme+yük düzeltmesi — KTS YOK |
| `anomaly_detection/detector.py` | ROS'tan bağımsız çekirdek: iki ONNX özkodlayıcı + log-normalize birleşim + rejim eşiği |
| `anomaly_detection/detector_node.py` | ROS 2 sarmalayıcısı (ince) |
| `anomaly_detection/replay_publisher.py` | `training_features.csv`'yi 500 Hz'de yayınlayan test yayıncısı (senaryo + fiziksel-tutarlı arıza seçilebilir) |
| `anomaly_detection/replay_scores.py` | Kaydedilmiş bir oturumu arayüze geri yayınlar (robot/dedektör gerekmez) |
| `launch/detector.launch.py` | Düğümü başlatır |

### Çalışma zamanı varlıkları

`share/anomaly_detection` altına kurulur; düğüm önce oradan okur, yoksa
kaynak ağacına düşer.

| Yol | İşlev |
|---|---|
| `resources/` | FMU ters dinamik çekirdeği (`ur10_solver_py…so`). **Değiştirilmemiştir** — gerçek robotla doğrulanmış hâlidir. |
| `residual_ae_v4/` | Kalıntı özkodlayıcı: 6 kanal, val kaybı 0,0744, θ(P97)=0,565 |
| `raw_ae_v4/` | Ham özkodlayıcı: 16 kanal (q₂₋₅,q̇₁₋₆,τ₁₋₆), val kaybı 0,0798, θ(P97)=0,739 |
| `fusion_v4/fusion_config.json` | Birleşim ağırlığı, log-normalizasyon parametreleri, rejim×persentil eşik tablosu. **`PROVISIONAL: true`** — offline val'den, henüz gerçek robotta ölçülmedi. |
| `current_to_torque.json` | Akım→tork katsayıları (RTDE ile ölçüldü) |
| `residual_calibration.json` | Sürtünme (Fc,Fv) + senaryo başına ofset (b) + yük fiziği (m, u) — TEK dosya (eskiden iki ayrı dosyaydı) |

## Ölçülen başarım (OFFLINE — henüz gerçek robot değil)

Test kümesi (tur-ayrık split), fiziksel-tutarlı arıza enjeksiyonu (arıza
gerçek sinyale eklenir, modele özel ayrı genlik yok — `evaluate_faults.py`):

| Arıza | AUC @1× şiddet (kalıntı / ham / birleşim) |
|---|---|
| Çarpışma (tork darbesi) | 0,999 / 0,962 / 0,999 |
| Ölçüm gürültüsü | 0,991 / 0,821 / 0,990 |
| Motor kayması | 0,838 / 0,606 / 0,820 |
| Enkoder adımı | 0,517 / 0,549 / 0,523 |

Enkoder hariç hepsinde kalıntı baskın (fizik: 0,3 rad'lık bir konum hatası
J5'te yerçekimi torkunu yalnızca ~0,1 Nm değiştiriyor — kalıntının gürültü
tabanının altında). Ortalama (tüm arıza×şiddet): AUC 0,825 / PR-AUC 0,506 /
BestF1 0,611 (kalıntı) — birleşim marjinal olarak yakın (0,824/0,504/0,609)
ama **uçtan uca canlı testte iki modelin birlikte tetiklediği alarm
gözlendi** (kalıntı+ham), yani birleşim salt kâğıt üstünde değil.

Rejim-koşullu eşik, kalıntı model, şiddet 1×:

| Persentil | Yanlış alarm/saat | Çarpışma | Gürültü | Motor kayması | Enkoder |
|---|---:|---:|---:|---:|---:|
| P97 | 2.010 | 1,00 | 0,83 | 0,50 | 0,03 |
| P99 | 459 | 1,00 | 0,49 | 0,26 | 0,01 |
| P99,9 | 100 | 1,00 | 0,48 | 0,00 | 0,00 |

**Eşik çevrimdışından taşınmıyor** — v3'ün merkezî bulgusu burada da geçerli
(21.08.2026: 0,6436 → 18,0; 26.08.2026: kararların %58'i alarm). Bu yüzden
`fusion_config.json` açıkça `PROVISIONAL` işaretli: robotta çalıştırmadan önce
gerçek hücreden yeniden kalibre edilmesi (v3'teki `calibrate_cell.py`'nin
eşdeğeri, henüz v4'e taşınmadı) **şart**.

## Burada olmayanlar

Eğitim/değerlendirme hattı (`prepare_dataset.py`→`evaluate_faults.py`),
veri kümeleri ve ham kayıtlar `anomaly_detection_v2/dataset_prep/` altında —
kardeş pakette, bu paketin çalışma zamanı ona bağlı **değildir**.

v3 artefaktları (KTS'li, gerçek hücrede kalibre edilmiş) ve eski
kalibrasyon dosyaları `backup_anomaly_detection/anomaly_detection_v3_real_cell/`
altında arşivli.
