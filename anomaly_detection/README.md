# anomaly_detection

UR10e işbirlikçi robotu için **çevrimiçi anomali tespiti**: FMU tabanlı kalıntı
ayrıştırma ile ikili LSTM özkodlayıcının skor düzeyinde birleşimi (245.pdf,
Denklem 4). 500 Hz eklem/kuvvet akışını dinler, 20 Hz'de birleşik anomali skoru
üretir.

```
/joint_states (500 Hz) ─┐
                        ├─→ öznitelik motoru ─→ 12 kanal kalıntı ─→ ONNX ─┐
/…/wrench     (500 Hz) ─┘     (50 ms gecikme)   24 kanal ham     ─→ ONNX ─┤
                                                                          ▼
                          w_kal·z_kal + w_ham·z_ham  →  eşik  →  ~/detected
```

## Kurulum ve çalıştırma

```bash
colcon build --packages-select anomaly_detection --symlink-install
source install/setup.bash

ros2 launch anomaly_detection detector.launch.py

# bildirinin saf davranışına dönmek için (eklentileri kapatır):
ros2 launch anomaly_detection detector.launch.py \
     adaptive:=false consecutive_for_alarm:=1
```

Dikkat edilecek iki parametre: `tf_prefix` (`/joint_states` içindeki eklem
adlarıyla eşleşmelidir; varsayılan `ur10e_`) ve `wrench_topic`
(varsayılan `/force_torque_sensor_broadcaster/wrench`).

### Arayüz

| Yön | Ad | Tip |
|---|---|---|
| abone | `/joint_states` | `sensor_msgs/JointState` — `effort` alanı **Amper** |
| abone | `/force_torque_sensor_broadcaster/wrench` | `geometry_msgs/WrenchStamped` (`tool0` çerçevesi) |
| yayın | `~/score` | `std_msgs/Float32` — birleşik skor |
| yayın | `~/detected` | `std_msgs/Bool` — gecikme kuralı uygulanmış alarm |
| yayın | `~/detail` | `std_msgs/Float32MultiArray` — 11 eleman |

`~/detail` sırası: `s_kal, s_ham, z_kal, z_ham, birleşik, mutlak eşik,
uyarlanabilir eşik, mutlak isabet, uyarlanabilir isabet, kalıntı isabet, ham isabet`.

## Yapı

| Yol | İşlev |
|---|---|
| `anomaly_detection/features.py` | Çevrimiçi öznitelik motoru: SG türevi, ters dinamik, `Jᵀ·F`, kalıntı ayrıştırma |
| `anomaly_detection/detector.py` | ROS'tan bağımsız çekirdek: iki ONNX özkodlayıcı + birleşim + eşik |
| `anomaly_detection/detector_node.py` | ROS 2 sarmalayıcısı (ince) |
| `anomaly_detection/replay_publisher.py` | Parquet'i 500 Hz'de yayınlayan test yayıncısı |
| `launch/detector.launch.py` | Düğümü başlatır |

### Çalışma zamanı varlıkları

Bunlar `share/anomaly_detection` altına kurulur; düğüm önce oradan okur, yoksa
kaynak ağacına düşer.

| Yol | İşlev |
|---|---|
| `resources/` | FMU ters dinamik çekirdeği (`ur10_solver_py…so`). **Değiştirilmemiştir** — gerçek robotla doğrulanmış hâlidir. |
| `residual_ae_v3/` | Kalıntı özkodlayıcı: 12 kanal, 478.892 parametre, θ = 1,3723 |
| `raw_ae_v3/` | Ham özkodlayıcı: 24 kanal, 1.907.032 parametre, θ = 0,3934 |
| `fusion_v3/fusion_config_cell.json` | **Dağıtım varsayılanı.** Sınırlar ve θ = 5,0 gerçek hücrenin 15.627 temiz kararından ölçüldü (`--scale p97`). |
| `fusion_v3/fusion_config.json` | Çevrimdışı türetilmiş karşılığı; makalenin çevrimdışı tablosuyla aynı, **sahada kullanılmaz**. |
| `fusion_v3/KOKEN.md` | Bu artefaktların hangi hattan çıktığı, tohum seçimi |
| `current_to_torque.json` | Akım→tork katsayıları. UR sürücüsü `effort` alanına amper yazdığı için gereklidir. |
| `residual_calibration_fric.json` | Kalıntı ofseti `b` (eklem başına, Nm) |
| `friction_model.json` | Coulomb/viskoz katsayıları. **Kurulmazsa düğüm başlamaz** — modeller sürtünme çıkarılmış kalıntıyla eğitildi (`detector.py` uyuşmazlıkta hata verir). |

v2 (bildiri) artefaktları 28.08.2026'da paketten çıkarıldı; yedeğin altında
durur ve `reproduce_erratum.sh` ile çevrimdışı yeniden üretilir.

## Ölçülen başarım

Koşu-ayrık test kümesi, tohum 1 (dağıtılan tohum): 95 koşu · 6.016 pencere ·
553 arızalı. Tohumlar arası dağılım için `docs/ESOGU_MMF_Makale.pdf`.

| | Değer |
|---|---|
| Birleşim ROC-AUC | **0,963** (yalnız ham 0,871 · yalnız kalıntı 0,869) |
| Birleşim PR-AUC | **0,845** (yalnız ham 0,666 · yalnız kalıntı 0,645) |
| Çalışma noktası (rejim-koşullu P97) | K 0,701 · G 0,898 · F1 **0,788** |
| Tamamlayıcılık | %22,8 yalnızca kalıntı · %28,0 yalnızca ham · %20,8 hiçbiri |
| Çıkarım | karar başına 5,9 ms — 50 ms bütçenin %12'si |
| Tespit gecikmesi | ≈ 120–150 ms (SG 50 ms + karar periyodu 50 ms) |

**Eşik çevrimdışından taşınmıyor** — bu çalışmanın merkezî bulgusu.
21.08.2026'da v2 için ölçüldü (0,6436 → 18,0), 26.08.2026'da v3 için
tekrarlandı (0,8553 ile kararların %58'i alarm). Dağıtım varsayılanı bu yüzden
`fusion_config_cell.json`'dır: bildirinin min–max formülü aynen korunur, yalnız
sınırların ve θ'nın ölçüldüğü küme gerçek hücredir (`calibrate_cell.py`).

## Burada olmayanlar

Eğitim ve değerlendirme hattı (`prepare_dataset.py`, `generate_residuals.py`,
`make_splits.py`, `train_ae.py`, `evaluate_v3.py`, `calibrate_cell.py`),
doğrulama betikleri (`verify_*.py`, `replay_detector.py`), veri kümeleri
(`*.parquet`, `ros-joint-states.csv`), `UR10e_InverseDynamics.fmu` arşivi,
v2 artefaktları, erratum yeniden üretimi ve teknik rapor çalışma anında
gerekmediği için **paketin dışındaki** `backup_anomaly_detection/` altında
durur (28.08.2026 itibarıyla `~/Desktop/backup_anomaly_detection`).

Paketin çalışma zamanı yedeğe bağlı **değildir** — dedektör ona hiç bakmaz.
Bağlı olanlar yalnız `docs/` içindeki makale üreteçleridir; yedeği kendileri
arar, taşınırsa `AD_BACKUP` ile gösterilir:

```bash
AD_BACKUP=~/Desktop/backup_anomaly_detection python3 docs/makale_uret.py
```

Hattı yeniden çalıştırmak için:

```bash
cd ~/Desktop/backup_anomaly_detection/anomaly_detection
SEEDS="0 1 2 3 4" bash run_pipeline_v3.sh     # güncel hat
bash reproduce_erratum.sh                     # bildirinin karışık birimli hattı
```

Hat betikleri pakete **mutlak** yolla bakar (`AD_PACKAGE` ile değiştirilebilir),
bu yüzden yedeğin nerede durduğu onları etkilemez.

28.08.2026'da yer kazanmak için silinen ara ürünler (`injected_*/`,
`ur10e_features_nofric.parquet`, `erratum/residual.parquet`, v1/v2 parquet'leri)
bu iki betik tarafından yeniden üretilir. Ham kayıt `ros-joint-states.csv` ve
v3 eğitim girdisi `ur10e_features_fric.parquet` korunmaktadır.
