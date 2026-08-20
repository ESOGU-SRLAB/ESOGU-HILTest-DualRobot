# anomaly_detection

UR10e işbirlikçi robotu için **çevrimiçi anomali tespiti**: FMU tabanlı kalıntı
ayrıştırma ile ikili LSTM özkodlayıcının skor düzeyinde birleşimi (245.pdf,
Denklem 4). 500 Hz eklem/kuvvet akışını dinler, 20 Hz'de birleşik anomali skoru
üretir.

```
/joint_states (500 Hz) ─┐
                        ├─→ öznitelik motoru ─→ 12 kanal kalıntı ─→ ONNX ─┐
/…/ft_data    (500 Hz) ─┘     (50 ms gecikme)   24 kanal ham     ─→ ONNX ─┤
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
(varsayılan `/force_torque_sensor_broadcaster/ft_data`).

### Arayüz

| Yön | Ad | Tip |
|---|---|---|
| abone | `/joint_states` | `sensor_msgs/JointState` — `effort` alanı **Amper** |
| abone | `/force_torque_sensor_broadcaster/ft_data` | `geometry_msgs/WrenchStamped` (`tool0` çerçevesi) |
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
| `residual_ae_v2/` | Kalıntı özkodlayıcı: 12 kanal, 478.892 parametre, θ = 1,6008 |
| `raw_ae_v2/` | Ham özkodlayıcı: 24 kanal, 1.907.032 parametre, θ = 3,4461 |
| `fusion_v2/fusion_config.json` | `norm = minmax_val`, `w_kal = 0,95`, `θ_birleşik = 0,6436`, affin ölçek |
| `current_to_torque.json` | Akım→tork katsayıları. UR sürücüsü `effort` alanına amper yazdığı için gereklidir. |
| `residual_calibration_clean.json` | Kalıntı ofseti `b` (eklem başına, Nm) |

## Ölçülen başarım

| | Değer |
|---|---|
| Birleşim F1 | **0,792** (yalnız ham 0,613 · yalnız kalıntı 0,579) |
| Tamamlayıcılık | %24,3 yalnızca kalıntı · %28,7 yalnızca ham |
| Çıkarım | karar başına 5,9 ms — 50 ms bütçenin %12'si |
| Tespit gecikmesi | ≈ 120–150 ms (SG 50 ms + karar periyodu 50 ms) |
| Temiz veride yanlış alarm | %0,0 |

## Burada olmayanlar

Eğitim ve değerlendirme hattı (`prepare_dataset.py`, `generate_residuals.py`,
`train_ae.py`, `evaluate_fusion.py`, `tune_adaptive.py`), doğrulama betikleri
(`verify_*.py`, `replay_detector.py`), veri kümeleri (`*.parquet`,
`ros-joint-states.csv`), `UR10e_InverseDynamics.fmu` arşivi, erratum yeniden
üretimi ve teknik rapor çalışma anında gerekmediği için ayrı saklanmaktadır.
Modelleri yeniden üretmek gerekirse tüm hat `run_pipeline.sh` ile tek komutta
çalışır; o dosya da yedektedir.
