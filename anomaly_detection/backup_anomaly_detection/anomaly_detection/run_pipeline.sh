#!/usr/bin/env bash
# run_pipeline.sh
# ===============
# ANA HAT — birim olarak tutarlı, sızıntısız, çevrimiçi düğümle aynı hesap.
#
# Denetim bulgularının (F1–F4) tamamı burada düzeltilmiş durumda:
#   F1  global zaman sıralaması + kesintisiz 500 Hz koşulara bölme
#   F2  q̈ koşu bazlı, her koşunun kendi dt'siyle; filtre kenarı geçersiz
#   F3  KTS wrench'i tool0'dan tabana döndürülüyor (JᵀF çerçeve tutarlı)
#   F4  akım→tork katsayıları quasi-statik yerçekiminden, döngüsel olmayan biçimde
#
# FMU'ya dokunulmuyor (gerçek robotla doğrulandı). Bunun bilinen bedeli:
# wrist_2 ve wrist_3'te ters dinamik model açıklayıcı güç taşımıyor.
#
# Bildirinin ORİJİNAL sayılarını yeniden üretmek için: bash reproduce_erratum.sh
#
# Kullanım:  bash run_pipeline.sh

set -euo pipefail
cd "$(dirname "$0")"

echo "### 1/8  Veri hazırlama (F1) ###"
python3 prepare_dataset.py --raw ur10e_raw_features.parquet --out ur10e_clean.parquet

echo; echo "### 2/8  Akım→tork kalibrasyonu (F4) ###"
python3 calibrate_current_to_torque.py --raw ur10e_clean.parquet --out current_to_torque.json

echo; echo "### 3/8  Nm'ye dönüşüm ###"
python3 convert_effort_to_nm.py --raw ur10e_clean.parquet \
    --calibration current_to_torque.json --out ur10e_clean_nm.parquet

echo; echo "### 4/8  Kalıntı ayrıştırma (F2, F3) ###"
python3 generate_residuals.py --raw ur10e_clean_nm.parquet --out ur10e_features.parquet \
    --calib-out residual_calibration_clean.json \
    --backend so --units nm --calibrate offset --fts-frame tool

echo; echo "### 5/8  İki modelin eğitimi ###"
python3 train_ae.py --mode residual --parquet ur10e_features.parquet \
    --model-dir residual_ae_v2 --amp --num-workers 0
python3 train_ae.py --mode raw --parquet ur10e_features.parquet \
    --model-dir raw_ae_v2 --amp --num-workers 0

echo; echo "### 6/8  Değerlendirme + birleşim ağırlığının dondurulması ###"
python3 evaluate_fusion.py --residual-dir residual_ae_v2 --raw-dir raw_ae_v2 \
    --calibration residual_calibration_clean.json --out fusion_v2

echo; echo "### 7/8  Uyarlanabilir alarm kuralının ayarı ###"
# Mutlak eşik θ doğrulama setinin EN KÖTÜ uyan koşularından gelir; iyi uyan bir
# koşuda taban çizgisi çok altta oturur ve oradaki gerçek arıza θ'yı hiç geçemez.
# Bu adım mutlak eşiğin yanına gelen medyan+k·MAD kuralının k'sını ölçerek seçer.
python3 tune_adaptive.py

echo; echo "### 8/8  Doğrulama: çevrimiçi motor ↔ çevrimdışı hat ###"
python3 verify_online_features.py --runs 8
python3 replay_detector.py

echo; echo "Çevrimiçi düğüm için hazır dosyalar:"
echo "  residual_ae_v2/{model.onnx,metadata.json}"
echo "  raw_ae_v2/{model.onnx,metadata.json}"
echo "  fusion_v2/fusion_config.json  fusion_v2/adaptive_tuning.json"
echo "  current_to_torque.json  residual_calibration_clean.json"
echo
echo "Çalıştır:  ros2 launch ur10e_anomaly_detection detector.launch.py"
