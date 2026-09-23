#!/usr/bin/env bash
# run_pipeline_v3.sh
# ==================
# KOŞU-AYRIK hat. run_pipeline.sh bildirinin protokolünü yeniden üretir ve
# erratum için öyle KALIR; bu betik hakem bulgularını kapatan hattır.
#
# Kapatılan bulgular
#   K1  arıza maskesi tüm veri setinde tek bitişik bloktu ve konumu arıza tipini
#       kaydın konumuyla eşliyordu → arızalar artık KOŞU BAŞINA enjekte ediliyor
#   K2  "sızıntısız" iddiası → bölme artık gerçekten koşu-ayrık (make_splits.py)
#   K3  arıza tipi tablosu BestF1'i AUC diye raporluyordu → ikisi ayrı basılıyor
#   K4  birleşim ağırlığı test kümesinde seçiliyordu → artık DOĞRULAMADA seçiliyor
#   K5  tek tohum, belirsizlik yok → SEEDS ile çok tohumlu koşu
#   sürtünme  FMU'da sürtünme modeli yok → terim çözücünün DIŞINDA, kalıntı
#             tanımında; katsayılar yalnız eğitim koşularından
#
# FMU'ya DOKUNULMUYOR (gerçek robotla doğrulandı).
#
# Kullanım:  bash run_pipeline_v3.sh            # tek tohum (0)
#            SEEDS="0 1 2 3 4" bash run_pipeline_v3.sh

set -euo pipefail
cd "$(dirname "$0")"

RES="${AD_PACKAGE:-$HOME/colcon_ws/src/anomaly_detection}/resources"
SEEDS="${SEEDS:-0}"
FEAT=ur10e_features_fric.parquet

echo "### 1/7  Veri hazırlama ###"
python3 prepare_dataset.py --raw ur10e_raw_features.parquet --out ur10e_clean.parquet

echo; echo "### 2/7  Akım→tork kalibrasyonu ###"
python3 calibrate_current_to_torque.py --raw ur10e_clean.parquet \
    --resources "$RES" --out "${AD_PACKAGE:-$HOME/colcon_ws/src/anomaly_detection}/current_to_torque.json"

echo; echo "### 3/7  Nm'ye dönüşüm ###"
python3 convert_effort_to_nm.py --raw ur10e_clean.parquet \
    --calibration "${AD_PACKAGE:-$HOME/colcon_ws/src/anomaly_detection}/current_to_torque.json" --out ur10e_clean_nm.parquet

echo; echo "### 4/7  Koşu-ayrık bölme ###"
# Bölme burada BİR KEZ yapılır; sürtünme uydurması, eğitim ve değerlendirme
# aynı dosyayı okur. Sırası önemli: sürtünme katsayıları YALNIZ eğitim
# koşularından gelmeli, yoksa düzeltmenin kendisi bir sızıntı kanalı olur.
python3 make_splits.py --parquet ur10e_clean.parquet --out splits.json

echo; echo "### 5/7  Kalıntı ayrıştırma + sürtünme (FMU'nun dışında) ###"
python3 generate_residuals.py --raw ur10e_clean_nm.parquet --out "$FEAT" \
    --calib-out residual_calibration_fric.json --resources "$RES" \
    --backend so --units nm --calibrate offset --fts-frame tool \
    --friction fit --splits splits.json --friction-model friction_model.json

echo; echo "### 6/7  Eğitim ve değerlendirme (tohumlar: $SEEDS) ###"
for S in $SEEDS; do
  echo "--- tohum $S ---"
  python3 train_ae.py --mode residual --parquet "$FEAT" --splits splits.json \
      --model-dir "residual_ae_v3_s$S" --seed "$S" --amp --num-workers 0
  python3 train_ae.py --mode raw --parquet "$FEAT" --splits splits.json \
      --model-dir "raw_ae_v3_s$S" --seed "$S" --amp --num-workers 0
  python3 evaluate_v3.py --residual-dir "residual_ae_v3_s$S" --raw-dir "raw_ae_v3_s$S" \
      --parquet "$FEAT" --splits splits.json --seed "$S" \
      --calibration residual_calibration_fric.json --out "fusion_v3_s$S"
done
python3 aggregate_seeds.py --pattern 'fusion_v3_s*' --out fusion_v3_summary.json

echo; echo "### 6b/7  Fiziksel (ölçüm uzayı) enjeksiyon protokolü ###"
# Bildiriden miras protokol arızayı iki temsil uzayına AYRI genliklerle koyuyor;
# kalıntı tarafındaki genlik elle seçilmiş bir sayı. Ölçüldü: çarpışmada o sayı
# fizikle uyumlu (40 Nm ↔ 80,5 Nm tepe) ama sensör gürültüsünde 51 kat küçük
# (0,08 Nm ↔ 4,1 Nm std) — ve tamamlayıcılık iddiası tam da o senaryodan geliyor.
# Bu adım arızayı yalnız ÖLÇÜLEN kanallara koyup kalıntıyı yeniden hesaplar.
python3 make_injected_features.py --split val  --out-dir injected_val
python3 make_injected_features.py --split test --out-dir injected_test
for S in $SEEDS; do
  python3 evaluate_v3.py --residual-dir "residual_ae_v3_s$S" --raw-dir "raw_ae_v3_s$S" \
      --parquet "$FEAT" --splits splits.json --seed "$S" --inject-space measurement \
      --calibration residual_calibration_fric.json --out "fusion_v3m_s$S"
done
python3 aggregate_seeds.py --pattern 'fusion_v3m_s*' --out fusion_v3m_summary.json

echo; echo "### 6c/7  Sürtünme ablasyonu (aynı bölme, aynı tohumlar, sürtünme YOK) ###"
python3 generate_residuals.py --raw ur10e_clean_nm.parquet --out ur10e_features_nofric.parquet \
    --calib-out residual_calibration_nofric.json --resources "$RES" \
    --backend so --units nm --calibrate offset --fts-frame tool --friction none
for S in $SEEDS; do
  python3 train_ae.py --mode residual --parquet ur10e_features_nofric.parquet \
      --splits splits.json --model-dir "residual_ae_nf_s$S" --seed "$S" --amp --num-workers 0
  python3 train_ae.py --mode raw --parquet ur10e_features_nofric.parquet \
      --splits splits.json --model-dir "raw_ae_nf_s$S" --seed "$S" --amp --num-workers 0
  python3 evaluate_v3.py --residual-dir "residual_ae_nf_s$S" --raw-dir "raw_ae_nf_s$S" \
      --parquet ur10e_features_nofric.parquet --splits splits.json --seed "$S" \
      --calibration residual_calibration_nofric.json --out "fusion_nf_s$S"
done
python3 aggregate_seeds.py --pattern 'fusion_nf_s*' --out fusion_nf_summary.json

echo; echo "### 7/7  Doğrulama: çevrimiçi motor ↔ çevrimdışı hat ###"
# Sürtünme terimi kalıntı tanımını değiştirdi; düğümün AYNI ifadeyi uyguladığı
# kayan nokta düzeyinde doğrulanmalı, yoksa fark her karara sabit yanlılık girer.
python3 verify_online_features.py --clean ur10e_clean.parquet --features "$FEAT" \
    --calib "${AD_PACKAGE:-$HOME/colcon_ws/src/anomaly_detection}/current_to_torque.json" --residual-calib residual_calibration_fric.json \
    --friction-model friction_model.json --runs 8

echo
echo "Çevrimiçi düğüm için hazır dosyalar (tohum 0):"
echo "  residual_ae_v3_s0/{model.onnx,metadata.json}"
echo "  raw_ae_v3_s0/{model.onnx,metadata.json}"
echo "  fusion_v3_s0/fusion_config.json"
echo "  ../../current_to_torque.json  residual_calibration_fric.json  friction_model.json"
echo
echo "DİKKAT: düğüme friction_model parametresini vermeyi unutma — modeller"
echo "sürtünme çıkarılmış kalıntıyla eğitildi, düğüm çıkarmazsa fark sabit"
echo "bir yanlılık olarak her karara girer."
