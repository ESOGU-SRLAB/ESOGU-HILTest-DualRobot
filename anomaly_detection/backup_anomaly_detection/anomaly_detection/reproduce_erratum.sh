#!/usr/bin/env bash
# reproduce_erratum.sh
# ====================
# 245.pdf'in ORİJİNAL hattını birebir yeniden kurar — bildirinin Tablo II ve III
# sayılarının nereden geldiğini belgelemek için.
#
# Bildirinin hattı birim olarak KARIŞIKTI:
#     τ_ölç   : amper   (UR sürücüsü effort alanına actual_current yazıyor)
#     τ_model : Nm      (FMU ters dinamik)
#     r_dis   : Nm      (JᵀF, üstelik wrench tool0 çerçevesindeyken taban Jacobian'ıyla)
#     r_top   = τ_amper − τ_model_Nm        ← üç farklı büyüklük doğrudan çıkarılmış
#     arıza genlikleri: "Nm" diye yazılan sayılar amper kanallara ham eklenmiş
#
# Bu koşu o düzeni tekrar kurar. Beklenen: bildirinin imza bulguları geri gelir —
# "kalıntı model sensör gürültüsüne kör" (0,272) ve "anomalilerin ~%24'ü yalnızca
# kalıntı tarafından yakalanıyor" (%24,1).
#
# ANA SONUÇ BU DEĞİLDİR. Fiziksel olarak tutarlı hat için: run_pipeline.sh
#
# Kullanım:  bash reproduce_erratum.sh [çıktı_dizini]

set -euo pipefail
OUT="${1:-erratum}"
mkdir -p "$OUT"
cd "$(dirname "$0")"

echo "=============================================================="
echo "ERRATUM — 245.pdf'in orijinal karışık birimli hattı"
echo "  çıktı: $OUT/"
echo "=============================================================="

# 1) Kalıntılar: kalibrasyon YOK, model sıfırlaması YOK, wrench çerçevesi düzeltilmemiş,
#    birim amper (yani τ_model Nm'den doğrudan çıkarılıyor — bildirinin yaptığı).
python3 generate_residuals.py \
    --raw ur10e_raw_features.parquet \
    --out "$OUT/residual.parquet" \
    --calib-out "$OUT/calibration.json" \
    --backend so \
    --calibrate none --min-r2=-1e9 --units current \
    --fts-frame base

# 2) İki modeli de bu veriyle sıfırdan eğit.
python3 train_ae.py --mode residual --parquet "$OUT/residual.parquet" \
    --model-dir "$OUT/residual_ae" --amp --num-workers 0 --no-onnx
python3 train_ae.py --mode raw --parquet ur10e_raw_features.parquet \
    --model-dir "$OUT/raw_ae" --amp --num-workers 0 --no-onnx

# 3) Değerlendir. calibration.json'da residual_units="current" ve a=1 olduğu için
#    arıza genlikleri ham sayı olarak uygulanır — bildirinin yaptığı.
python3 evaluate_fusion.py \
    --residual-dir "$OUT/residual_ae" --raw-dir "$OUT/raw_ae" \
    --calibration "$OUT/calibration.json" --out "$OUT/fusion"

echo
echo "Beklenen eşleşmeler (bildiri → bu koşu):"
echo "  kalıntı en iyi dönem        129 → ~127"
echo "  yalnızca-kalıntı payı     %24,1 → ~%23,9"
echo "  sensör gürültüsü kalıntı  0,272 → ~0,255"
echo "  gizyazar kalıntı/ham 0,986/0,530 → ~0,982/0,492"
echo "  OR geri çağırma           0,854 → ~0,841"
