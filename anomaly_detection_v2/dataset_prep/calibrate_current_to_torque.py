"""
calibrate_current_to_torque.py
===============================
`actual_current` (RTDE, ölçülen akım) ile `target_moment` (RTDE, kontrolcünün
kendi hesapladığı hedef tork) arasında eklem başına doğrusal bir ilişki
kurar: actual_current[j] ≈ a[j] * target_moment[j] + c[j]

Buradan nm_per_amp[j] = 1/a[j] çıkar - `current_to_torque_v2.json`'a yazılan
asıl sayı bu (Aşama B'nin effort_amps'i torka çevirmek için kullandığı
katsayı).

ÖNEMLİ - BU ÇALIŞTIRMA BİR SMOKE TEST:
Şu an elimizdeki `ur-rtde-data.csv` yalnızca 78 örnek, ~0,6 saniyelik bir
kayıt. Gerçek bir kalibrasyon için (özellikle bilek eklemlerinde yeterli
tork çeşitliliği görmek için) Pazartesi'den sonra toplanacak, dakikalarca
süren ve robotun çeşitli hız/pozlarda hareket ettiği bir kayıt gerekiyor.
Bu script'in ÇIKTISI gerçek kalibrasyon olarak KULLANILMAYACAK - yalnızca
kodun/regresyonun/JSON şemasının doğru çalıştığını görmek için.

Bu yüzden çıktı, Aşama B'nin varsayılan olarak aradığı
`current_to_torque_v2.json` yerine `current_to_torque_v2_SMOKETEST.json`
adıyla yazılıyor - yanlışlıkla "gerçek" kalibrasyon sanılmasın diye.

Çalıştırma:
    cd ~/colcon_ws/src/anomaly_detection_v2/dataset_prep
    python3 calibrate_current_to_torque.py
"""

from __future__ import annotations

import ast
import csv
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
INPUT_CSV = HERE.parent.parent / "anomaly_detection" / "ur-rtde-data.csv"
OUTPUT_JSON = HERE / "data" / "current_to_torque_v2_SMOKETEST.json"

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow",
               "wrist_1", "wrist_2", "wrist_3"]

# R² bu değerin altındaysa "trusted: false" - eski current_to_torque.json'daki
# 0.3 eşiği fazla gevşekti (neredeyse gürültüyü bile geçiriyordu), burada
# daha sıkı bir çıta kullanıyoruz. 78 örneklik bu smoke test'te muhtemelen
# HİÇBİR eklem bu çıtayı geçmeyecek - beklenen, sorun değil.
MIN_R2_TRUSTED = 0.7


def main() -> None:
    if not INPUT_CSV.exists():
        raise FileNotFoundError(f"{INPUT_CSV} yok.")

    actual_current = []
    target_moment = []
    with open(INPUT_CSV, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            # `data` sütunu bir Python dict'inin string hâli (RTDE client'ının
            # ham çıktısı) - ast.literal_eval yalnızca literal veri yapılarını
            # (dict/list/sayı/string) çözer, eval() gibi keyfi kod ÇALIŞTIRMAZ,
            # bu yüzden güvenli.
            d = ast.literal_eval(row["data"])
            actual_current.append(d["actual_current"])
            target_moment.append(d["target_moment"])

    actual_current = np.asarray(actual_current, dtype=np.float64)  # (N, 6)
    target_moment = np.asarray(target_moment, dtype=np.float64)    # (N, 6)
    n = len(actual_current)
    print(f"{n} örnek okundu (bu bir SMOKE TEST - gerçek kalibrasyon için çok az).")

    nm_per_amp = np.zeros(6)
    per_joint = []
    for j in range(6):
        x = target_moment[:, j]
        y = actual_current[:, j]
        # np.polyfit(x, y, 1): y ≈ a*x + c en küçük kareler ile - a=eğim, c=kesişim.
        a, c = np.polyfit(x, y, 1)
        y_pred = a * x + c
        ss_res = np.sum((y - y_pred) ** 2)
        ss_tot = np.sum((y - y.mean()) ** 2)
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
        trusted = bool(r2 >= MIN_R2_TRUSTED and abs(a) > 1e-9)
        nm = float(1.0 / a) if abs(a) > 1e-9 else float("nan")
        nm_per_amp[j] = nm
        per_joint.append({
            "joint": JOINT_NAMES[j], "a_amp_per_nm": float(a), "c_amp": float(c),
            "r2": float(r2), "nm_per_amp": nm, "trusted": trusted,
        })
        print(f"  {JOINT_NAMES[j]:<14} a={a:+.5f} A/Nm  c={c:+.3f} A  "
              f"R²={r2:.3f}  nm_per_amp={nm:+.3f}  trusted={trusted}")

    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(json.dumps({
        "SMOKE_TEST_ONLY": True,
        "warning": "78 örnek / ~0.6 saniye - GERÇEK kalibrasyon için kullanılamaz. "
                   "Yalnızca kod/regresyon/şema doğrulaması içindir.",
        "source_csv": str(INPUT_CSV),
        "n_samples": n,
        "nm_per_amp": nm_per_amp.tolist(),
        "per_joint": per_joint,
    }, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nYazıldı: {OUTPUT_JSON}")
    print("\nBEKLENEN: R² değerleri muhtemelen düşük/tutarsız çıktı - 0,6 saniyede "
          "hem çok az örnek var hem de robot büyük ihtimalle geniş bir tork "
          "aralığı görmedi. Bu, Pazartesi'nin daha uzun/çeşitli kaydıyla düzelecek.")


if __name__ == "__main__":
    main()
