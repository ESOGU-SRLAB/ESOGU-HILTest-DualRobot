"""
inspect_runs.py
================
Aşama A'nın çıktısındaki (`stage_a_clean.csv`) 139 kullanılabilir oturumun
(`run_id`) her birinin ne zaman kaydedildiğini ve robotun o oturumda ne kadar
hareketli olduğunu özetler. Amaç: bu 139 parçanın hangi tarihlere/geçmiş
çalışmalara denk geldiğini kabaca görmek - eğitim/doğrulama/test bölünmesini
Aşama B'de yaparken hangi oturumun "büyük/önemli" olduğunu bilerek karar
vermek için.

Hiçbir dosyayı DEĞİŞTİRMEZ, yalnız okuyup ekrana bir tablo basar.

Çalıştırma:
    cd ~/colcon_ws/src/anomaly_detection_v2/dataset_prep
    python3 inspect_runs.py
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
INPUT_CSV = HERE / "data" / "stage_a_clean.csv"


def main() -> None:
    if not INPUT_CSV.exists():
        raise FileNotFoundError(
            f"{INPUT_CSV} yok - önce stage_a_prepare.py'yi çalıştırmış olman lazım.")

    # Bu analiz için q ve effort/wrench'e gerek yok, yalnızca run_id, t ve
    # qd (hız) sütunlarını okuyoruz - dosya zaten küçüldü ama yine de gereksiz
    # sütunu okumamak daha hızlı.
    qd_cols = [f"qd_{i}" for i in range(1, 7)]
    df = pd.read_csv(INPUT_CSV, usecols=["run_id", "t"] + qd_cols)

    # Her satır için "o anki tepe eklem hızı" = 6 eklemin |qd|'sinin en büyüğü.
    # Robotun o anda gerçekten hareket edip etmediğinin kaba bir göstergesi.
    peak_speed = df[qd_cols].abs().max(axis=1)

    # groupby("run_id"): aynı run_id'ye sahip satırları tek grupta toplar.
    # .agg(...) her grup için istediğimiz özet istatistikleri hesaplar.
    df["_peak_speed"] = peak_speed
    g = df.groupby("run_id").agg(
        n_samples=("t", "size"),
        t_start=("t", "min"),
        t_end=("t", "max"),
        mean_peak_speed=("_peak_speed", "mean"),
    ).reset_index()

    g["duration_s"] = g["t_end"] - g["t_start"]
    # Kronolojik sırada göstermek daha anlaşılır.
    g = g.sort_values("t_start").reset_index(drop=True)

    def fmt_dt(ts: float) -> str:
        # header.stamp epoch saniye - yerel saat dilimine çevirip okunaklı basıyoruz.
        return datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")

    def fmt_dur(s: float) -> str:
        if s < 60:
            return f"{s:.1f} sn"
        if s < 3600:
            return f"{s/60:.1f} dk"
        return f"{s/3600:.2f} sa"

    print(f"{'run_id':>7} {'başlangıç':>19} {'süre':>9} {'örnek':>10} "
          f"{'ort. tepe hız (rad/s)':>22}")
    print("-" * 72)
    for _, row in g.iterrows():
        print(f"{int(row.run_id):>7} {fmt_dt(row.t_start):>19} "
              f"{fmt_dur(row.duration_s):>9} {int(row.n_samples):>10,} "
              f"{row.mean_peak_speed:>22.4f}")

    print("\n=== özet ===")
    print(f"  toplam oturum       : {len(g)}")
    print(f"  toplam örnek        : {g['n_samples'].sum():,}")
    print(f"  ilk kayıt           : {fmt_dt(g['t_start'].min())}")
    print(f"  son kayıt           : {fmt_dt(g['t_end'].max())}")
    print(f"  en büyük 5 oturum (örnek sayısına göre):")
    top5 = g.sort_values("n_samples", ascending=False).head(5)
    for _, row in top5.iterrows():
        pct = 100 * row.n_samples / g["n_samples"].sum()
        print(f"    run_id={int(row.run_id):<5} {fmt_dt(row.t_start)}  "
              f"{int(row.n_samples):,} örnek  (%{pct:.1f})")


if __name__ == "__main__":
    main()
