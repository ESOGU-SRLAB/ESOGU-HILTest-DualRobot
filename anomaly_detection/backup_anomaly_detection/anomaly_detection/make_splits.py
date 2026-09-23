#!/usr/bin/env python3
"""
make_splits.py
==============
KOŞU-AYRIK eğitim/doğrulama/test bölmesi.

Neden ayrı bir adım
-------------------
Eski hat bölmeyi satır indeksinden yapıyordu (`int(N * 0.8)`), test kümesini ise
TÜM pencerelerden kuruyordu. Sonuç: 41.688 değerlendirme penceresinin 33.176'sı
(%79,6) modelin eğitimde temiz hâlini gördüğü pencerelerdi. Ölçtük — temiz
pencerelerde yeniden yapılanma hatası eğitim bölgesinde 0,007, dışında 0,486;
yani 70 kat. Bu bölmeyle ölçülen her genelleme sayısı şişkindir.

Bölme burada BİR KEZ yapılır ve üç tüketici de aynı dosyayı okur:
  · generate_residuals.py  → sürtünme katsayıları YALNIZ eğitim koşularından
  · train_ae.py            → eğitim/doğrulama koşuları
  · evaluate_fusion.py     → arızalar YALNIZ test koşularına enjekte edilir,
                             lo/span ve P97 YALNIZ doğrulama koşularından

Uzunluk çarpıklığı
------------------
600 koşunun 521'i 500 örnekten (1 s) kısa; en uzun 32 koşu verinin %54'ünü,
tek başına en uzunu %10,5'ini taşıyor. Rastgele koşu ataması bu yüzden çalışmaz:
bir kez zar atınca test kümesi ya %5 ya %25 veri alır. Bunun yerine koşular
uzundan kısaya sıralanıp her adımda ORANSAL açığı en büyük olan bölmeye verilir
(largest-first bin packing). Sonuç deterministiktir ve örnek payları hedefe
yakınsar.

Kullanım
--------
    python3 make_splits.py --parquet ur10e_clean.parquet --out splits.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd


def _greedy(items: list[int], order_by: dict[int, int], weight: dict[int, float],
            target: dict[str, float], cur: dict[str, float],
            out: dict[str, list[int]]) -> None:
    """
    `order_by` değerine göre büyükten küçüğe gez; her adımda ORANSAL açığı en
    büyük bölmeye ata ve o bölmenin sayacını `weight` kadar artır.

    Sıralama ölçütü ile kota ölçütü ayrı tutulur: arıza taşıyıcıları uzunluğa
    göre sıralanır ama kotaları SAYIyla tutulur (her bölme kendi bağımsız arıza
    olaylarını alsın), kısa koşular ise örnek sayısıyla kotalanır.

    Oransal açık şart: mutlak açıkla ilk birkaç dev koşunun tamamı eğitime
    yığılır (eğitim hedefi en büyük olduğu için açığı da hep en büyüktür) ve
    test kümesi yalnız kısa koşulardan oluşur.
    """
    for r in sorted(items, key=lambda x: (-order_by[x], x)):
        k = max(target, key=lambda s_: ((target[s_] - cur[s_]) / max(target[s_], 1e-9),
                                        -cur[s_], s_))
        out[k].append(r)
        cur[k] += weight[r]


def assign(lengths: dict[int, int], ratios: dict[str, float],
           min_fault_len: int) -> dict[str, list[int]]:
    """
    İKİ AŞAMALI koşu-ayrık dağıtım.

    Tek aşamalı (yalnız örnek sayısına bakan) dağıtım bu veri setinde çalışmıyor:
    600 koşunun 521'i 1 saniyeden kısa, en uzun 32 koşu ise verinin %54'ünü
    taşıyor. Örnek kotasına göre paketlenince doğrulama kümesine tek bir dev koşu
    + 71 kırıntı düşüyor ve arıza enjekte edilebilecek koşu sayısı 1'de kalıyor —
    ağırlık taraması o kümede yapılamaz.

    Bu yüzden önce ARIZA TAŞIYABİLEN koşular (>= min_fault_len) SAYIca oranlanır;
    her bölme kendi bağımsız arıza olaylarını alır. Kalan kısa koşular sonra
    ÖRNEK kotasını doldurur. Sonuç deterministiktir.
    """
    hosts = [r for r, L in lengths.items() if L >= min_fault_len]
    rest = [r for r in lengths if r not in set(hosts)]
    out: dict[str, list[int]] = {k: [] for k in ratios}

    # 1) arıza taşıyıcıları: uzunluğa göre sırala, kotayı SAYIyla tut
    _greedy(hosts, lengths, {r: 1.0 for r in hosts},
            {k: len(hosts) * v for k, v in ratios.items()},
            {k: 0.0 for k in ratios}, out)

    # 2) kısa koşular: kalan ÖRNEK kotasını doldur
    total = sum(lengths.values())
    cur = {k: float(sum(lengths[r] for r in out[k])) for k in ratios}
    _greedy(rest, lengths, {r: float(lengths[r]) for r in rest},
            {k: total * v for k, v in ratios.items()}, cur, out)

    return {k: sorted(v) for k, v in out.items()}


def main() -> int:
    ap = argparse.ArgumentParser(description="Koşu-ayrık veri bölmesi")
    ap.add_argument("--parquet", default="ur10e_clean.parquet")
    ap.add_argument("--out", default="splits.json")
    ap.add_argument("--train", type=float, default=0.70)
    ap.add_argument("--val", type=float, default=0.15)
    ap.add_argument("--test", type=float, default=0.15)
    ap.add_argument("--window", type=int, default=100,
                    help="pencere boyu — koşu başına pencere tahmini için")
    ap.add_argument("--stride", type=int, default=25)
    ap.add_argument("--min-fault-len", type=int, default=400,
                    help="arıza enjekte edilebilmesi için gereken en kısa koşu")
    args = ap.parse_args()

    ratios = {"train": args.train, "val": args.val, "test": args.test}
    s = sum(ratios.values())
    if abs(s - 1.0) > 1e-6:
        print(f"HATA: oranlar 1.0 etmiyor ({s:.3f})", file=sys.stderr)
        return 2

    df = pd.read_parquet(args.parquet, columns=["run_id"])
    runs = df["run_id"].to_numpy()
    uniq, counts = np.unique(runs, return_counts=True)
    lengths = {int(r): int(c) for r, c in zip(uniq, counts)}
    n = len(runs)

    print("=" * 74)
    print("KOŞU-AYRIK BÖLME")
    print("=" * 74)
    print(f"  kaynak    : {args.parquet}")
    print(f"  {len(lengths):,} koşu · {n:,} örnek ({n*0.002:.0f} s robot zamanı)")
    q = np.percentile(counts, [50, 90, 99])
    print(f"  koşu uzunluğu: medyan {q[0]:.0f} · p90 {q[1]:.0f} · p99 {q[2]:.0f} · "
          f"maks {counts.max():,}")
    print(f"  en uzun koşu tek başına verinin %{100*counts.max()/n:.1f}'i; "
          f"en uzun 32 koşu %{100*np.sort(counts)[-32:].sum()/n:.1f}'i")

    sp = assign(lengths, ratios, args.min_fault_len)

    # Pencere sayısı tahmini: koşu içinde SG kenar payı (window//2 her iki uçta)
    # düşüldükten sonra kaç adım sığdığı. Gerçek sayı `valid` maskesine bağlı,
    # bu yüzden burada üst sınır olarak raporlanıyor.
    def est_windows(rs: list[int]) -> int:
        return sum(max(0, (lengths[r] - args.window) // args.stride + 1) for r in rs)

    print()
    print(f"  {'bölme':<8}{'koşu':>7}{'örnek':>11}{'pay':>8}{'hedef':>8}"
          f"{'≈pencere':>11}{'arıza taşıyabilen':>19}")
    print("  " + "-" * 70)
    meta = {}
    for k in ("train", "val", "test"):
        rs = sp[k]
        smp = sum(lengths[r] for r in rs)
        host = [r for r in rs if lengths[r] >= args.min_fault_len]
        print(f"  {k:<8}{len(rs):>7}{smp:>11,}{100*smp/n:>7.1f}%"
              f"{100*ratios[k]:>7.1f}%{est_windows(rs):>11,}{len(host):>19}")
        meta[k] = {"n_runs": len(rs), "n_samples": smp, "share": smp / n,
                   "est_windows": est_windows(rs), "n_fault_hosts": len(host)}

    inter = set(sp["train"]) & set(sp["val"]) | set(sp["val"]) & set(sp["test"]) \
        | set(sp["train"]) & set(sp["test"])
    if inter:
        print(f"\nHATA: bölmeler kesişiyor: {sorted(inter)[:10]}", file=sys.stderr)
        return 2
    if sum(len(v) for v in sp.values()) != len(lengths):
        print("\nHATA: koşu sayısı tutmuyor", file=sys.stderr)
        return 2

    # Poz kapsaması: bölmeler aynı çalışma uzayını görüyor mu? Görmüyorlarsa
    # test kümesindeki her yüksek skor "anomali" değil "yeni poz" demektir — ve
    # bunu eğitimden SONRA değil ÖNCE bilmek gerekir. Kapsama eksikse bölme
    # kabul edilebilir ama makalede açıkça raporlanmalıdır.
    coverage = {}
    try:
        qdf = pd.read_parquet(args.parquet,
                              columns=["run_id"] + [f"q_{j}" for j in range(1, 7)])
        rng_ = {}
        for k in ("train", "val", "test"):
            m = qdf["run_id"].isin(sp[k])
            rng_[k] = {j: (float(qdf.loc[m, f"q_{j}"].min()),
                           float(qdf.loc[m, f"q_{j}"].max())) for j in range(1, 7)}
        print("\n  eklem aralığı kapsaması (rad) — 'dışarı' = eğitimin görmediği bölge")
        print(f"  {'eklem':<8}{'eğitim':>16}{'doğrulama':>18}{'test':>16}{'dışarı':>10}")
        print("  " + "-" * 70)
        for j in range(1, 7):
            tr = rng_["train"][j]
            out_frac = {}
            for k in ("val", "test"):
                m = qdf["run_id"].isin(sp[k])
                v = qdf.loc[m, f"q_{j}"].to_numpy()
                out_frac[k] = float(((v < tr[0]) | (v > tr[1])).mean())
            cells = "".join(f"{rng_[k][j][0]:>7.2f}–{rng_[k][j][1]:<8.2f}"
                            for k in ("train", "val", "test"))
            worst = max(out_frac.values())
            print(f"  q{j:<7}{cells}{100*worst:>9.1f}%"
                  + ("  ⚠" if worst > 0.01 else ""))
            coverage[f"q_{j}"] = {"train": tr, "val": rng_["val"][j],
                                  "test": rng_["test"][j],
                                  "outside_train_frac": out_frac}
        worst_all = max(max(c["outside_train_frac"].values()) for c in coverage.values())
        if worst_all > 0.01:
            print(f"\n  ⚠ doğrulama/test örneklerinin %{100*worst_all:.1f}'i eğitimin")
            print("    hiç görmediği eklem aralığında. Bu bölmeyle ölçülen her yüksek")
            print("    skor kısmen 'yeni poz' demektir; makalede belirtilmeli.")
        else:
            print("\n  ✓ doğrulama ve test, eğitimin gördüğü eklem aralıklarının içinde")
    except Exception as e:
        print(f"  (kapsama özeti atlandı: {e})")

    payload = {
        "parquet": str(args.parquet), "n_runs": len(lengths), "n_samples": n,
        "ratios": ratios, "window": args.window, "stride": args.stride,
        "min_fault_len": args.min_fault_len,
        "method": "largest-first bin packing on relative sample deficit",
        "summary": meta,
        "train": sp["train"], "val": sp["val"], "test": sp["test"],
        "fault_hosts": [r for r in sp["test"] if lengths[r] >= args.min_fault_len],
        "coverage": coverage,
        "run_lengths": lengths,
    }
    Path(args.out).write_text(json.dumps(payload, indent=1), encoding="utf-8")
    print(f"\n  → {args.out}")
    print("  Bu dosyayı generate_residuals.py, train_ae.py ve evaluate_fusion.py okur.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
