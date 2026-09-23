#!/usr/bin/env python3
"""
analyze_session.py
==================
Gerçek hücre oturumunun karar kayıtlarını çözümler.

Neyi cevaplıyor
---------------
Çevrimdışı ölçüm şunu söylüyor: dört SENTETİK arıza fiziksel olarak tutarlı
biçimde (ölçüm uzayına) enjekte edildiğinde ham modelin katkısı yok (ΔPR-AUC
−0,003 ± 0,001, 5 tohum). Ama gerçek arızalar bu dördü değil ve elimizdeki tek
karşı kanıt gerçek robottan: 2026-08-21 oturumunda bir anda ham model kararı
sürüklüyordu (kendi eşiğinin 1,62 katı, kalıntı 0,98 katı).

Bu betik o soruyu veriyle kapatır: DOĞRULANMIŞ olaylarda hangi model önde?
Sistem w=0,95 ile dağıtıldığı için iki model de her kararda çalışıyor ve her
ikisinin skoru da kaydediliyor — yani soru oturum sonunda cevaplanabilir.

Ayrıca raporlar:
  · rejim-koşullu eşiğin sahada hangi oranda hangi kolu kullandığı
  · eşik taraması (her aday eşikte gerçek/yanlış alarm), kayıttan yeniden sayım
  · yanlış alarmların görev çevrimine kilitli olup olmadığı

Kullanım
--------
    python3 analyze_session.py --dir ~/anomali_kayit --labels ~/anomali_kayit/etiketler.json
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd

COLS_MIN = {"t_ros", "s_kal", "s_ham", "birlesik", "thr_mutlak"}


def blocks(flag: np.ndarray, need: int = 2) -> list[tuple[int, int]]:
    """Ardışık en az `need` karar süren alarm blokları (düğümün kuralıyla aynı)."""
    out, i, n = [], 0, len(flag)
    while i < n:
        if flag[i]:
            j = i
            while j < n and flag[j]:
                j += 1
            if j - i >= need:
                out.append((i, j))
            i = j
        else:
            i += 1
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.expanduser("~/anomali_kayit"))
    ap.add_argument("--labels", default=None,
                    help="operatörün doğruladığı olaylar (etiketler.json)")
    ap.add_argument("--sweep", default="0.1,0.25,0.5,0.86,1.5,3,6,12,25",
                    help="yeniden sayım için aday eşikler")
    ap.add_argument("--rate", type=float, default=20.0, help="karar/s")
    args = ap.parse_args()

    files = sorted(glob.glob(str(Path(args.dir) / "**" / "skorlar_*.csv"), recursive=True))
    if not files:
        print(f"HATA: {args.dir} altında skorlar_*.csv yok.", file=sys.stderr)
        return 2

    runs = []
    for f in files:
        d = pd.read_csv(f)
        if not COLS_MIN.issubset(d.columns):
            print(f"  ⚠ {Path(f).name}: beklenen sütunlar eksik, atlanıyor")
            continue
        # Skorun hiç üretilmediği koşu (ör. yanlış wrench konu adı) DIŞLANIR;
        # ama sırf robot durduğu için skoru düşük olan koşu MEŞRUDUR ve
        # dışlanmaz — rejim-koşullu eşik zaten tam bunun için var. 2026-08-21
        # oturumundaki 42 saniyelik koşu bu ikinci türdendi (maks 0,0057,
        # diğer koşuların medyanı 1,1–5,5): ölü değil, tamamen duran.
        b = d["birlesik"].to_numpy()
        if not np.isfinite(b).any() or float(np.nanmax(b)) == 0.0:
            print(f"  ⚠ {Path(f).name}: hiç skor üretilmemiş → dışlandı")
            continue
        runs.append((Path(f).name, d))

    tot = sum(len(d) for _, d in runs)
    print("=" * 78)
    print("OTURUM ÇÖZÜMLEMESİ")
    print("=" * 78)
    print(f"  {len(runs)} koşu · {tot:,} karar · {tot/args.rate/60:.1f} dk kayıtlı karar")
    # Tamamen duran koşular ayrıca sayılır: global eşik altında bunlar paydayı
    # şişirip yanlış alarm oranını olduğundan iyi gösterir.
    med_all = float(np.median(np.concatenate([d["birlesik"].to_numpy() for _, d in runs])))
    still = [(n, d) for n, d in runs
             if float(np.median(d["birlesik"].to_numpy())) < 0.05 * med_all]
    if still:
        ns = sum(len(d) for _, d in still)
        print(f"  bunların {len(still)}'i neredeyse tamamen duran "
              f"({ns:,} karar, %{100*ns/tot:.1f}) — oran hesaplarında ayrı tutulmalı:")
        for n, d in still:
            print(f"    {n}  medyan {np.median(d['birlesik'].to_numpy()):.4f}  "
                  f"maks {d['birlesik'].to_numpy().max():.4f}")

    has_reg = all("hareket" in d.columns for _, d in runs)
    if has_reg:
        mv = np.concatenate([d["hareket"].to_numpy() > 0.5 for _, d in runs])
        thr = np.concatenate([d["thr_mutlak"].to_numpy() for _, d in runs])
        print(f"  hareketli karar payı: %{100*mv.mean():.0f}")
        u = np.unique(np.round(thr, 6))
        if len(u) > 1:
            print(f"  rejim-koşullu eşik ETKİN: kullanılan eşikler {u.tolist()}")
            print(f"    duran kolda %{100*(~mv).mean():.0f} · "
                  f"hareketli kolda %{100*mv.mean():.0f} karar")
        else:
            print(f"  tek global eşik kullanılmış: {u[0]:.4f}")

    # ── hangi model önde ──────────────────────────────────────────────────
    # Her modelin skoru KENDİ P97 eşiğine oranlanır; oran > 1 ise o model
    # kendi başına tetiklerdi. Ham modelin gerçek olaylarda önde olduğu tek bir
    # karar bile, çevrimdışı "ham modelin katkısı yok" sonucunun sentetik
    # senaryolara özgü olduğunu gösterir.
    print("\n" + "=" * 78)
    print("HANGİ MODEL ÖNDE — çevrimdışı ölçümün gerçek arızalarda sınanması")
    print("=" * 78)
    need = {"hit_kal", "hit_ham"}
    if not all(need.issubset(d.columns) for _, d in runs):
        print("  (hit_kal/hit_ham sütunları yok — düğüm eski sürümle koşmuş)")
    else:
        A = pd.concat([d for _, d in runs], ignore_index=True)
        al = A["birlesik"].to_numpy() > A["thr_mutlak"].to_numpy()
        hk = A["hit_kal"].to_numpy() > 0.5
        hh = A["hit_ham"].to_numpy() > 0.5
        n = max(int(al.sum()), 1)
        print(f"  eşiği aşan {int(al.sum()):,} kararda:")
        print(f"    yalnız kalıntı tetiklerdi : {int((al&hk&~hh).sum()):>6,}  "
              f"(%{100*(al&hk&~hh).sum()/n:.1f})")
        print(f"    yalnız ham tetiklerdi     : {int((al&hh&~hk).sum()):>6,}  "
              f"(%{100*(al&hh&~hk).sum()/n:.1f})   ← ham modelin katkısı")
        print(f"    ikisi birden              : {int((al&hk&hh).sum()):>6,}  "
              f"(%{100*(al&hk&hh).sum()/n:.1f})")
        print(f"    hiçbiri (yalnız birleşim) : {int((al&~hk&~hh).sum()):>6,}  "
              f"(%{100*(al&~hk&~hh).sum()/n:.1f})")

    # ── eşik taraması: kayıttan yeniden sayım ────────────────────────────
    print("\n" + "=" * 78)
    print("EŞİK TARAMASI — tüm kararlardan yeniden sayım")
    print("=" * 78)
    labels = None
    if args.labels and Path(args.labels).exists():
        labels = json.loads(Path(args.labels).read_text(encoding="utf-8"))
        print(f"  operatör etiketleri: {args.labels}")
    else:
        print("  ⚠ etiket dosyası yok → bloklar yalnız SAYILIR, gerçek/yanlış ayrımı yapılamaz")
    print(f"\n  {'θ':>8}{'blok':>7}{'blok/saat':>11}{'alarm doluluğu':>17}")
    print("  " + "-" * 44)
    hours = tot / args.rate / 3600
    for th in [float(x) for x in args.sweep.split(",")]:
        nb, duty = 0, 0
        for _, d in runs:
            f = d["birlesik"].to_numpy() > th
            bl = blocks(f)
            nb += len(bl)
            duty += sum(j - i for i, j in bl)
        print(f"  {th:>8.3f}{nb:>7}{nb/max(hours,1e-9):>11.1f}{100*duty/max(tot,1):>16.1f}%")
    print("\n  Not: 'blok/saat' iki rejimi havuzlar. Yanlız alarmların görev çevrimine")
    print("  kilitli olduğu ölçüldüyse doğru gözlem birimi geçen süre değil ÇEVRİMdir.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
