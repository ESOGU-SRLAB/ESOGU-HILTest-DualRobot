#!/usr/bin/env python3
"""
calibrate_cell.py
=================
Normalleştirme sınırlarını ve eşiği GERÇEK HÜCREDE ölçer.

Neden gerekli
-------------
Bu çalışmanın merkezi bulgusu şuydu: çevrimdışı türetilen eşik donanıma taşınmaz.
2026-08-21 kampanyasında v2 için ölçüldü — çevrimdışı 0,6436, sahada gereken 18,0;
28 kat. Buna rağmen v3 eşiği yine çevrimdışı doğrulamadan türetilip (0,8553)
sahaya gönderildi ve 2026-08-26 oturumunda kararların %58'i alarm oldu.
Aynı hücrede v2'nin payı 15,6× iken v3'ünki 0,89× idi; skorlar aynı ölçekte,
yanlış olan yalnız eşiğin kaynağıydı.

Bu betik bildirinin Denk. 4/6 yapısını AYNEN korur — değişen tek şey (lo, span)
ve θ'nın hangi kümeden alındığıdır: temiz doğrulama PENCERELERİ yerine temiz
gerçek hücre KARARLARI. Makalenin D1 sapması için kurduğu argümanın aynısı.

Kalibrasyon koşusu nasıl olmalı
-------------------------------
  · Anomali İÇERMEMELİ (provoke edilmiş ya da kendiliğinden).
  · Görev çevriminin TAMAMINI kapsamalı — özellikle robotun en uzağa uzandığı
    pozları. 2026-08-26'da θ=18'in hemen altındaki dört tepe tam olarak orada
    oluştu ve anomali değildi; o pozlar kalibrasyon koşusunda yoksa sonra
    yanlış alarm olarak geri gelirler.
  · En az bir tam çevrim, tercihen üç.

Kullanım
--------
    python3 calibrate_cell.py --dir ~/anomali_kayit --runs skorlar_2026...csv \\
        --config <mevcut fusion_config.json> --out fusion_config_cell.json
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

NEED = {"s_kal", "s_ham", "birlesik", "hareket"}


def load(paths: list[str]) -> pd.DataFrame:
    frames = []
    for p in paths:
        d = pd.read_csv(p)
        miss = NEED - set(d.columns)
        if miss:
            print(f"  ⚠ {Path(p).name}: eksik sütun {sorted(miss)} → atlandı")
            continue
        d["_run"] = Path(p).name
        frames.append(d)
    if not frames:
        print("HATA: kullanılabilir koşu yok.", file=sys.stderr)
        sys.exit(2)
    return pd.concat(frames, ignore_index=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=None, help="skorlar_*.csv dizini")
    ap.add_argument("--runs", nargs="*", default=None, help="tek tek dosyalar")
    ap.add_argument("--config", required=True, help="mevcut fusion_config.json")
    ap.add_argument("--out", default="fusion_config_cell.json")
    ap.add_argument("--margin", type=float, default=1.5,
                    help="θ = (temiz koşunun tepe değeri) × margin. 1,0 sıfır pay "
                         "bırakır; 1,5 varsayılan. 2026-08-21'de seçilen aralığın "
                         "alt ucu da temiz çevrimin tavanıydı.")
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--scale", choices=["minmax", "p97"], default="p97",
                    help="normalleştirme genişliği. p97 (VARSAYILAN): model başına "
                         "temiz kararların 97. persentili — sağlam. minmax: bildirinin "
                         "formülü, ama gerçek hücrede yük taşıma platosuna kilitleniyor "
                         "ve genişliği 30'a çıkarıp gerçek anomalileri görünmez yapıyor "
                         "(26.08.2026'da ölçüldü).")
    ap.add_argument("--threshold", type=float, default=None,
                    help="θ'yı doğrudan ver (her iki rejime de aynı değer). Verilirse "
                         "--margin yok sayılır. Tek bir eşik, bildirinin θ_mutlak "
                         "yapısıyla aynı; rejim ayrımı offline'da anlamlı bir kazanç "
                         "vermemişti (ΔF1 −0,015 ± 0,088, 5 tohum).")
    ap.add_argument("--min-minutes", type=float, default=5.0,
                    help="bu süreden kısa kalibrasyon koşusunda gürültülü uyarı")
    args = ap.parse_args()

    paths = args.runs or sorted(glob.glob(str(Path(args.dir or ".") / "**" /
                                              "skorlar_*.csv"), recursive=True))
    d = load(paths)
    cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
    w = float(cfg["w_kal"])
    qd_min = float(cfg.get("motion_qd_min", 0.02))

    print("=" * 78)
    print("GERÇEK HÜCREDE KALİBRASYON")
    print("=" * 78)
    print(f"  {d['_run'].nunique()} koşu · {len(d):,} karar · {len(d)/args.rate/60:.1f} dk")
    print(f"  temel alınan yapılandırma: {args.config}  (w_kal={w:.2f})")

    # Kalibrasyon koşusu yeterince uzun ve kapsayıcı değilse, ölçülen tavan
    # görev çevriminin gerçek tavanı DEĞİLDİR ve eksik kalan pozlar sonradan
    # yanlış alarm olarak geri döner. 2026-08-26'da θ=18'in hemen altındaki dört
    # tepe robotun en uzağa uzandığı pozlarda oluştu ve anomali değildi.
    minutes = len(d) / args.rate / 60
    if minutes < args.min_minutes:
        print(f"\n  {'!'*70}")
        print(f"  KALİBRASYON KOŞUSU KISA: {minutes:.1f} dk (en az {args.min_minutes:.0f} dk önerilir).")
        print("  Ölçülen tavan görev çevriminin tavanı olmayabilir. Özellikle robotun")
        print("  en uzağa uzandığı pozlar koşuda yoksa, o pozlar sonradan yanlış")
        print("  alarm üretir. Tam bir görev çevrimi koşturup tekrar kalibre edin.")
        print(f"  {'!'*70}")

    mv = d["hareket"].to_numpy() > 0.5
    print(f"  hareketli karar payı: %{100*mv.mean():.0f}")
    if mv.all() or not mv.any():
        print("  ⚠ TEK rejim gözlendi. Rejim-koşullu eşik bu koşudan ölçülemez;")
        print("    eksik rejim için mevcut yapılandırmadaki değer korunacak.")

    # ── 1) normalleştirme sınırları: bildirinin min–max formülü, sınırlar
    #        temiz GERÇEK HÜCRE kararlarından ────────────────────────────
    print("\n" + "-" * 78)
    print("1) Normalleştirme sınırları (bildiri formülü, sınırlar gerçek hücreden)")
    print("-" * 78)
    scale, old = {}, cfg.get("scale", {})
    print(f"  {'model':<10}{'lo':>10}{'span':>12}{'span (eski)':>14}{'oran':>9}")
    for key, col in (("residual", "s_kal"), ("raw", "s_ham")):
        v = d[col].to_numpy()
        if args.scale == "p97":
            lo, span = 0.0, max(float(np.percentile(v, 97)), 1e-9)
        else:
            lo = float(np.min(v))
            span = max(float(np.max(v)) - lo, 1e-9)
        o = float(old.get(key, {}).get("span", np.nan))
        scale[key] = {"lo": lo, "span": span}
        print(f"  {key:<10}{lo:>10.5f}{span:>12.5f}{o:>14.5f}{span/o:>8.2f}x")
    print(f"  ölçek kuralı: {args.scale}")
    print("  Modelin ÇEVRİMDIŞI eşiği hücrede ölçek olarak kullanılamaz: ham modelin")
    print("  eşiği 0,3934, hücredeki normal hatası ~2,9 — 25 kat şişik. Bu yüzden")
    print("  %5 ağırlıklı model birleşiğin %30'unu sürüyordu.")

    def z(key, v):
        return (v - scale[key]["lo"]) / scale[key]["span"]

    fused = w * z("residual", d["s_kal"].to_numpy()) + \
        (1 - w) * z("raw", d["s_ham"].to_numpy())

    # ── 2) eşik: temiz koşunun TAVANI (21 Ağustos'un alt sınır kuralı) ──
    print("\n" + "-" * 78)
    print("2) Eşik — temiz koşunun tavanı × pay")
    print("-" * 78)
    thr = {}
    ceiling = {}
    for tag, m in (("static", ~mv), ("moving", mv)):
        if m.sum() < 50:
            keep = float(cfg.get("threshold_by_regime", {}).get(tag, np.nan))
            thr[tag] = keep
            ceiling[tag] = np.nan
            print(f"  {tag:<8} n={int(m.sum()):>6}  → yetersiz, mevcut değer korundu: {keep:.4f}")
            continue
        x = fused[m]
        ceiling[tag] = float(x.max())
        thr[tag] = (float(args.threshold) if args.threshold is not None
                    else float(x.max() * args.margin))
        print(f"  {tag:<8} n={int(m.sum()):>6}  medyan {np.median(x):7.4f}  "
              f"p99 {np.percentile(x,99):7.4f}  tavan {x.max():7.4f}  "
              f"→ θ = {thr[tag]:7.4f}"
              + (f"  (tavanın {thr[tag]/x.max():.2f} katı)" if x.max() > 0 else ""))
    if args.threshold is not None:
        print(f"\n  θ ELLE verildi: {args.threshold:g} (her iki rejim). --margin yok sayıldı.")
        for tag in ("static", "moving"):
            c = ceiling.get(tag, np.nan)
            if np.isfinite(c) and c > 0:
                print(f"    {tag:<8} temiz tavan {c:.4f} → pay {thr[tag]/c:.2f}x"
                      + ("   ⚠ TAVANIN ALTINDA, yanlış alarm üretir"
                         if thr[tag] < c else ""))

    print(f"\n  {'θ adayı':>10}{'duran YA':>11}{'hareketli YA':>15}{'toplam YA/dk':>15}")
    print("  " + "-" * 52)
    for mult in (1.0, 1.25, 1.5, 2.0, 3.0):
        cand = {k: (v / args.margin) * mult if np.isfinite(v) else v for k, v in thr.items()}
        t = np.where(mv, cand["moving"], cand["static"])
        fa = fused > t
        per = fa.sum() / (len(d) / args.rate / 60)
        print(f"  ×{mult:<9.2f}{100*(fa&~mv).mean():>10.1f}%{100*(fa&mv).mean():>14.1f}%"
              f"{per:>15.1f}")

    out = dict(cfg)
    out.update({
        "scale": scale, "threshold_by_regime": thr, "regime_threshold": True,
        "fused_threshold": float(np.nanmax(list(thr.values()))),
        "calibrated_on": "real cell", "calibration_runs": sorted(d["_run"].unique()),
        "calibration_decisions": int(len(d)),
        "margin": (None if args.threshold is not None else args.margin),
        "scale_rule": args.scale,
        "threshold_source": ("explicit" if args.threshold is not None
                             else "clean ceiling x margin"),
        "clean_ceiling": ceiling,
        "motion_qd_min": qd_min,
        "note": "lo/span ve theta gercek hucrenin TEMIZ kararlarindan olculdu. "
                "Cevrimdisi turetilen esik bu donanima tasinmiyor (2026-08-21: "
                "0.6436 -> 18.0; 2026-08-26 v3: 0.8553 ile kararlarin %58'i alarm).",
    })
    Path(args.out).write_text(json.dumps(out, indent=2, ensure_ascii=False),
                              encoding="utf-8")
    print(f"\n  → {args.out}")
    print("  Düğüme vermek için:  ros2 launch anomaly_detection detector.launch.py \\")
    print(f"      fusion_config:={Path(args.out).resolve()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
