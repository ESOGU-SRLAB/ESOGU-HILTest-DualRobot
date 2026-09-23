#!/usr/bin/env python3
"""
check_fmu.py
============
Ters dinamik modelinin GERÇEKTEN çalıştığını, artık üretmeye başlamadan önce doğrular.

Bunu her şeyden önce çalıştır. Geçmiyorsa devam etme — çünkü `resources/model.py`
solver yüklenemediğinde hatayı yutup sıfır tork döndürüyor ve bozuk parquet üretiliyor.

Kontroller
----------
  1. Ortam: işletim sistemi, mimari, Python sürümü, `.so` uyumluluğu
  2. `so` arka ucu: modül import ediliyor mu, sınıf örnekleniyor mu
  3. `fmu` arka ucu: FMU örnekleniyor mu (fmpy varsa)
  4. Sıfır testi: bilinen bir duruşta τ sıfırdan farklı mı
  5. Fizik testi: tabanı dik robotta yerçekimi Eklem 1 etrafında tork üretmez
  6. Eşdeğerlik: `so` ve `fmu` arka uçları aynı sayıları mı veriyor

Kullanım
--------
    python3 check_fmu.py
    python3 check_fmu.py --resources resources --fmu UR10e_InverseDynamics.fmu
"""

from __future__ import annotations

import argparse
import glob
import platform
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fmu_backend import SoBackend, FmuBackend, SolverUnavailable  # noqa: E402

JN = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--resources", default="resources")
    ap.add_argument("--fmu", default="UR10e_InverseDynamics.fmu")
    ap.add_argument("--skip-fmu", action="store_true")
    args = ap.parse_args()

    print("=" * 70)
    print("TERS DİNAMİK SAĞLIK KONTROLÜ")
    print("=" * 70)

    # ── 1) ortam ──
    print("\n[1] Ortam")
    print(f"  işletim sistemi : {platform.system()} {platform.release()}")
    print(f"  mimari          : {platform.machine()}")
    print(f"  Python          : {sys.version.split()[0]}")
    so = glob.glob(str(Path(args.resources) / "ur10_solver_py*"))
    for p in so:
        print(f"  solver dosyası  : {Path(p).name}")
    need = (platform.system() == "Linux" and platform.machine() in ("x86_64", "AMD64")
            and sys.version_info[:2] == (3, 10))
    if not need:
        print("  ⚠ Mevcut derleme Linux x86-64 / CPython 3.10 içindir.")
        print("    Bu ortam uyuşmuyor → `so` arka ucu muhtemelen yüklenmeyecek.")
        print("    WSL2 Ubuntu 22.04 altında `python3.10` ile çalıştır.")
    else:
        print("  ✅ Ortam solver derlemesiyle uyumlu (Linux x86-64, CPython 3.10)")

    # ── test duruşları ──
    q0 = np.zeros((1, 6))
    # kol yatay uzanmış: yerçekimi shoulder_lift ve elbow'da büyük tork üretmeli
    q_h = np.array([[0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0]])
    zero = np.zeros((1, 6))

    backends = {}

    # ── 2) so arka ucu ──
    print("\n[2] `so` arka ucu (doğrudan C++ import)")
    try:
        b = SoBackend(args.resources)
        backends["so"] = b
        print("  ✅ ur10_solver_py import edildi, InverseDynamicsSolverUR10 örneklendi")
    except SolverUnavailable as e:
        print(f"  ❌ {e}")

    # ── 3) fmu arka ucu ──
    if not args.skip_fmu:
        print("\n[3] `fmu` arka ucu (fmpy → UniFMU)")
        try:
            b = FmuBackend(args.fmu)
            backends["fmu"] = b
            print("  ✅ FMU örneklendi ve başlatıldı")
        except SolverUnavailable as e:
            print(f"  ❌ {e}")
        except Exception as e:
            print(f"  ❌ FMU başlatılamadı: {type(e).__name__}: {e}")
    else:
        print("\n[3] `fmu` arka ucu — atlandı (--skip-fmu)")

    if not backends:
        print("\n❌ Hiçbir arka uç çalışmıyor. Artık üretimine GEÇME.")
        return 2

    # ── 4-5) sıfır ve fizik testleri ──
    results = {}
    for name, b in backends.items():
        print(f"\n[4] Sıfır testi — arka uç `{name}`")
        t0 = b.torques(q0, zero, zero)
        th = b.torques(q_h, zero, zero)
        results[name] = (t0, th)
        print(f"  q = 0 duruşunda      τ = {np.round(t0[0], 4)}")
        print(f"  kol yatay duruşunda  τ = {np.round(th[0], 4)}")
        if np.abs(th).max() < 1e-9:
            print("  ❌ τ tamamen sıfır → solver hesaplamıyor.")
            return 2
        print("  ✅ τ sıfırdan farklı")

        print(f"\n[5] Fizik testi — arka uç `{name}`")
        print(f"  {'eklem':<16}{'τ (yatay kol)':>15}")
        print("  " + "-" * 32)
        for j in range(6):
            print(f"  {JN[j]:<16}{th[0, j]:>15.4f}")
        r = abs(th[0, 0]) / (abs(th[0, 1]) + 1e-9)
        print(f"\n  |Eklem1| / |Eklem2| = {r:.4f}")
        if abs(th[0, 1]) < 1e-6:
            print("  ⚠ shoulder_lift torku ~0 — yerçekimi terimi yok gibi, şüpheli.")
        elif r < 0.05:
            print("  ✅ Beklenen yerçekimi imzası: düşey eksenli Eklem 1'de tork ≈ 0,")
            print("     shoulder_lift'te büyük tork. Model fiziksel olarak tutarlı.")
        else:
            print("  ⚠ Eklem 1 torku beklenenden büyük — taban yönelimi/konvansiyon farkı olabilir.")

    # ── 6) eşdeğerlik ──
    if len(backends) == 2:
        print("\n[6] Arka uç eşdeğerliği (`so` vs `fmu`)")
        rng = np.random.default_rng(0)
        Q = rng.uniform(-np.pi, np.pi, (200, 6))
        QD = rng.uniform(-1.0, 1.0, (200, 6))
        QDD = rng.uniform(-2.0, 2.0, (200, 6))
        a = backends["so"].torques(Q, QD, QDD)
        b_ = backends["fmu"].torques(Q, QD, QDD)
        d = np.abs(a - b_).max()
        rel = d / (np.abs(a).max() + 1e-12)
        print(f"  200 rastgele durumda maks mutlak fark = {d:.3e} Nm  (bağıl {rel:.2e})")
        if d < 1e-6:
            print("  ✅ İki yol birebir aynı — `so` arka ucu güvenle kullanılabilir (çok daha hızlı).")
        else:
            print("  ⚠ Fark var; artık üretiminde `--backend fmu` kullan.")
    elif "so" in backends:
        print("\n[6] Eşdeğerlik testi atlandı (fmu arka ucu yok).")
    else:
        print("\n[6] Eşdeğerlik testi atlandı (so arka ucu yok).")

    for b in backends.values():
        b.close()

    print("\n" + "=" * 70)
    print(f"SONUÇ: çalışan arka uç(lar): {', '.join(backends)}")
    print("Artık üretimine geçebilirsin.")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
