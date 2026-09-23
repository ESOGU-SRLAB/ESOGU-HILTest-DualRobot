#!/usr/bin/env python3
"""
replay_detector.py
==================
Çevrimiçi tespit düğümünü ROS'suz, gerçek veriyle uçtan uca sınar.

`ur10e_clean.parquet` örneklerini tek tek `FusionDetector`'a besler — yani ROS
düğümünün gerçekten çalıştırdığı sınıfa, teste özel bir kopyasına değil. İki şeyi
ölçer:

  1. Temiz veride yanlış alarm oranı (eşiğin canlıda ne yaptığı).
  2. Enjekte edilen arızada tespit edilip edilmediği ve KAÇ MİLİSANİYEDE.

Arıza enjeksiyonu ölçüm uzayında yapılır (motor akımına Nm karşılığı eklenir), yani
düğümün gördüğü şeyin aynısı — bildirinin çevrimdışı enjeksiyonu gibi kalıntı
kanalına doğrudan değil. Gerçekçi olan bu.

Kullanım
--------
    python3 replay_detector.py
    python3 replay_detector.py --fault carpisma --runs 6
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

def _find_package(env: str = "AD_PACKAGE") -> Path:
    """
    ROS paketinin kökünü bulur (içinde anomaly_detection/features.py olan dizin).

    Bu betik paketin dışına taşınabilir — nitekim taşındı: hat dosyaları büyük
    parquet ve model ağırlıkları içerdiği için depoya girmiyor ve paketin yanında
    durmuyor. Yolu `__file__`'a göre saymak bu yüzden kırılgan; sırayla ortam
    değişkenine, bilinen konumlara ve üst dizinlere bakılır.
    """
    import os
    cands = []
    if os.environ.get(env):
        cands.append(Path(os.environ[env]))
    cands += [Path.home() / "colcon_ws" / "src" / "anomaly_detection"]
    cands += list(Path(__file__).resolve().parents)
    for c in cands:
        if (c / "anomaly_detection" / "features.py").exists():
            return c
    raise SystemExit(
        "anomaly_detection paketi bulunamadi. "
        f"{env} ortam degiskeniyle verin, or.:  "
        f"export {env}=~/colcon_ws/src/anomaly_detection")


PKG = _find_package()
BASE = PKG

# Ölçüm uzayında (akım / newton) uygulanan arızalar. Genlikler bildiriden;
# tork kaynaklı olanlar Nm→A çevrilerek uygulanır.
FAULTS = {
    "yok":            {"tr": "Arıza yok (temiz)", "kind": None},
    "motor_kaymasi":  {"tr": "Motor kayması", "kind": "ramp", "joint": 2, "amp_nm": 15.0},
    "carpisma":       {"tr": "Çarpışma", "kind": "wrench_pulse", "amp_n": 30.0},
    "gizyazar":       {"tr": "Gizyazar hatası", "kind": "q_step", "joint": 4, "amp_rad": 1.5},
    "sensor_gurultu": {"tr": "Sensör gürültüsü", "kind": "wrench_noise", "amp_n": 3.5},
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean", default=str(BASE / "ur10e_clean.parquet"))
    ap.add_argument("--fault", default="all", choices=list(FAULTS) + ["all"])
    ap.add_argument("--runs", type=int, default=6, help="kaç uzun koşu denensin")
    ap.add_argument("--min-run", type=int, default=1500)
    ap.add_argument("--onset", type=float, default=0.60, help="arızanın başladığı oran")
    ap.add_argument("--residual-dir", default=str(BASE / "residual_ae_v2"))
    ap.add_argument("--raw-dir", default=str(BASE / "raw_ae_v2"))
    ap.add_argument("--fusion-config", default=str(BASE / "fusion_v2" / "fusion_config.json"))
    ap.add_argument("--residual-calib", default=str(BASE / "residual_calibration_clean.json"))
    ap.add_argument("--friction-model", default="",
                    help="modeller sürtünme çıkarılmış kalıntıyla eğitildiyse ŞART")
    # Hangi koşularda oynatıldığı sonucun ANLAMINI belirler. Eski çağrı en uzun
    # koşuları seçiyordu; bunların hepsi eğitim koşusuydu ve "temiz veride %0,0
    # yanlış alarm" sayısı oradan geliyordu — bir başarı değil, tanım gereği.
    # Varsayılan artık TEST koşuları.
    ap.add_argument("--splits", default=str(Path(__file__).resolve().parent / "splits.json"))
    ap.add_argument("--split", default="test", choices=["test", "val", "train", "any"])
    ap.add_argument("--out", default=str(BASE / "fusion_v2" / "replay_results.json"))
    args = ap.parse_args()

    sys.path.insert(0, str(PKG))
    sys.path.insert(0, str(BASE / "resources"))
    sys.path.insert(0, str(PKG.parent))
    from anomaly_detection.detector import FusionDetector
    import ur10_solver_py

    ctt = json.loads((BASE / "current_to_torque.json").read_text())
    nm_per_amp = np.array(ctt["nm_per_amp"])

    det = FusionDetector(
        residual_model_dir=args.residual_dir,
        raw_model_dir=args.raw_dir,
        fusion_config=args.fusion_config,
        current_to_torque=BASE / "current_to_torque.json",
        residual_calibration=args.residual_calib,
        friction_model=(args.friction_model or None),
        solver=ur10_solver_py.InverseDynamicsSolverUR10(),
    )

    print("=" * 74)
    print("ÇEVRİMİÇİ TESPİT — GERÇEK VERİYLE YENİDEN OYNATMA")
    print("=" * 74)
    print(f"  kalıntı {det.ae_res.provider}  θ={det.ae_res.threshold:.4f} | "
          f"ham θ={det.ae_raw.threshold:.4f} | birleşik θ={det.thr_fused:.5f}")
    print(f"  w_kal={det.w_res:.2f}  w_ham={det.w_raw:.2f}  "
          f"karar periyodu {det.stride*det.extractor.dt*1000:.0f} ms")

    df = pd.read_parquet(args.clean)
    run = df["run_id"].to_numpy()
    Q = df[[f"q_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    QD = df[[f"qd_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    AMP = df[[f"tau_{j}" for j in range(1, 7)]].to_numpy(np.float64)
    WR = df[["fx", "fy", "fz", "tx", "ty", "tz"]].to_numpy(np.float64)
    del df

    edges = np.concatenate([[0], np.flatnonzero(np.diff(run) != 0) + 1, [len(run)]])
    segs = [(int(a), int(c - a)) for a, c in zip(edges[:-1], edges[1:])]
    keep_runs = None
    if args.split != "any":
        sp_path = Path(args.splits)
        if not sp_path.exists():
            print(f"HATA: {sp_path} yok. --split any ile bölmeden bağımsız "
                  f"oynatabilirsin ama o zaman sonuç bir genelleme ölçümü DEĞİLDİR.",
                  file=sys.stderr)
            return 2
        keep_runs = set(json.loads(sp_path.read_text(encoding="utf-8"))[args.split])
        segs = [(a, L) for a, L in segs if int(run[a]) in keep_runs]
    segs = sorted([s for s in segs if s[1] >= args.min_run], key=lambda s: -s[1])[:args.runs]
    if not segs:
        print(f"HATA: '{args.split}' bölmesinde >= {args.min_run} örneklik koşu yok.",
              file=sys.stderr)
        return 2
    n_smp = sum(L for _, L in segs)
    print(f"  bölme '{args.split}': {len(segs)} koşu, {n_smp:,} örnek "
          f"({n_smp*0.002:.0f} s = {n_smp*0.002/60:.1f} dk robot zamanı)")
    if args.split == "train":
        print("  ⚠ EĞİTİM koşuları — buradaki temiz yanlış alarm oranı bir başarı")
        print("    ölçüsü değildir; model bu pencereleri zaten görmüştür.")
    print()

    names = list(FAULTS) if args.fault == "all" else [args.fault]
    print(f"  {'senaryo':<22}{'karar':>8}{'alarm':>8}{'oran':>8}"
          f"{'tespit gecikmesi':>20}{'çıkarım':>10}")
    print("  " + "-" * 76)

    rng = np.random.default_rng(0)
    results = {}
    for fname in names:
        cfg = FAULTS[fname]
        n_dec = n_alarm = 0
        lags, infer = [], []
        for a, L in segs:
            det.reset()
            onset = int(L * args.onset)
            first_hit = None
            for k in range(L):
                i = a + k
                q, qd = Q[i].copy(), QD[i].copy()
                amps, wr = AMP[i].copy(), WR[i].copy()
                if cfg["kind"] and k >= onset:
                    prog = (k - onset) / max(L - onset, 1)
                    if cfg["kind"] == "ramp":
                        j = cfg["joint"]
                        amps[j] += cfg["amp_nm"] * prog / nm_per_amp[j]
                    elif cfg["kind"] == "q_step":
                        q[cfg["joint"]] += cfg["amp_rad"]
                    elif cfg["kind"] == "wrench_pulse":
                        c, s = onset + 0.15 * (L - onset), 0.04 * L
                        wr += cfg["amp_n"] * np.exp(-0.5 * ((k - c) / s) ** 2)
                    elif cfg["kind"] == "wrench_noise":
                        wr += rng.normal(0.0, cfg["amp_n"], 6)
                t0 = time.perf_counter()
                r = det.push(q, qd, amps, wr)
                if r is None:
                    continue
                infer.append((time.perf_counter() - t0) * 1e3)
                # Arıza penceresi: kararın kapsadığı pencerenin sonu onset'i geçtiyse
                after = k >= onset
                if cfg["kind"] is None or after:
                    n_dec += 1
                    if r["detected"]:
                        n_alarm += 1
                        if after and first_hit is None:
                            first_hit = k
            if cfg["kind"] and first_hit is not None:
                lags.append((first_hit - onset) * det.extractor.dt * 1000)

        rate = n_alarm / max(n_dec, 1)
        if cfg["kind"] is None:
            lag_txt = "—  (yanlış alarm oranı)"
        elif lags:
            lag_txt = f"{np.median(lags):.0f} ms  ({len(lags)}/{len(segs)} koşu)"
        else:
            lag_txt = "TESPİT EDİLEMEDİ"
        print(f"  {cfg['tr']:<22}{n_dec:>8,}{n_alarm:>8,}{100*rate:>7.1f}%"
              f"{lag_txt:>20}{np.mean(infer):>9.2f}ms")
        results[fname] = {"decisions": n_dec, "alarms": n_alarm, "rate": rate,
                          "median_lag_ms": float(np.median(lags)) if lags else None,
                          "runs_detected": len(lags), "runs_total": len(segs),
                          "infer_ms": float(np.mean(infer))}

    print()
    fp = results.get("yok", {}).get("rate")
    if fp is not None:
        print(f"  Temiz veride yanlış alarm: %{100*fp:.1f}  "
              f"(eşik doğrulama setinin P97'sinden geldiği için ~%3 beklenir)")
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(
        json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"  → {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
