#!/usr/bin/env python3
"""
verify_fusion_logic.py
======================
Çevrimiçi sistemin GERÇEKTEN iki modelin birleşimiyle çalıştığını kanıtlar.

"İki model birleşiyor" cümlesi kodda öyle yazdığı için doğru sayılmaz. Ağırlık
w_ham = 0,05 olduğunda ham modelin katkısı ölçüm hatası kadar kalıyor olabilir; o
durumda sistem adı birleşim olan tek modelli bir dedektördür. Bu betik üç şeyi
ayrı ayrı sınar:

  [1] YAPISAL   — bildirinin Denklem 4'ü ile düğümün karar yolunun eşlenmesi;
                  her sapma açıkça listelenir.
  [2] ÇALIŞMA   — her kararda İKİ ONNX oturumunun da gerçekten koştuğu
                  (sayaçla, iddiayla değil).
  [3] İŞLEVSEL  — ablasyon: yalnız kalıntı (w=1,00), yalnız ham (w=0,00) ve
                  birleşim (w=0,95) karşılaştırması. Birleşim her iki tekil
                  modelden de iyi değilse "birleşim" iddiası kozmetiktir.

Kullanım
--------
    python3 verify_fusion_logic.py
    python3 verify_fusion_logic.py --runs 6 --max-len 12000
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import deque
from pathlib import Path

import numpy as np
import pandas as pd

PKG = Path("/home/cem/colcon_ws/src/anomaly_detection")   # paket 20.08.2026da yeniden adlandirildi
BASE = Path("/home/cem/colcon_ws/src/anomaly_detection")

FAULTS = {
    "yok":            {"tr": "Arıza yok (temiz)", "kind": None},
    "motor_kaymasi":  {"tr": "Motor kayması", "kind": "ramp", "joint": 2, "amp_nm": 15.0},
    "carpisma":       {"tr": "Çarpışma", "kind": "wrench_pulse", "amp_n": 30.0},
    "gizyazar":       {"tr": "Gizyazar hatası", "kind": "q_step", "joint": 4, "amp_rad": 1.5},
    "sensor_gurultu": {"tr": "Sensör gürültüsü", "kind": "wrench_noise", "amp_n": 3.5},
}


def alarms(fused: np.ndarray, thr_abs: float, adaptive: bool, k: float,
           window: int, warmup: int) -> np.ndarray:
    """Düğümdeki karar kuralının birebir aynısı: mutlak VEYA uyarlanabilir."""
    hist: deque = deque(maxlen=window)
    out = np.zeros(len(fused), dtype=bool)
    for i, v in enumerate(fused):
        hit = v > thr_abs
        if adaptive and not hit and len(hist) >= warmup:
            h = np.fromiter(hist, dtype=np.float64)
            med = float(np.median(h))
            mad = float(np.median(np.abs(h - med)))
            hit = v > med + k * max(1.4826 * mad, 0.05 * med, 1e-9)
        out[i] = hit
        if not hit:
            hist.append(v)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean", default=str(BASE / "ur10e_clean.parquet"))
    ap.add_argument("--runs", type=int, default=6)
    ap.add_argument("--max-len", type=int, default=12000)
    ap.add_argument("--min-run", type=int, default=3000)
    ap.add_argument("--onset", type=float, default=0.5)
    ap.add_argument("--amp-scale", type=float, default=1.0,
                    help="arıza genliklerini ölçekle. 1,0'da genlikler DOYURUCU "
                         "(her iki model de her şeyi yakalıyor) ve ablasyon iki "
                         "modeli ayırt edemez; 0,2–0,4 ayırt edici bölgedir.")
    ap.add_argument("--reuse-traces", action="store_true",
                    help="skor izlerini yeniden hesaplama, .npz'den oku "
                         "(yalnız ablasyon/eşik analizi değiştiyse; ölçek veya model "
                         "değiştiyse izler geçersizdir)")
    args = ap.parse_args()

    sys.path.insert(0, str(PKG))
    sys.path.insert(0, str(BASE / "resources"))
    from anomaly_detection import detector as D
    import ur10_solver_py

    fc = json.loads((Path(__file__).resolve().parent / "fusion_v2" / "fusion_config_paket.json").read_text())
    sweep = {round(r["w_kal"], 2): r for r in fc["online_sweep"]}
    NORM_TXT = {
        "theta": "min–max (test sınırları)  →  z = S/θ",
        "minmax_val": "min–max — sınırlar TEMİZ DOĞRULAMA'dan",
    }
    norm = str(fc.get("norm", "theta"))
    amp = float(args.amp_scale)
    trace_file = (BASE / "fusion_v2" /
                  (f"fusion_logic_traces{'' if amp == 1.0 else f'_a{amp:g}'}.npz"))

    # ─────────────────────── [1] YAPISAL ───────────────────────
    print("=" * 78)
    print("[1] YAPISAL EŞLEME — 245.pdf Denklem 4  ↔  düğümün karar yolu")
    print("=" * 78)
    rows = [
        ("İki ayrı model, iki ayrı temsil uzayı",
         "12 kanal kalıntı + 24 kanal ham", "AYNI"),
        ("Anomali skoru = yeniden yapılanma MSE",
         "mean((recon − x)²), eğitimdeki kayıpla aynı", "AYNI"),
        ("Skorlar normalleştiriliyor",
         NORM_TXT.get(norm, norm),
         "SAPMA (D1)" if norm == "theta" else "SAPMA (D1, azaltıldı)"),
        ("S_bir = w_kal·S̄_kal + w_ham·S̄_ham",
         "fused = w_res·z_res + w_raw·z_raw", "AYNI"),
        ("w, 0,05 adımla taranıp optimize ediliyor",
         "21 nokta, evaluate_fusion.py", "AYNI"),
        ("Tekil eşik = doğrulama hatalarının P97'si",
         "metadata.json θ", "AYNI"),
        ("Birleşik skorun çalışma eşiği",
         "bildiride YOK (BestF1) → temiz doğrulamanın P97'si", "EKLEME (D2)"),
        ("Uyarlanabilir kural (medyan + k·MAD)",
         "bildiride yok; mutlak eşikle VEYA'lanıyor", "EKLEME (D3)"),
        ("Alarm gecikmesi (ardışık N karar)",
         "bildiride yok; varsayılan 2 karar = 100 ms", "EKLEME (D4)"),
    ]
    print(f"  {'Bildiri':<42}{'Düğüm':<48}{'Durum'}")
    print("  " + "-" * 108)
    for a, b, c in rows:
        print(f"  {a:<42}{b:<48}{c}")
    print(f"""
  D1 ZORUNLU: bildirinin min–max sınırları ARIZA ENJEKTE EDİLMİŞ test kümesinden
     geliyor; akış üzerinde geleceği bilemeyeceğimiz için nedensel değil. Yerine
     iki nedensel ölçek yarıştırılıyor (evaluate_fusion.py): `theta` (z = S/θ) ve
     `minmax_val` (BİLDİRİNİN min–max formülü, sınırlar temiz doğrulamadan).
     `minmax_val` kazanırsa D1 "formül farklı" olmaktan çıkıp yalnız "sınırlar
     arıza görmemiş kümeden" farkına iner. Aktif ölçek: {norm}.
  D2 ZORUNLU: bildiri birleşik skoru yalnız BestF1 ile raporluyor, yani çalışma
     eşiği tanımlamıyor. Tekil eşikler için kullandığı P97 kuralı birleşik skora
     da uygulandı — bildirinin kendi kuralının devamı.
  D3/D4 İSTEĞE BAĞLI: `adaptive:=false` ve `consecutive_for_alarm:=1` ile kapatılıp
     bildirinin saf davranışına dönülebilir.""")

    # ─────────────────────── [2] ÇALIŞMA ───────────────────────
    print("\n" + "=" * 78)
    print("[2] ÇALIŞMA — her kararda iki ONNX oturumu da koşuyor mu?")
    print("=" * 78)
    det = D.FusionDetector(
        BASE / "residual_ae_v2", BASE / "raw_ae_v2",
        BASE / "fusion_v2" / "fusion_config.json",
        BASE / "current_to_torque.json", BASE / "residual_calibration_clean.json",
        ur10_solver_py.InverseDynamicsSolverUR10(),
    )
    if det.norm != norm:
        raise SystemExit("config ile dedektör ölçeği uyuşmuyor")

    if args.reuse_traces and trace_file.exists():
        # İzler diskten: ONNX sayaç kanıtı önceki koşudan okunur (yeniden
        # üretilmez), ablasyon anında koşar. Ölçek değiştiyse izler geçersizdir.
        z = np.load(trace_file)
        if str(z["norm"]) != norm:
            raise SystemExit(f"{trace_file.name} '{z['norm']}' ölçeğiyle üretilmiş, "
                             f"config şu an '{norm}'. --reuse-traces olmadan çalıştır.")
        n_runs = int(z["runs"]); n_dec = int(z["decisions"])
        traces = {f: [(z[f"{f}__{i}__zr"], z[f"{f}__{i}__zw"], int(z[f"{f}__{i}__on"][0]))
                      for i in range(n_runs)] for f in FAULTS}
        prev = json.loads((BASE / "fusion_v2" / "fusion_logic_audit.json").read_text())
        counts, ok2 = prev["onnx_calls"], bool(prev["both_models_each_decision"])
        print(f"  izler {trace_file.name} dosyasından okundu "
              f"({n_runs} koşu, {n_dec:,} karar) — ONNX sayaçları önceki koşudan.")
    else:
        counts = {"residual": 0, "raw": 0}
        orig_score = D.OnnxAE.score

        def counted(self, window):
            counts["residual" if self.n_feat == 12 else "raw"] += 1
            return orig_score(self, window)

        D.OnnxAE.score = counted

        df = pd.read_parquet(args.clean)
        run = df["run_id"].to_numpy()
        e = np.concatenate([[0], np.flatnonzero(np.diff(run) != 0) + 1, [len(run)]])
        segs = sorted([(int(a), int(c - a)) for a, c in zip(e[:-1], e[1:])
                       if int(c - a) >= args.min_run], key=lambda s: -s[1])[:args.runs]
        Q = df[[f"q_{j}" for j in range(1, 7)]].to_numpy(np.float64)
        QD = df[[f"qd_{j}" for j in range(1, 7)]].to_numpy(np.float64)
        AMP = df[[f"tau_{j}" for j in range(1, 7)]].to_numpy(np.float64)
        WR = df[["fx", "fy", "fz", "tx", "ty", "tz"]].to_numpy(np.float64)
        del df
        nm_per_amp = np.array(
            json.loads((BASE / "current_to_torque.json").read_text())["nm_per_amp"])

        rng = np.random.default_rng(0)
        traces = {}
        n_dec = 0
        n_runs = len(segs)
        print(f"  {n_runs} koşu × {len(FAULTS)} senaryo (genlik ×{amp:g}), "
              f"skorlar toplanıyor...")
        for fname, cfg in FAULTS.items():
            traces[fname] = []
            for a, L0 in segs:
                L = min(L0, args.max_len)
                onset = int(L * args.onset)
                det.reset()
                zr, zw, idx = [], [], []
                for k in range(L):
                    i = a + k
                    q, qd = Q[i].copy(), QD[i].copy()
                    amps, wr = AMP[i].copy(), WR[i].copy()
                    if cfg["kind"] and k >= onset:
                        prog = (k - onset) / max(L - onset, 1)
                        if cfg["kind"] == "ramp":
                            amps[cfg["joint"]] += (amp * cfg["amp_nm"] * prog
                                                   / nm_per_amp[cfg["joint"]])
                        elif cfg["kind"] == "q_step":
                            q[cfg["joint"]] += amp * cfg["amp_rad"]
                        elif cfg["kind"] == "wrench_pulse":
                            c = onset + 0.15 * (L - onset)
                            wr += amp * cfg["amp_n"] * np.exp(-0.5 * ((k - c) / (0.04 * L)) ** 2)
                        elif cfg["kind"] == "wrench_noise":
                            wr += rng.normal(0.0, amp * cfg["amp_n"], 6)
                    r = det.push(q, qd, amps, wr)
                    if r is not None:
                        zr.append(r["z_residual"]); zw.append(r["z_raw"]); idx.append(k)
                        n_dec += 1
                traces[fname].append((np.array(zr), np.array(zw),
                                      int(np.searchsorted(idx, onset))))
            print(f"    {cfg['tr']:<22} ✓")
        D.OnnxAE.score = orig_score

        # İzler diske: z_kal ve z_ham AYRI saklanır, böylece herhangi bir w yeniden
        # birleştirilebilir ve ablasyon saniyeler içinde tekrar koşturulabilir.
        np.savez_compressed(
            trace_file, norm=norm, runs=n_runs, decisions=n_dec, amp_scale=amp,
            stride=det.stride, dt=det.extractor.dt,
            **{f"{f}__{i}__{k}": v
               for f, lst in traces.items()
               for i, (zr_, zw_, on_) in enumerate(lst)
               for k, v in (("zr", zr_), ("zw", zw_), ("on", np.array([on_])))})
        ok2 = counts["residual"] == counts["raw"] == n_dec
        print(f"\n  karar sayısı            : {n_dec:,}")
        print(f"  kalıntı modeli çağrısı  : {counts['residual']:,}")
        print(f"  ham modeli çağrısı      : {counts['raw']:,}")
        print(f"  → {'✅ Her kararda İKİ model de koşuyor.' if ok2 else '❌ Sayılar tutmuyor!'}")
        print(f"  izler → fusion_v2/{trace_file.name} "
              f"({trace_file.stat().st_size/1e6:.1f} MB, --reuse-traces ile tekrar kullanılır)")

    # ─────────────────────── [3] İŞLEVSEL ───────────────────────
    print("\n" + "=" * 78)
    print("[3] İŞLEVSEL — ablasyon: birleşim tekil modelleri gerçekten geçiyor mu?")
    print("=" * 78)
    print(f"  {'yapılandırma':<28}{'yanlış alarm':>13}", end="")
    for f in list(FAULTS)[1:]:
        print(f"{FAULTS[f]['tr'][:11]:>13}", end="")
    print(f"{'toplam':>9}")
    print("  " + "-" * 76)

    def evaluate(w: float, adaptive: bool) -> dict:
        thr = sweep[round(w, 2)]["threshold"]
        fa = []
        for zr, zw, _ in traces["yok"]:
            fa.append(alarms(w * zr + (1 - w) * zw, thr, adaptive,
                             det.adaptive_k, det.adaptive_window,
                             det.adaptive_warmup).mean())
        out = {"false_alarm": float(np.mean(fa)), "hits": {}, "total": 0}
        for fname in list(FAULTS)[1:]:
            hit = 0
            for zr, zw, on in traces[fname]:
                d = alarms(w * zr + (1 - w) * zw, thr, adaptive, det.adaptive_k,
                           det.adaptive_window, det.adaptive_warmup)
                if d[on:].any():
                    hit += 1
            out["hits"][fname] = hit
            out["total"] += hit
        return out

    n = n_runs
    results = {}
    for label, w in (("Yalnız Ham   (w_kal=0,00)", 0.00),
                     ("Yalnız Kalıntı (w_kal=1,00)", 1.00),
                     ("BİRLEŞİM     (w_kal=0,95)", 0.95)):
        r = evaluate(w, adaptive=True)
        results[label] = r
        line = f"  {label:<28}{100 * r['false_alarm']:>12.2f}%"
        for fname in list(FAULTS)[1:]:
            line += f"{str(r['hits'][fname]) + '/' + str(n):>13}"
        line += f"{str(r['total']) + '/' + str(4 * n):>9}"
        print(line)

    fus = results["BİRLEŞİM     (w_kal=0,95)"]
    only_r = results["Yalnız Kalıntı (w_kal=1,00)"]
    only_h = results["Yalnız Ham   (w_kal=0,00)"]
    print()
    better = fus["total"] > max(only_r["total"], only_h["total"])
    equal = fus["total"] == max(only_r["total"], only_h["total"])
    if better:
        print(f"  ✅ Birleşim ({fus['total']}/{4*n}) her iki tekil modeli de geçiyor "
              f"(kalıntı {only_r['total']}, ham {only_h['total']}). "
              f"İki modelin birlikte çalışması ölçülebilir katkı sağlıyor.")
    elif equal:
        print(f"  ⚠ Birleşim tekil en iyiyle EŞİT ({fus['total']}/{4*n}). Katkı bu "
              f"veri kümesinde gösterilemedi.")
    else:
        print(f"  ❌ Birleşim tekil modelden KÖTÜ. Ağırlık yeniden optimize edilmeli.")

    # Ham modelin marjinal katkısı: w=1,00 (saf kalıntı) → w=0,95
    print("\n  Ham modelin marjinal katkısı (w_kal 1,00 → 0,95), arıza tipine göre:")
    for fname in list(FAULTS)[1:]:
        d = fus["hits"][fname] - only_r["hits"][fname]
        sign = "+" if d > 0 else ("" if d else " ")
        print(f"    {FAULTS[fname]['tr']:<22}{only_r['hits'][fname]}/{n} → "
              f"{fus['hits'][fname]}/{n}   ({sign}{d})")

    # ── karar düzeyi ablasyon ────────────────────────────────────────────
    # Koşu düzeyi ölçüt ("koşuda en az bir kez tetiklendi mi") DOYUYOR: senaryo
    # başına yalnız `n` koşu var ve uyarlanabilir kural er geç tekil modelleri de
    # tetikliyor, bu yüzden tavan 4n'de sıkışıyor ve iki modelin farkını
    # gösteremiyor. Karar düzeyi ölçüt her kararı ayrı sayar — bildirinin pencere
    # düzeyinde raporladığı kesinlik/geri çağırma ile aynı granülerlik.
    def decision_level(w: float, adaptive: bool = True) -> dict:
        thr = sweep[round(w, 2)]["threshold"]
        tp = fp = fn = tn = 0
        for fname, cfg in FAULTS.items():
            for zr, zw, on in traces[fname]:
                d = alarms(w * zr + (1 - w) * zw, thr, adaptive, det.adaptive_k,
                           det.adaptive_window, det.adaptive_warmup)
                lab = np.zeros(len(d), dtype=bool)
                if cfg["kind"]:
                    lab[on:] = True          # başlangıçtan sonrası arızalı
                tp += int((d & lab).sum()); fp += int((d & ~lab).sum())
                fn += int((~d & lab).sum()); tn += int((~d & ~lab).sum())
        pr = tp / max(tp + fp, 1); rc = tp / max(tp + fn, 1)
        return {"tp": tp, "fp": fp, "fn": fn, "tn": tn, "precision": pr, "recall": rc,
                "f1": 2 * pr * rc / (pr + rc + 1e-12)}

    w_sel = round(float(fc["w_kal"]), 2)
    cfgs = (("Yalnız Ham   (w_kal=0,00)", 0.00),
            ("Yalnız Kalıntı (w_kal=1,00)", 1.00),
            ("BİRLEŞİM     (w_kal={:.2f})".format(w_sel).replace(".", ","), w_sel))

    # İki kural ayrı ayrı: bildirinin SAF kuralı (yalnız mutlak eşik) ve bizim
    # eklediğimiz uyarlanabilir kural (D3). Ayrım şart, çünkü uyarlanabilir kural
    # tekil modelleri de kurtarıyor ve birleşimin marjını gizliyor — yani "birleşim
    # işe yarıyor mu" sorusu ancak bildirinin kendi kuralı altında yanıtlanabilir.
    dec_all = {}
    for rule, adaptive in (("bildirinin saf kuralı (yalnız mutlak eşik)", False),
                           ("uyarlanabilir kural açık (D3)", True)):
        print(f"\n  Karar düzeyi ablasyon — {rule}:")
        print(f"  {'yapılandırma':<28}{'TP':>7}{'FP':>7}{'FN':>7}"
              f"{'kesinlik':>11}{'geri çağırma':>14}{'F1':>8}")
        print("  " + "-" * 82)
        d_ = {}
        for label, w in cfgs:
            r = decision_level(w, adaptive)
            d_[label] = dict(r, w_kal=w)
            print(f"  {label:<28}{r['tp']:>7,}{r['fp']:>7,}{r['fn']:>7,}"
                  f"{r['precision']:>11.3f}{r['recall']:>14.3f}{r['f1']:>8.3f}")
        f_fus = d_[cfgs[2][0]]["f1"]
        f_best = max(d_[cfgs[0][0]]["f1"], d_[cfgs[1][0]]["f1"])
        d_["_gain"] = f_fus - f_best
        dec_all[rule] = d_
        print(f"    → birleşim {f_fus:.3f} · en iyi tekil {f_best:.3f} · "
              f"kazanç {d_['_gain']:+.3f}")

    pure = dec_all["bildirinin saf kuralı (yalnız mutlak eşik)"]
    adap = dec_all["uyarlanabilir kural açık (D3)"]
    dec, gain = pure, pure["_gain"]
    print()
    if gain > 0.005:
        print(f"  ✅ Bildirinin kendi kuralı altında birleşim F1 {pure[cfgs[2][0]]['f1']:.3f}, "
              f"en iyi tekil {max(pure[cfgs[0][0]]['f1'], pure[cfgs[1][0]]['f1']):.3f} "
              f"(kazanç +{gain:.3f}) — iki modelin katkısı ölçülebilir.")
    else:
        print(f"  ⚠ Bildirinin kuralı altında bile kazanç {gain:+.3f}; birleşim katkısı yok.")
    if adap["_gain"] <= 0.005 < gain:
        print(f"  ℹ Uyarlanabilir kural açıkken kazanç {adap['_gain']:+.3f}'e düşüyor: kural,\n"
              f"    tekil modelleri de kurtardığı için birleşimin sağladığı marjın bir\n"
              f"    kısmının YERİNE geçiyor. İkisi birbirinin alternatifi değil —\n"
              f"    birleşim doğruluğu, kural ise iyi uyan koşulardaki körlüğü kapatıyor.")

    # Tam ağırlık taraması
    print("\n  Tam ağırlık taraması (tespit / yanlış alarm):")
    print(f"  {'w_kal':>7}{'koşu':>10}{'yanlış alarm':>15}{'karar F1':>11}")
    print("  " + "-" * 45)
    full = []
    for w in (0.0, 0.25, 0.5, 0.75, 0.90, 0.95, 1.0):
        r = evaluate(w, adaptive=True)
        dl = decision_level(w)
        full.append({"w_kal": w, "run_level": r["total"], "false_alarm": r["false_alarm"],
                     "decision_level": dl})
        mark = "  ← seçili" if abs(w - fc["w_kal"]) < 1e-9 else ""
        print(f"  {w:>7.2f}{r['total']:>7}/{4*n}{100*r['false_alarm']:>14.2f}%"
              f"{dl['f1']:>11.3f}{mark}")

    (BASE / "fusion_v2" /
     f"fusion_logic_audit{'' if amp == 1.0 else f'_a{amp:g}'}.json").write_text(
        json.dumps({"decisions": n_dec, "onnx_calls": counts,
                    "both_models_each_decision": ok2, "norm": norm, "amp_scale": amp,
                    "ablation": {k: {"false_alarm": v["false_alarm"],
                                     "hits": v["hits"], "total": v["total"]}
                                 for k, v in results.items()},
                    "ablation_decision_level": {
                        k: {kk: vv for kk, vv in v.items() if kk != "_gain"}
                        for k, v in dec_all.items()},
                    "decision_level_gain": {k: v["_gain"] for k, v in dec_all.items()},
                    "weight_sweep": full,
                    "runs": n}, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n  → fusion_v2/fusion_logic_audit"
          f"{'' if amp == 1.0 else f'_a{amp:g}'}.json")
    # Geçme koşulu: iki model de her kararda koşuyor VE birleşim hiçbir ölçütte
    # tekil modellerin gerisinde değil (koşu düzeyi doyduğu için karar düzeyi asıl).
    return 0 if (ok2 and not_worse(fus, only_r, only_h) and gain > -0.005) else 1


def not_worse(fus, a, b) -> bool:
    return fus["total"] >= max(a["total"], b["total"])


if __name__ == "__main__":
    sys.exit(main())
