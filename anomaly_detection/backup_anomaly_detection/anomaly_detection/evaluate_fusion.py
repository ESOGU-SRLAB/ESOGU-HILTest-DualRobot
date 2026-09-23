#!/usr/bin/env python3
"""
evaluate_fusion.py
==================
245.pdf'in tam değerlendirme protokolünü yeniden üretir.

Akış
----
1. Dört sentetik arıza senaryosunun her biri, ham ve kalıntı temsil uzaylarına
   kendi genliğiyle enjekte edilir.
2. Her senaryoda TÜM pencereler (44.974) iki modelle skorlanır
   → 4 × 44.974 = 179.896 test penceresi, 14.402'si arızalı (%8,0).
3. Tekil model metrikleri: AUC · PR-AUC · BestF1 (eşikten bağımsız en yüksek F1).
4. Skor düzeyinde birleşim: min–max normalleştirme sonrası
       S_bir = w_kal·S̄_kal + w_ham·S̄_ham ,  w_kal ∈ [0,1] adım 0,05
   ayrıca MAX ve OR stratejileri.
5. Referans yöntemler (kalıntı öznitelikleri üzerinde): Kalıntı Norm Eşiği,
   Isolation Forest (kontaminasyon 0,08), One-Class SVM (RBF, ν=0,05).
6. Tamamlayıcılık analizi ve arıza tipine göre BestF1 tablosu.

Kullanım
--------
    python evaluate_fusion.py --residual-dir residual_ae_model --raw-dir raw_ae_model
    python evaluate_fusion.py ... --limit-rows 200000        # hızlı deneme
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
import torch
from torch.utils.data import DataLoader
from sklearn.metrics import roc_auc_score, roc_curve, precision_recall_curve, average_precision_score
from sklearn.ensemble import IsolationForest
from sklearn.svm import OneClassSVM
from sklearn.preprocessing import StandardScaler

sys.path.insert(0, str(Path(__file__).resolve().parent))
from models import PRESETS, WindowDataset, build, window_starts  # noqa: E402
from inject_faults import FAULTS, inject, window_labels, build_scale  # noqa: E402


# ───────────────────────── yardımcılar ─────────────────────────

def best_f1(y: np.ndarray, s: np.ndarray) -> tuple[float, float, float, float]:
    """Eşikten bağımsız en yüksek F1 ve ona karşılık gelen P, R, eşik."""
    p, r, t = precision_recall_curve(y, s)
    f = 2 * p * r / (p + r + 1e-12)
    i = int(np.nanargmax(f))
    return float(f[i]), float(p[i]), float(r[i]), float(t[min(i, len(t) - 1)])


def metrics(y: np.ndarray, s: np.ndarray) -> dict:
    f1, p, r, thr = best_f1(y, s)
    return {"auc": float(roc_auc_score(y, s)),
            "pr_auc": float(average_precision_score(y, s)),
            "best_f1": f1, "precision": p, "recall": r, "threshold": thr}


def minmax(s: np.ndarray) -> np.ndarray:
    lo, hi = float(s.min()), float(s.max())
    return (s - lo) / (hi - lo + 1e-12)


def score_windows(model, data, mean, std, W, S, dev, bs=512, starts=None) -> np.ndarray:
    ds = WindowDataset((data - mean) / std, W, S)
    if starts is not None:
        ds.starts = starts; ds.n_windows = len(starts)
    ld = DataLoader(ds, batch_size=bs, shuffle=False)
    out = []
    with torch.no_grad():
        for b in ld:
            b = b.to(dev, non_blocking=True)
            out.append(((model(b) - b) ** 2).mean(dim=(1, 2)).float().cpu().numpy())
    return np.concatenate(out)


def win_feats(data: np.ndarray, W: int, S: int, chunk: int = 4096,
              starts: np.ndarray | None = None) -> np.ndarray:
    """
    Pencere başına kanal ortalaması + standart sapması (referans yöntemler için).

    NOT: Kümülatif toplam farkı kullanmıyoruz. 1,1 milyon örnekte float32 kümülatif
    toplam ~10⁷ mertebesine çıkar ve komşu pencereler arasındaki fark (~10³) bu
    büyüklüklerin farkından hesaplandığında hassasiyet tamamen kaybolur. Bunun yerine
    pencereleri parçalar hâlinde float64'te doğrudan indirgiyoruz — tam doğru, ve
    bellek parça başına sınırlı.
    """
    st = np.arange(0, len(data) - W + 1, S) if starts is None else np.asarray(starts)
    n_win = len(st)
    D = data.shape[1]
    out = np.empty((n_win, 2 * D), dtype=np.float32)
    for a in range(0, n_win, chunk):
        b = min(a + chunk, n_win)
        blk = np.stack([data[s:s + W] for s in st[a:b]]).astype(np.float64)
        out[a:b, :D] = blk.mean(axis=1)
        out[a:b, D:] = blk.std(axis=1)
    return out


def win_mean(vec: np.ndarray, W: int, S: int,
             starts: np.ndarray | None = None) -> np.ndarray:
    """1-B sinyalin pencere içi ortalaması (float64, kayan pencere görünümüyle)."""
    v = np.ascontiguousarray(vec, dtype=np.float64)
    sw = np.lib.stride_tricks.sliding_window_view(v, W)
    return (sw[::S] if starts is None else sw[np.asarray(starts)]).mean(axis=1)


def sweep_scale(zr_val, zh_val, zr_test, zh_test, y, pct: float) -> tuple[list, dict]:
    """
    Verilen NEDENSEL ölçek altında w_kal'ı tarar.

    `z*_val` temiz doğrulama pencerelerinin, `z*_test` arıza enjekte edilmiş test
    kümesinin normalleştirilmiş skorlarıdır. Birleşik eşik HER ZAMAN yalnızca
    doğrulamadan (P`pct`) gelir — test kümesi eşiği hiçbir noktada göremez.
    """
    out = []
    for w_ in np.round(np.arange(0.0, 1.0001, 0.05), 2):
        thr_f = float(np.percentile(w_ * zr_val + (1 - w_) * zh_val, pct))
        d = (w_ * zr_test + (1 - w_) * zh_test) > thr_f
        tp = int((d & y).sum()); fp = int((d & ~y).sum()); fn = int((~d & y).sum())
        pr = tp / max(tp + fp, 1); rc = tp / max(tp + fn, 1)
        out.append({"w_kal": float(w_), "threshold": thr_f, "precision": pr,
                    "recall": rc, "f1": 2 * pr * rc / (pr + rc + 1e-12)})
    return out, max(out, key=lambda d: d["f1"])


def labels_at(starts: np.ndarray, n_samples: int, cfg, W: int,
              overlap: float) -> np.ndarray:
    """Pencere etiketi: pencerenin >= %overlap'i arıza maskesine denk geliyorsa anomali."""
    from inject_faults import fault_mask
    m = fault_mask(n_samples, cfg).astype(np.float64)
    cs = np.concatenate([[0.0], np.cumsum(m)])
    st = np.asarray(starts)
    return ((cs[st + W] - cs[st]) / W) >= overlap


# ─────────────────────────────── ana ───────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--residual-dir", default="residual_ae_model")
    ap.add_argument("--raw-dir", default="raw_ae_model")
    ap.add_argument("--residual-parquet", default=None)
    ap.add_argument("--raw-parquet", default=None)
    ap.add_argument("--out", default="fusion_results")
    ap.add_argument("--device", default=None)
    ap.add_argument("--batch-size", type=int, default=512)
    ap.add_argument("--limit-rows", type=int, default=None)
    ap.add_argument("--overlap", type=float, default=0.10)
    ap.add_argument("--ocsvm-subsample", type=int, default=8000)
    ap.add_argument("--no-baselines", action="store_true")
    ap.add_argument("--calibration", default="residual_calibration.json",
                    help="generate_residuals.py'nin yazdığı kalibrasyon; arıza "
                         "genliklerini Nm'den ölçüm uzayına çevirmek için")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    out_dir = Path(args.out); out_dir.mkdir(parents=True, exist_ok=True)
    dev = torch.device(args.device) if args.device else torch.device(
        "cuda" if torch.cuda.is_available() else "cpu")

    print("=" * 74)
    print("245.pdf — İKİLİ LSTM ÖZKODLAYICI BİRLEŞİMİ · DEĞERLENDİRME")
    print("=" * 74)
    print(f"  cihaz: {dev}" + (f" ({torch.cuda.get_device_name(0)})" if dev.type == "cuda" else ""))

    # ── modeller ──
    spaces = {}
    for space, mdir, pq in (("residual", args.residual_dir, args.residual_parquet),
                            ("raw", args.raw_dir, args.raw_parquet)):
        d = Path(mdir)
        meta = json.loads((d / "metadata.json").read_text(encoding="utf-8"))
        parquet = pq or meta.get("parquet") or PRESETS[space]["default_parquet"]
        model = build(meta).to(dev)
        model.load_state_dict(torch.load(d / "best_model.pt", map_location=dev,
                                         weights_only=True))
        model.eval()
        df = pd.read_parquet(parquet, columns=meta["feature_cols"])
        if args.limit_rows:
            df = df.iloc[:args.limit_rows]
        spaces[space] = {
            "meta": meta, "model": model, "cols": list(meta["feature_cols"]),
            "data": df.to_numpy(dtype=np.float32),
            "mean": np.array(meta["mean"], dtype=np.float32),
            "std": np.array(meta["std"], dtype=np.float32),
            "W": meta["window_size"], "S": meta["stride"],
            "thr": float(meta["threshold"]), "parquet": str(parquet),
        }
        del df
        print(f"  {space:9s}: {mdir:22s} {spaces[space]['data'].shape[0]:>9,} örnek × "
              f"{spaces[space]['data'].shape[1]:>2} kanal  θ={spaces[space]['thr']:.4f}")

    # Arıza genlikleri bildiride Nm; kanallarımız ölçüm (akım) uzayındaysa çevir.
    calib_a = None
    cp = Path(args.calibration)
    if cp.exists():
        cj = json.loads(cp.read_text(encoding="utf-8"))
        if cj.get("residual_units", "current") == "current":
            calib_a = cj.get("a")
    if calib_a:
        print(f"  kalibrasyon: {cp}  → arıza genlikleri Nm'den akım uzayına çevrilecek")
        print(f"    a = {[round(v, 4) for v in calib_a]}")
    else:
        print("  ⚠ kalibrasyon bulunamadı/Nm modunda → arıza genlikleri olduğu gibi uygulanacak")
    for sp in spaces.values():
        sp["scale"] = build_scale(sp["cols"], calib_a)

    nres, nraw = len(spaces["residual"]["data"]), len(spaces["raw"]["data"])
    if nres != nraw:
        n = min(nres, nraw)
        print(f"  ⚠ satır sayıları farklı ({nres:,} vs {nraw:,}) → {n:,}'e kırpılıyor")
        for s in spaces.values():
            s["data"] = s["data"][:n]
    N = len(spaces["residual"]["data"])
    W, S = spaces["residual"]["W"], spaces["residual"]["S"]
    if spaces["raw"]["W"] != W or spaces["raw"]["S"] != S:
        print("HATA: iki modelin pencere/adım değerleri farklı — birleşim yapılamaz.",
              file=sys.stderr)
        return 2
    # Pencereler koşu sınırlarına ve geçerlilik maskesine göre kurulur; iki model de
    # AYNI pencere kümesini kullanır, yoksa skorlar hizalanmaz.
    import pyarrow.parquet as _pq
    pq_path = spaces["residual"]["parquet"]
    have = set(_pq.ParquetFile(pq_path).schema.names)
    aux = [c for c in ("run_id", "valid") if c in have]
    GROUPS = VALID = None
    if aux:
        adf = pd.read_parquet(pq_path, columns=aux)
        if args.limit_rows:
            adf = adf.iloc[:args.limit_rows]
        GROUPS = adf["run_id"].to_numpy() if "run_id" in aux else None
        VALID = adf["valid"].to_numpy(bool) if "valid" in aux else None
        del adf
    STARTS = window_starts(N, W, S, GROUPS, VALID)
    n_win = len(STARTS)
    if GROUPS is None:
        print("  ⚠ 'run_id' yok → pencereler kopukluklara yayılabilir "
              "(prepare_dataset.py çalıştırılmamış).")
    else:
        print(f"  {len(np.unique(GROUPS)):,} kesintisiz koşu, geçerli örnek %{100*VALID.mean():.1f}"
              if VALID is not None else f"  {len(np.unique(GROUPS)):,} kesintisiz koşu")
    print(f"  senaryo başına pencere: {n_win:,}   toplam test: {4*n_win:,}")

    # ── senaryolar ──
    per_scn, y_all = {}, []
    sc_all = {"residual": [], "raw": []}
    t0 = time.time()
    for name, cfg in FAULTS.items():
        y = labels_at(STARTS, N, cfg, W, args.overlap)
        y_all.append(y)
        row = {"labels": y}
        for space, s in spaces.items():
            inj = inject(s["data"], s["cols"], cfg, space,
                         rng=np.random.default_rng(args.seed), scale=s["scale"])
            sc = score_windows(s["model"], inj, s["mean"], s["std"], W, S, dev,
                               args.batch_size, starts=STARTS)
            del inj
            sc_all[space].append(sc)
            row[space] = sc
        per_scn[name] = row
        print(f"  {cfg['tr']:<20} arıza penceresi {y.sum():>7,} (%{100*y.mean():.1f})  "
              f"[{time.time()-t0:.0f}s]")

    y = np.concatenate(y_all)
    S_res = np.concatenate(sc_all["residual"])
    S_raw = np.concatenate(sc_all["raw"])
    print(f"\n  TOPLAM test penceresi {len(y):,}  arızalı {y.sum():,} (%{100*y.mean():.1f})"
          f"   (bildiri 179.896 / 14.402 / %8,0)")

    # ── tekil modeller ──
    res = {"Kalıntı LSTM Özk.": metrics(y, S_res), "Ham LSTM Özk.": metrics(y, S_raw)}

    # ── birleşim ──
    Zr, Zh = minmax(S_res), minmax(S_raw)
    sweep = []
    for w in np.round(np.arange(0.0, 1.0001, 0.05), 2):
        f1, p, r, _ = best_f1(y, w * Zr + (1 - w) * Zh)
        sweep.append({"w_kal": float(w), "best_f1": f1, "precision": p, "recall": r})
    bw = max(sweep, key=lambda d: d["best_f1"])
    w = bw["w_kal"]
    res[f"Bir. A-Ort ({w:.2f}/{1-w:.2f})"] = metrics(y, w * Zr + (1 - w) * Zh)
    res["Bir. MAX"] = metrics(y, np.maximum(Zr, Zh))

    # OR: her iki modelden birinin KENDİ eşiğini aşması (ikili karar)
    det_r = S_res > spaces["residual"]["thr"]
    det_h = S_raw > spaces["raw"]["thr"]
    det_or = det_r | det_h
    tp = int((det_or & y).sum()); fp = int((det_or & ~y).sum()); fn = int((~det_or & y).sum())
    pr_or = tp / max(tp + fp, 1); rc_or = tp / max(tp + fn, 1)
    or_row = dict(metrics(y, np.maximum(Zr, Zh)))   # AUC/PR-AUC skor tabanlı (MAX ile aynı)
    or_row.update({"precision": pr_or, "recall": rc_or,
                   "f1_binary": 2 * pr_or * rc_or / (pr_or + rc_or + 1e-12)})
    res["Bir. OR"] = or_row

    # ── referans yöntemler ──
    extra_scores: dict[str, np.ndarray] = {}
    if not args.no_baselines:
        rd = spaces["residual"]
        cols = rd["cols"]
        ii_int = [cols.index(f"r_int_{j}") for j in range(1, 7)]
        ii_ext = [cols.index(f"r_ext_{j}") for j in range(1, 7)]
        split = int(N * rd["meta"].get("train_ratio", 0.8))
        n_tr_win = int((STARTS + W <= split).sum())

        norm_sc, if_sc, sv_sc = [], [], []
        feats_clean = win_feats(rd["data"], W, S, starts=STARTS)
        sca = StandardScaler().fit(feats_clean[:n_tr_win])
        print("\n  Referans yöntemler eğitiliyor (temiz kalıntı öznitelikleri)...")
        iso = IsolationForest(contamination=0.08, random_state=args.seed, n_jobs=-1)
        iso.fit(sca.transform(feats_clean[:n_tr_win]))
        sub = rng.choice(n_tr_win, size=min(args.ocsvm_subsample, n_tr_win), replace=False)
        svm = OneClassSVM(kernel="rbf", nu=0.05, gamma="scale")
        svm.fit(sca.transform(feats_clean[:n_tr_win][sub]))
        del feats_clean

        for name, cfg in FAULTS.items():
            inj = inject(rd["data"], cols, cfg, "residual",
                         rng=np.random.default_rng(args.seed), scale=rd["scale"])
            r_tot = inj[:, ii_int].astype(np.float64) + inj[:, ii_ext]  # r_top = r_ic + r_dis
            norm_sc.append(win_mean(np.linalg.norm(r_tot, axis=1), W, S, starts=STARTS))
            del r_tot
            f = sca.transform(win_feats(inj, W, S, starts=STARTS))
            if_sc.append(-iso.score_samples(f))
            sv_sc.append(-svm.decision_function(f))
            del inj, f

        extra_scores["Art. Norm Eşik"] = np.concatenate(norm_sc)
        extra_scores["Isolation Forest"] = np.concatenate(if_sc)
        extra_scores["One-Class SVM"] = np.concatenate(sv_sc)
        for k_, v_ in extra_scores.items():
            res[k_] = metrics(y, v_)

    # ── genel tablo ──
    paper = {"Kalıntı LSTM Özk.": (0.908, 0.695, 0.692), "Ham LSTM Özk.": (0.952, 0.761, 0.698),
             "Bir. A-Ort (0.95/0.05)": (0.980, 0.905, 0.859), "Bir. MAX": (0.976, 0.889, 0.824),
             "Bir. OR": (0.976, 0.889, 0.824), "Art. Norm Eşik": (0.849, 0.805, 0.707),
             "Isolation Forest": (0.688, 0.459, 0.547), "One-Class SVM": (0.815, 0.816, 0.760)}
    print("\n" + "=" * 74)
    print("TABLO II — GENEL PERFORMANS KARŞILAŞTIRMASI")
    print("=" * 74)
    print(f"  {'Model':<26}{'AUC':>8}{'PR-AUC':>9}{'BestF1':>9}   {'bildiri (AUC/PR/F1)':>24}")
    print("  " + "-" * 70)
    for k, v in res.items():
        p = paper.get(k)
        ptxt = f"{p[0]:.3f} / {p[1]:.3f} / {p[2]:.3f}" if p else "—"
        print(f"  {k:<26}{v['auc']:>8.3f}{v['pr_auc']:>9.3f}{v['best_f1']:>9.3f}   {ptxt:>24}")

    # ── arıza tipine göre ──
    per_type = {}
    print("\n" + "=" * 74)
    print("TABLO III — ARIZA TİPİNE GÖRE BestF1")
    print("=" * 74)
    paper3 = {"motor_kaymasi": (0.597, 0.605, 0.588), "carpisma": (0.916, 0.933, 0.920),
              "gizyazar_hatasi": (0.986, 0.530, 0.977), "sensor_gurultusu": (0.272, 1.000, 0.992)}
    # ÖNEMLİ: birleşim skoru, tüm test kümesi üzerinden hesaplanan GLOBAL min–max ile
    # üretilir; senaryo bazında yeniden normalleştirmek ağırlıkların anlamını bozar.
    # (Tekil model sütunları monoton dönüşüme duyarsız olduğu için etkilenmez.)
    Zfus = w * Zr + (1 - w) * Zh
    print(f"  {'Arıza Tipi':<20}{'Kalıntı':>10}{'Ham':>9}{'Birleşim':>11}   {'bildiri':>22}")
    print("  " + "-" * 70)
    for i, (name, cfg) in enumerate(FAULTS.items()):
        sl = slice(i * n_win, (i + 1) * n_win)
        yy = per_scn[name]["labels"]
        a = best_f1(yy, per_scn[name]["residual"])[0]
        b = best_f1(yy, per_scn[name]["raw"])[0]
        c = best_f1(yy, Zfus[sl])[0]
        per_type[name] = {"residual": a, "raw": b, "fusion": c}
        p = paper3[name]
        print(f"  {cfg['tr']:<20}{a:>10.3f}{b:>9.3f}{c:>11.3f}   "
              f"{p[0]:.3f} / {p[1]:.3f} / {p[2]:.3f}")

    # ── tamamlayıcılık ──
    fw = y
    only_r = int((det_r & ~det_h & fw).sum())
    only_h = int((det_h & ~det_r & fw).sum())
    both = int((det_r & det_h & fw).sum())
    none_ = int((~det_r & ~det_h & fw).sum())
    nf = int(fw.sum())
    comp = {"n_fault_windows": nf,
            "only_residual": only_r, "only_residual_pct": 100 * only_r / max(nf, 1),
            "only_raw": only_h, "only_raw_pct": 100 * only_h / max(nf, 1),
            "both": both, "both_pct": 100 * both / max(nf, 1),
            "neither": none_, "neither_pct": 100 * none_ / max(nf, 1),
            "recall_residual": float((det_r & fw).sum() / max(nf, 1)),
            "recall_raw": float((det_h & fw).sum() / max(nf, 1)),
            "recall_or": float((det_or & fw).sum() / max(nf, 1))}
    print("\n" + "=" * 74)
    print("TAMAMLAYICILIK ANALİZİ (pencere düzeyinde, kendi eşikleriyle)")
    print("=" * 74)
    print(f"  {nf:,} arıza penceresinden:")
    print(f"    yalnızca Kalıntı : {only_r:>7,}  %{comp['only_residual_pct']:.1f}   (bildiri %24,1)")
    print(f"    yalnızca Ham     : {only_h:>7,}  %{comp['only_raw_pct']:.1f}   (bildiri %25,9)")
    print(f"    her ikisi        : {both:>7,}  %{comp['both_pct']:.1f}")
    print(f"    hiçbiri          : {none_:>7,}  %{comp['neither_pct']:.1f}")
    print(f"  geri çağırma — Kalıntı {comp['recall_residual']:.3f} (bildiri 0,595) · "
          f"Ham {comp['recall_raw']:.3f} (0,613) · OR {comp['recall_or']:.3f} (0,854)")

    print(f"\n  Ağırlık taraması: en iyi w_kal = {w:.2f} → BestF1 {bw['best_f1']:.3f}"
          f"   (bildiri w_kal=0,95 → 0,859)")

    # ── ÇEVRİMİÇİ yapılandırma ────────────────────────────────────────────
    # Bildirinin min–max normalleştirmesi sınırlarını ARIZA ENJEKTE EDİLMİŞ test
    # kümesinden alır. Canlıda o maksimumlar hiç görülmez; tüm çalışma aralığı
    # ~1e-4'e sıkışır ve birleşik eşik her iki tekil eşikten de KATI hale gelir
    # (replay_detector.py ile ölçüldü: motor kayması ve gizyazar hiç tespit edilmiyor).
    #
    # Bu yüzden İKİ nedensel ölçek yarıştırılır — ikisi de bildirinin Denk. 4
    # yapısını (ağırlıklı ortalama) aynen korur, yalnız normalleştirmenin
    # sınırlarını geleceği görmeyen bir kümeden alır:
    #
    #   theta       z = S / θ                  — modelin kendi doğrulama-P97 eşiğine oran
    #   minmax_val  z = (S − min)/(max − min)  — BİLDİRİNİN formülü, sınırlar TEMİZ
    #                                            doğrulama pencerelerinden (D1 sapmasını
    #                                            "formül farklı" olmaktan çıkarıp yalnız
    #                                            "sınır kümesi farklı"ya indirger)
    #
    # İkisi de aynı w taramasından ve aynı P97 birleşik eşik kuralından geçer;
    # kazanan `fusion_config.json`'a yazılır, düğüm her ikisini de okuyabilir.
    print("\n" + "=" * 74)
    print("ÇEVRİMİÇİ YAPILANDIRMA (nedensel ölçekler yarıştırılıyor)")
    print("=" * 74)
    thr_r, thr_h = spaces["residual"]["thr"], spaces["raw"]["thr"]
    S_res_clean = score_windows(spaces["residual"]["model"], spaces["residual"]["data"],
                                spaces["residual"]["mean"], spaces["residual"]["std"],
                                W, S, dev, args.batch_size, starts=STARTS)
    S_raw_clean = score_windows(spaces["raw"]["model"], spaces["raw"]["data"],
                                spaces["raw"]["mean"], spaces["raw"]["std"],
                                W, S, dev, args.batch_size, starts=STARTS)
    split_row = int(N * spaces["residual"]["meta"].get("train_ratio", 0.8))
    val_w = (STARTS + W) > split_row
    print(f"  temiz doğrulama penceresi: {int(val_w.sum()):,}")
    rv, hv = S_res_clean[val_w], S_raw_clean[val_w]
    pct = spaces["residual"]["meta"]["threshold_percentile"]

    # Her ölçek bir AFFİN dönüşümdür: z = (S − lo)/span. Düğüm de tam olarak bunu
    # uygular, yani ölçek eklemek düğümde kod değişikliği gerektirmez.
    scales = {
        "theta": {"residual": {"lo": 0.0, "span": float(thr_r)},
                  "raw": {"lo": 0.0, "span": float(thr_h)}},
        "minmax_val": {"residual": {"lo": float(rv.min()),
                                    "span": float(rv.max() - rv.min())},
                       "raw": {"lo": float(hv.min()),
                               "span": float(hv.max() - hv.min())}},
    }

    cand = {}
    for nm, sc in scales.items():
        zr_v = (rv - sc["residual"]["lo"]) / sc["residual"]["span"]
        zh_v = (hv - sc["raw"]["lo"]) / sc["raw"]["span"]
        zr_t = (S_res - sc["residual"]["lo"]) / sc["residual"]["span"]
        zh_t = (S_raw - sc["raw"]["lo"]) / sc["raw"]["span"]
        sw, b_ = sweep_scale(zr_v, zh_v, zr_t, zh_t, y, pct)
        fa_ = float(((b_["w_kal"] * zr_v + (1 - b_["w_kal"]) * zh_v) > b_["threshold"]).mean())
        cand[nm] = {"scale": sc, "sweep": sw, "best": b_, "clean_val_false_alarm": fa_,
                    "z": (zr_v, zh_v)}

    for nm, c in cand.items():
        sc, b_ = c["scale"], c["best"]
        print(f"\n  ── {nm} ──  kalıntı z=(S−{sc['residual']['lo']:.4g})/{sc['residual']['span']:.4g}"
              f"   ham z=(S−{sc['raw']['lo']:.4g})/{sc['raw']['span']:.4g}")
        print(f"  {'w_kal':>7}{'θ_birleşik':>13}{'kesinlik':>11}{'geri çağırma':>14}{'F1':>8}")
        print("  " + "-" * 55)
        for r_ in c["sweep"]:
            if r_["w_kal"] in (0.0, 0.25, 0.5, 0.75, 0.9, 0.95, 1.0) or r_ is b_:
                mark = "  ←" if r_ is b_ else ""
                print(f"  {r_['w_kal']:>7.2f}{r_['threshold']:>13.4f}{r_['precision']:>11.3f}"
                      f"{r_['recall']:>14.3f}{r_['f1']:>8.3f}{mark}")
        print(f"  en iyi: w_kal={b_['w_kal']:.2f} F1 {b_['f1']:.3f}  "
              f"temiz doğrulamada yanlış alarm %{100*c['clean_val_false_alarm']:.1f}")

    # Kazanan: F1. Berabere kalırsa `theta` — sınırları tek bir sayıya (θ) dayandığı
    # için doğrulama kümesinin uç değerlerine `minmax_val`'dan daha az duyarlı.
    norm = max(cand, key=lambda k: (cand[k]["best"]["f1"], k == "theta"))
    c = cand[norm]
    bo, fa = c["best"], c["clean_val_false_alarm"]
    other = [k for k in cand if k != norm][0]
    print(f"\n  KAZANAN ÖLÇEK: {norm}  (F1 {c['best']['f1']:.3f} vs "
          f"{cand[other]['best']['f1']:.3f} [{other}])")
    print(f"  Seçilen: w_kal={bo['w_kal']:.2f}  θ_birleşik={bo['threshold']:.4f}  "
          f"→ F1 {bo['f1']:.3f} (kesinlik {bo['precision']:.3f}, geri çağırma {bo['recall']:.3f})")
    print(f"  temiz doğrulamada yanlış alarm: %{100*fa:.1f}  (P97 kuralı gereği ~%3 beklenir)")
    online = c["sweep"]

    # ── kayıt ──
    # Düğüm `scale` bloğunu okur: z = (S − lo)/span, sonra Denk. 4 aynen uygulanır.
    fusion_norm = {
        # Çevrimiçi düğümün kullandığı bölüm. İkisi de nedensel; ayrıntı yukarıda.
        "norm": norm,
        "w_kal": float(bo["w_kal"]), "w_ham": float(1 - bo["w_kal"]),
        "fused_threshold": float(bo["threshold"]),
        "scale": c["scale"],
        "residual": {"threshold": float(thr_r)},
        "raw": {"threshold": float(thr_h)},
        "online_sweep": online,
        "expected": {"precision": bo["precision"], "recall": bo["recall"], "f1": bo["f1"],
                     "clean_val_false_alarm": fa},
        # Yarışan ölçeklerin tam kaydı — raporun D1 tablosu buradan üretiliyor.
        "norm_candidates": {k: {"scale": v["scale"], "best": v["best"],
                                "clean_val_false_alarm": v["clean_val_false_alarm"],
                                "sweep": v["sweep"]} for k, v in cand.items()},
        # Bildirinin ORİJİNAL min–max normalleştirmesi — yalnız çevrimdışı tablo
        # karşılaştırması için saklanıyor; sınırları arıza enjekte edilmiş kümeden
        # geldiği için canlıda KULLANILMAMALI.
        "paper_minmax": {
            "residual": {"min": float(S_res.min()), "max": float(S_res.max())},
            "raw": {"min": float(S_raw.min()), "max": float(S_raw.max())},
            "w_kal": float(w), "w_ham": float(1 - w),
        },
    }
    (out_dir / "fusion_config.json").write_text(
        json.dumps(fusion_norm, indent=2), encoding="utf-8")

    payload = {"n_test_windows": int(len(y)), "n_fault_windows": int(y.sum()),
               "fault_ratio": float(y.mean()), "window": W, "stride": S,
               "overlap_rule": args.overlap, "best_w_kal": float(w),
               "fusion_config": fusion_norm,
               "overall": res, "per_fault_type": per_type,
               "weight_sweep": sweep, "complementarity": comp,
               "paper_reference": {"table2": paper, "table3": paper3}}
    (out_dir / "fusion_results.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    pd.DataFrame([{"Model": k, "AUC": round(v["auc"], 3), "PR-AUC": round(v["pr_auc"], 3),
                   "BestF1": round(v["best_f1"], 3)} for k, v in res.items()]
                 ).to_csv(out_dir / "table2_overall.csv", index=False, encoding="utf-8")
    pd.DataFrame([{"Arıza Tipi": FAULTS[k]["tr"], "Kalıntı": round(v["residual"], 3),
                   "Ham": round(v["raw"], 3), "Birleşim": round(v["fusion"], 3)}
                  for k, v in per_type.items()]
                 ).to_csv(out_dir / "table3_per_fault.csv", index=False, encoding="utf-8")
    np.savez_compressed(out_dir / "scores.npz", y=y, s_residual=S_res, s_raw=S_raw,
                        z_residual=Zr, z_raw=Zh)

    # ── figürler ──
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        curves = {"Kalıntı LSTM Özk.": S_res, "Ham LSTM Özk.": S_raw,
                  f"Bir. A-Ort ({w:.2f}/{1-w:.2f})": Zfus,
                  "Bir. MAX": np.maximum(Zr, Zh), **extra_scores}
        fig, ax = plt.subplots(figsize=(6.5, 5.5))
        for k, s in curves.items():
            fpr, tpr, _ = roc_curve(y, s)
            ax.plot(fpr, tpr, lw=1.8, label=f"{k} ({res[k]['auc']:.3f})")
        ax.plot([0, 1], [0, 1], "k--", lw=.8)
        ax.set_xlabel("Yanlış Pozitif Oranı"); ax.set_ylabel("Doğru Pozitif Oranı")
        ax.set_title("Şekil 2 — ROC eğrileri"); ax.legend(fontsize=8, loc="lower right")
        ax.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(out_dir / "fig2_roc.png", dpi=150); plt.close(fig)

        fig, ax = plt.subplots(figsize=(7, 4.2))
        ws = [d["w_kal"] for d in sweep]; fs = [d["best_f1"] for d in sweep]
        ax.plot(ws, fs, "o-", lw=1.8, ms=4)
        ax.axvline(w, color="r", ls="--", lw=1, label=f"en iyi w_kal={w:.2f} ({bw['best_f1']:.3f})")
        ax.axhline(0.84, color="g", ls=":", lw=1, label="bildiri kararlı bölge eşiği 0,84")
        ax.set_xlabel("w_kal"); ax.set_ylabel("BestF1")
        ax.set_title("Şekil 4 — Birleşim ağırlık duyarlılığı")
        ax.legend(fontsize=8); ax.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(out_dir / "fig4_weight_sweep.png", dpi=150); plt.close(fig)

        fig, ax = plt.subplots(figsize=(6.5, 4.2))
        lb = ["yalnız Kalıntı", "yalnız Ham", "her ikisi", "hiçbiri"]
        vl = [comp["only_residual_pct"], comp["only_raw_pct"], comp["both_pct"],
              comp["neither_pct"]]
        bars = ax.bar(lb, vl, color=["indianred", "steelblue", "mediumpurple", "gray"])
        for b_, v_ in zip(bars, vl):
            ax.text(b_.get_x() + b_.get_width() / 2, v_ + .5, f"%{v_:.1f}",
                    ha="center", fontsize=9)
        ax.set_ylabel("arıza penceresi payı [%]")
        ax.set_title("Şekil 3 — Pencere düzeyinde tamamlayıcılık")
        ax.grid(alpha=.3, axis="y")
        fig.tight_layout(); fig.savefig(out_dir / "fig3_complementarity.png", dpi=150)
        plt.close(fig)

        fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=False)
        for axx, (name, cfg) in zip(axes, FAULTS.items()):
            d = per_scn[name]
            xx = np.arange(len(d["labels"]))
            axx.plot(xx, minmax(d["residual"]), lw=.5, label="Kalıntı", color="indianred")
            axx.plot(xx, minmax(d["raw"]), lw=.5, label="Ham", color="steelblue", alpha=.8)
            axx.fill_between(xx, 0, 1, where=d["labels"], color="orange", alpha=.25,
                             label="arıza bölgesi", lw=0)
            axx.set_ylabel("skor"); axx.set_title(cfg["tr"], fontsize=10)
            axx.legend(fontsize=7, loc="upper left"); axx.grid(alpha=.3)
        axes[-1].set_xlabel("pencere")
        fig.suptitle("Şekil 3 — Dört arıza senaryosunda zamansal anomali tespiti", fontsize=12)
        fig.tight_layout(); fig.savefig(out_dir / "fig3_temporal.png", dpi=140); plt.close(fig)

        print(f"\n→ {out_dir}/ fig2_roc.png · fig3_complementarity.png · "
              f"fig3_temporal.png · fig4_weight_sweep.png")
    except Exception as e:
        print(f"  (grafik atlandı: {e})")

    print(f"→ {out_dir}/fusion_results.json · table2_overall.csv · table3_per_fault.csv")
    print(f"\nToplam süre: {time.time()-t0:.0f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
