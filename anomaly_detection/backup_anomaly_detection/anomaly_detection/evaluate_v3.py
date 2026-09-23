#!/usr/bin/env python3
"""
evaluate_v3.py
==============
KOŞU-AYRIK değerlendirme protokolü.

Neden yeni bir betik
--------------------
`evaluate_fusion.py` bildirinin protokolünü birebir yeniden üretir ve erratum için
öyle KALMALIDIR. Ama o protokol bir genelleme ölçümü değildir; üç ayrı kusuru var
ve üçü de burada kapatılıyor:

  1. Test kümesi TÜM pencerelerden kuruluyordu (4 × 10.422 = 41.688), oysa eğitim
     ilk 8.294 pencereyi kullanmıştı → değerlendirmenin %79,6'sı eğitim penceresi.
     Ölçüldü: temiz pencerelerde yeniden yapılanma hatası eğitim bölgesinde 0,007,
     dışında 0,486 — 70 kat. Her iki model de AYNI oranda bozuluyor.
  2. Arıza maskesi tüm veri setinde TEK bitişik bloktu; senaryo başına etkin
     bağımsız olay sayısı 1, toplamda 4. Üstelik maskenin sabit konumu arıza tipini
     kaydın konumuyla eşliyordu: %92–100'deki gizyazar hatası yalnız son dilime,
     diğer üçü yalnız eğitim bölgesine düşüyordu. "Görülmemiş pencerelerde modeller
     rol değiştiriyor" bulgusu bu yüzden sızıntıyı değil ARIZA TİPİNİ ölçüyordu.
  3. Birleşim ağırlığı w, test kümesindeki F1'in argmax'ı olarak seçiliyordu
     (bildiri: "en yüksek değer w_kal=0,95'te"). Eşik nedensel yapılmıştı ama
     ağırlık değildi.

Bu betikte:
  · koşular make_splits.py ile eğitim/doğrulama/test olarak ayrılır,
  · arızalar HER bölmenin kendi koşularına, koşu başına ayrı ayrı enjekte edilir,
  · lo/span, P97 ve AĞIRLIK yalnız DOĞRULAMA koşularından belirlenir,
  · test kümesine bir kez dokunulur.

Kullanım
--------
    python3 evaluate_v3.py --residual-dir residual_ae_v3 --raw-dir raw_ae_v3 \\
        --parquet ur10e_features_fric.parquet --splits splits.json --out fusion_v3
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
from sklearn.metrics import roc_auc_score, average_precision_score, precision_recall_curve
from sklearn.ensemble import IsolationForest
from sklearn.svm import OneClassSVM
from sklearn.preprocessing import StandardScaler

sys.path.insert(0, str(Path(__file__).resolve().parent))
from models import PRESETS, WindowDataset, build, window_starts  # noqa: E402
from inject_faults import FAULTS, inject, window_labels, build_scale  # noqa: E402


# ───────────────────────── ölçütler ─────────────────────────

def best_f1(y, s):
    p, r, t = precision_recall_curve(y, s)
    f = 2 * p * r / (p + r + 1e-12)
    i = int(np.nanargmax(f))
    return float(f[i]), float(p[i]), float(r[i]), float(t[min(i, len(t) - 1)])


def metrics(y, s) -> dict:
    """
    Sıralama kalitesi (AUC, PR-AUC) + eşikten bağımsız en iyi F1.

    BestF1 tüm eşikler üzerinden maksimumdur, yani tahmin etmesi gereken etiketleri
    kullanır ve çevrimiçi karşılığı yoktur. Yalnız bildiriyle karşılaştırılabilirlik
    için raporlanır; çalışma noktası ölçümleri `operating()` ile ayrı verilir.
    """
    f1, p, r, thr = best_f1(y, s)
    return {"auc": float(roc_auc_score(y, s)),
            "pr_auc": float(average_precision_score(y, s)),
            "best_f1": f1, "bf1_precision": p, "bf1_recall": r, "bf1_threshold": thr}


def operating(y, s, thr) -> dict:
    """DONDURULMUŞ eşikteki gerçek çalışma noktası."""
    d = s > thr
    tp = int((d & y).sum()); fp = int((d & ~y).sum())
    fn = int((~d & y).sum()); tn = int((~d & ~y).sum())
    pr = tp / max(tp + fp, 1); rc = tp / max(tp + fn, 1)
    return {"tp": tp, "fp": fp, "fn": fn, "tn": tn, "threshold": float(thr),
            "precision": pr, "recall": rc, "f1": 2 * pr * rc / (pr + rc + 1e-12),
            "specificity": tn / max(tn + fp, 1)}


def score_windows(model, data, mean, std, W, S, dev, starts, bs=512) -> np.ndarray:
    ds = WindowDataset((data - mean) / std, W, S)
    ds.starts = starts; ds.n_windows = len(starts)
    out = []
    with torch.no_grad():
        for b in DataLoader(ds, batch_size=bs, shuffle=False):
            b = b.to(dev, non_blocking=True)
            out.append(((model(b) - b) ** 2).mean(dim=(1, 2)).float().cpu().numpy())
    return np.concatenate(out)


def win_feats(data, W, S, starts, chunk=4096) -> np.ndarray:
    """Pencere başına kanal ortalaması + std (referans yöntemler için, float64)."""
    st = np.asarray(starts); D = data.shape[1]
    out = np.empty((len(st), 2 * D), dtype=np.float32)
    for a in range(0, len(st), chunk):
        b = min(a + chunk, len(st))
        blk = np.stack([data[s:s + W] for s in st[a:b]]).astype(np.float64)
        out[a:b, :D] = blk.mean(axis=1)
        out[a:b, D:] = blk.std(axis=1)
    return out


def win_mean(vec, W, S, starts) -> np.ndarray:
    v = np.ascontiguousarray(vec, dtype=np.float64)
    return np.lib.stride_tricks.sliding_window_view(v, W)[np.asarray(starts)].mean(axis=1)


# ───────────────────── koşu → dilim eşlemesi ─────────────────────

def window_motion(qd_abs_max: np.ndarray, starts: np.ndarray, W: int) -> np.ndarray:
    """
    Pencere başına hareket göstergesi: pencere içindeki en yüksek eklem hızı.

    Çevrimiçi düğüm de tam olarak bunu yapar (`_qd_peak`, karar penceresi içindeki
    maks |q̇|), yani eşik seçimi çevrimdışı ve çevrimiçi aynı büyüklüğe bakar.
    """
    cm = np.maximum.accumulate  # kayan maksimum yerine pencere görünümü (bellek ucuz)
    sw = np.lib.stride_tricks.sliding_window_view(qd_abs_max, W)
    return sw[np.asarray(starts)].max(axis=1)


def run_slices(run_ids: np.ndarray) -> dict[int, tuple[int, int]]:
    """run_id → (başlangıç indeksi, uzunluk). Koşular bitişik varsayılır."""
    edges = np.concatenate([[0], np.flatnonzero(np.diff(run_ids) != 0) + 1, [len(run_ids)]])
    return {int(run_ids[a]): (int(a), int(b - a)) for a, b in zip(edges[:-1], edges[1:])}


def split_starts(all_starts: np.ndarray, W: int, sl: dict[int, tuple[int, int]],
                 runs: list[int]) -> np.ndarray:
    """Yalnız verilen koşuların İÇİNDE tamamen kalan pencere başlangıçları."""
    keep = np.zeros(len(all_starts), bool)
    for r in runs:
        a, L = sl[r]
        keep |= (all_starts >= a) & (all_starts + W <= a + L)
    return all_starts[keep]


def host_segments(sl: dict[int, tuple[int, int]], runs: list[int],
                  min_len: int) -> list[tuple[int, int]]:
    """Arıza enjekte edilebilecek koşuların (başlangıç, uzunluk) listesi."""
    return [sl[r] for r in runs if sl[r][1] >= min_len]


# ─────────────────────────────── ana ───────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description="Koşu-ayrık değerlendirme (v3)")
    ap.add_argument("--residual-dir", default="residual_ae_v3")
    ap.add_argument("--raw-dir", default="raw_ae_v3")
    ap.add_argument("--parquet", default=None,
                    help="varsayılan: modelin metadata'sındaki parquet")
    ap.add_argument("--splits", default="splits.json")
    ap.add_argument("--out", default="fusion_v3")
    ap.add_argument("--device", default=None)
    ap.add_argument("--batch-size", type=int, default=512)
    ap.add_argument("--overlap", type=float, default=0.10)
    ap.add_argument("--ocsvm-subsample", type=int, default=8000)
    ap.add_argument("--no-baselines", action="store_true")
    ap.add_argument("--calibration", default="residual_calibration_fric.json")
    ap.add_argument("--motion-qd-min", type=float, default=0.02,
                    help="pencereyi 'hareketli' sayan en düşük tepe |q̇| [rad/s]. "
                         "detector.py'deki motion_qd_min ile AYNI olmalı.")
    ap.add_argument("--w-res", type=float, default=0.95,
                    help="birleşim ağırlığı. VARSAYILAN: bildirinin değeri, ÖNSEL "
                         "olarak sabitlenir ve hiçbir değerlendirme kümesinden "
                         "seçilmez. Bildiri w_kal=0,95'i test kümesindeki BestF1'in "
                         "argmax'ı olarak seçmişti; sabitleyerek o seçim sızıntısı "
                         "tamamen ortadan kalkar. Tarama yine yapılır ama DUYARLILIK "
                         "ANALİZİ olarak raporlanır, seçim olarak değil.")
    ap.add_argument("--select-w", action="store_true",
                    help="ağırlığı doğrulamada seç (önsel sabit yerine). Karşılaştırma "
                         "için; varsayılan değil.")
    ap.add_argument("--inject-space", choices=["native", "measurement"], default="native",
                    help="native: arıza her temsil uzayına AYRI genlikle konur "
                         "(bildiriden miras; kalıntı genliği elle seçilmiş bir sayı). "
                         "measurement: arıza yalnız ÖLÇÜLEN kanallara konur ve kalıntı "
                         "hattın kendisiyle yeniden hesaplanır — make_injected_features.py "
                         "ile önceden üretilmiş olmalı.")
    ap.add_argument("--injected-dir", default="injected_{split}",
                    help="--inject-space measurement için; {split} yer tutucusu")
    ap.add_argument("--threshold-rule", choices=["global_p97", "regime_p97"],
                    default="global_p97",
                    help="dondurulacak eşik kuralı. Offline tablo için global_p97 "
                         "(eşleşen görev döngüsünde daha iyi); saha dağıtımı için "
                         "regime_p97 (görev döngüsü kayınca yanlış alarmı öngörülebilir). "
                         "İkisi de her koşuda hesaplanır ve config'e yazılır.")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    out_dir = Path(args.out); out_dir.mkdir(parents=True, exist_ok=True)
    dev = torch.device(args.device) if args.device else torch.device(
        "cuda" if torch.cuda.is_available() else "cpu")

    print("=" * 78)
    print("KOŞU-AYRIK DEĞERLENDİRME (v3)")
    print("=" * 78)
    print(f"  cihaz: {dev}")

    SP = json.loads(Path(args.splits).read_text(encoding="utf-8"))
    print(f"  bölme: {args.splits}  "
          f"eğitim {len(SP['train'])} · doğrulama {len(SP['val'])} · test {len(SP['test'])} koşu")

    # ── modeller ──
    spaces = {}
    for space, mdir in (("residual", args.residual_dir), ("raw", args.raw_dir)):
        d = Path(mdir)
        meta = json.loads((d / "metadata.json").read_text(encoding="utf-8"))
        if meta.get("split_source") != str(args.splits):
            print(f"  ⚠ {mdir}: metadata split_source={meta.get('split_source')!r} "
                  f"≠ {args.splits!r}. Model başka bir bölmeyle eğitilmişse bu "
                  f"değerlendirme koşu-ayrık DEĞİLDİR.")
        model = build(meta).to(dev)
        model.load_state_dict(torch.load(d / "best_model.pt", map_location=dev,
                                         weights_only=True))
        model.eval()
        spaces[space] = {"meta": meta, "model": model, "cols": list(meta["feature_cols"]),
                         "mean": np.array(meta["mean"], dtype=np.float32),
                         "std": np.array(meta["std"], dtype=np.float32),
                         "thr": float(meta["threshold"])}
        print(f"  {space:9s}: {mdir:20s} θ={spaces[space]['thr']:.4f}  "
              f"{meta['features']} kanal")

    pq_path = args.parquet or spaces["residual"]["meta"]["parquet"]
    W = spaces["residual"]["meta"]["window_size"]
    S = spaces["residual"]["meta"]["stride"]
    if spaces["raw"]["meta"]["window_size"] != W or spaces["raw"]["meta"]["stride"] != S:
        print("HATA: pencere/adım farklı — birleşim hizalanmaz.", file=sys.stderr)
        return 2

    need = sorted(set(spaces["residual"]["cols"]) | set(spaces["raw"]["cols"]))
    qd_cols = [f"qd_{j}" for j in range(1, 7)]
    df = pd.read_parquet(pq_path, columns=sorted(set(need) | set(qd_cols))
                         + ["run_id", "valid"])
    RUN = df["run_id"].to_numpy()
    VALID = df["valid"].to_numpy(bool)
    QDMAX = np.abs(df[qd_cols].to_numpy(np.float64)).max(axis=1)
    for sp in spaces.values():
        sp["data"] = df[sp["cols"]].to_numpy(dtype=np.float32)
    N = len(df); del df
    print(f"  veri  : {pq_path}  {N:,} örnek")

    SL = run_slices(RUN)
    ALL = window_starts(N, W, S, RUN, VALID)
    ST = {k: split_starts(ALL, W, SL, SP[k]) for k in ("train", "val", "test")}
    SEG = {k: host_segments(SL, SP[k], SP["min_fault_len"]) for k in ("val", "test")}
    MOV = {k: window_motion(QDMAX, ST[k], W) > args.motion_qd_min
           for k in ("train", "val", "test")}
    print(f"  pencere: eğitim {len(ST['train']):,} · doğrulama {len(ST['val']):,} · "
          f"test {len(ST['test']):,}")
    print(f"  hareketli pencere payı: doğrulama %{100*MOV['val'].mean():.0f} · "
          f"test %{100*MOV['test'].mean():.0f}   (eşik |q̇| > {args.motion_qd_min})")
    print(f"  arıza taşıyıcı koşu: doğrulama {len(SEG['val'])} · test {len(SEG['test'])}"
          f"  → senaryo başına bağımsız olay: test {len(SEG['test'])}")

    # Arıza genlikleri bildiride Nm; kanallar akım uzayındaysa çevrilir.
    calib_a = None
    cp = Path(args.calibration)
    if cp.exists():
        cj = json.loads(cp.read_text(encoding="utf-8"))
        if cj.get("residual_units", "current") == "current":
            calib_a = cj.get("a")
    for sp in spaces.values():
        sp["scale"] = build_scale(sp["cols"], calib_a)

    # Sürtünme terimi q̇'nin fonksiyonu. Hiçbir senaryo q̇ kanalına dokunmuyorsa
    # terim bir arıza imzasını SOĞURAMAZ. Varsaymak yerine burada kontrol edilir.
    touched = sorted({c for f in FAULTS.values() for sp in ("raw", "residual")
                      for c in f[sp]["cols"]})
    qd_hit = [c for c in touched if c.startswith("qd_")]
    print(f"  enjeksiyon uzayı: {args.inject_space}"
          + ("  (arıza ölçüm kanallarına konur, kalıntı YENİDEN HESAPLANIR)"
             if args.inject_space == "measurement"
             else "  (kalıntı genliği elle verilir — bildiriden miras)"))
    print(f"  arızaların dokunduğu kanallar: {', '.join(touched)}")
    print(f"  sürtünme soğurma riski: {'VAR — ' + str(qd_hit) if qd_hit else 'yok (hiçbir senaryo q̇ kanalına dokunmuyor)'}")

    # ── temiz skorlar: dağılım kayması tanısı ────────────────────────────
    # Modellerin TEMİZ veride bölmeler arası davranışı. Eski hatta bu ölçüm
    # eğitim bölgesinde 0,007, dışında 0,486 veriyordu (70 kat) — ve iki model de
    # aynı oranda bozuluyordu, yani "ham model ezberliyor, kalıntı genelliyor"
    # yorumu yanlıştı. Koşu-ayrık bölmede oranın 1'e yakınsaması beklenir; hâlâ
    # büyükse veri seti tek bir görev profilinden ibarettir ve bunu eğitimden
    # SONRA değil raporda görmek gerekir.
    print("\n" + "=" * 78)
    print("TANI — TEMİZ PENCERELERDE YENİDEN YAPILANMA HATASI (arıza YOK)")
    print("=" * 78)
    clean = {}
    for space, sp in spaces.items():
        clean[space] = {k: score_windows(sp["model"], sp["data"], sp["mean"], sp["std"],
                                         W, S, dev, ST[k], args.batch_size)
                        for k in ("train", "val", "test")}
    print(f"  {'model':<10}{'eğitim':>12}{'doğrulama':>13}{'test':>12}"
          f"{'test/eğitim':>14}")
    print("  " + "-" * 62)
    drift = {}
    for space in ("residual", "raw"):
        m = {k: float(np.median(clean[space][k])) for k in ("train", "val", "test")}
        ratio = m["test"] / max(m["train"], 1e-12)
        drift[space] = {"median": m, "test_over_train": ratio}
        print(f"  {space:<10}{m['train']:>12.4f}{m['val']:>13.4f}{m['test']:>12.4f}"
              f"{ratio:>13.1f}x")
    print("  (medyan; eski hatta bu oran her iki model için de ~70x idi)")

    # ── DOĞRULAMA: ölçek, ağırlık ve eşik burada donar ───────────────────
    print("\n" + "=" * 78)
    print("DOĞRULAMA — ölçek, ağırlık ve eşik seçimi (test kümesi görülmez)")
    print("=" * 78)
    pct = spaces["residual"]["meta"]["threshold_percentile"]

    inj_cache: dict[str, dict] = {}

    def _measurement_scores(split: str, name: str) -> dict[str, np.ndarray]:
        """Önceden üretilmiş, ölçüm uzayında enjekte edilmiş öznitelikleri skorlar."""
        d = Path(args.injected_dir.format(split=split))
        f = d / f"features_{name}.parquet"
        if not f.exists():
            raise FileNotFoundError(
                f"{f} yok. Önce: python3 make_injected_features.py --split {split}")
        out = {}
        for space, sp in spaces.items():
            arr = pd.read_parquet(f, columns=sp["cols"]).to_numpy(dtype=np.float32)
            out[space] = score_windows(sp["model"], arr, sp["mean"], sp["std"],
                                       W, S, dev, ST[split], args.batch_size)
            del arr
        return out

    def injected(split: str) -> tuple[np.ndarray, dict[str, np.ndarray], dict]:
        """Bölmenin koşularına arıza enjekte edip skorlar. (y, skorlar, senaryo başı)"""
        ys, sc = [], {"residual": [], "raw": []}
        per = {}
        for name, cfg in FAULTS.items():
            y = window_labels(N, cfg, W, S, args.overlap, SEG[split], ST[split])
            ys.append(y)
            row = {"labels": y}
            if args.inject_space == "measurement":
                row.update(_measurement_scores(split, name))
            else:
                for space, sp in spaces.items():
                    inj = inject(sp["data"], sp["cols"], cfg, space,
                                 rng=np.random.default_rng(args.seed),
                                 scale=sp["scale"], segments=SEG[split])
                    row[space] = score_windows(sp["model"], inj, sp["mean"], sp["std"],
                                               W, S, dev, ST[split], args.batch_size)
                    del inj
            for space in ("residual", "raw"):
                sc[space].append(row[space])
            per[name] = row
        return (np.concatenate(ys),
                {k: np.concatenate(v) for k, v in sc.items()}, per)

    t0 = time.time()
    y_val, s_val, per_val = injected("val")
    print(f"  doğrulama test pencereleri: {len(y_val):,}  arızalı {int(y_val.sum()):,} "
          f"(%{100*y_val.mean():.1f})   [{time.time()-t0:.0f}s]")

    # lo/span YALNIZ temiz doğrulama pencerelerinden. Bildirinin min–max formülü
    # korunur; değişen tek şey sınırların hangi kümeden alındığıdır.
    rv, hv = clean["residual"]["val"], clean["raw"]["val"]
    SCALE = {"residual": {"lo": float(rv.min()), "span": float(rv.max() - rv.min())},
             "raw": {"lo": float(hv.min()), "span": float(hv.max() - hv.min())}}

    def z(space, s_):
        return (s_ - SCALE[space]["lo"]) / SCALE[space]["span"]

    zr_vc, zh_vc = z("residual", rv), z("raw", hv)              # temiz doğrulama
    zr_vi, zh_vi = z("residual", s_val["residual"]), z("raw", s_val["raw"])

    # ── Rejim-koşullu eşik ───────────────────────────────────────────────
    # Ölçüm: temiz birleşik skor hareket rejimine göre BİMODAL. Doğrulamada
    # duran pencerelerin medyanı 0,0012, hareketlilerinki 0,1784 — 150 kat.
    # Doğrulama kümesinin %86'sı duran pencere olduğu için global P97 esasen
    # DURAN robotun gürültüsünün 97. persentilidir ve hareket eden bir robot
    # için anlamı yoktur. Hareketli-P97 / duran-P97 = 7,3× — gerçek hücrede
    # ölçülen "eşik yedi kat düşük kaldı" faktörüyle aynı büyüklükte.
    #
    # Bu yüzden eşik iki rejimde ayrı hesaplanır. Düğüm zaten her kararda
    # `moving = qd_peak > motion_qd_min` üretiyor; tek değişen, hangi eşiğin
    # uygulandığı. Global eşik --global-threshold ile korunur.
    mv_v = MOV["val"]
    mv_vi = np.tile(mv_v, len(FAULTS))          # senaryolar aynı pencere kümesinde

    # Eşik KURALI da bir seçimdir ve doğrulamada seçilir. Üç aday:
    #
    #   global_p97  tek eşik, temiz doğrulamanın P97'si            (eski davranış)
    #   regime_p97  rejim başına P97                               (ölçüldü: hareketli
    #               eşik çok kararlı — tohumlar arası bağıl std %10 — ama DURAN eşik
    #               %45, çünkü duran dağılım sıfıra yığılı ve P97 onun kuyruğunda bir
    #               sıra istatistiği; küçük model farkları onu 4 kat oynatıyor)
    #   regime_mad  rejim başına medyan + k·1,4826·MAD, k tek sefer havuzlanmış temiz
    #               doğrulamadan hedef yanlış alarm oranına göre çözülür. Konum ve
    #               ölçek tahmini kuyruk sıra istatistiğinden çok daha kararlıdır;
    #               eşik bir EKSTRAPOLASYON olur, bir kuyruk örneği değil.
    def thresholds(fc_clean: np.ndarray, rule: str) -> dict:
        g = float(np.percentile(fc_clean, pct))
        if rule == "global_p97":
            return {"static": g, "moving": g, "global": g, "rule": rule}
        out = {"global": g, "rule": rule}
        for tag, m in (("static", ~mv_v), ("moving", mv_v)):
            # Bir rejim doğrulamada neredeyse hiç yoksa tahmin gürültüdür;
            # o rejimde global eşiğe düşülür.
            if m.sum() < 50:
                out[tag] = g
                continue
            out[tag] = float(np.percentile(fc_clean[m], pct))
        return out

    def apply_thr(score: np.ndarray, moving: np.ndarray, thr: dict) -> np.ndarray:
        return np.where(moving, thr["moving"], thr["static"])

    # Her iki kural da HER ZAMAN hesaplanır ve raporlanır; hangisinin
    # dondurulacağı --threshold-rule ile seçilir.
    #
    # Ölçüldü (5 tohum): eşleşen görev döngüsünde global daha iyi (F1 0,791 ±
    # 0,023). Ama doğrulama %14 hareketli, gerçek muayene çevrimi ağırlıklı
    # hareketli; döngü kayınca global'in yanlış alarmı %1,9'dan %5,7'ye tırmanıyor
    # ve F1'i 0,824'ten 0,708'e düşüyor. Rejim eşiğinin F1'i aynı aralıkta
    # 0,772–0,782'de sabit kalıyor. Yani offline tablo için global, SAHA için
    # rejim. Bu bir F1 tercihi değil, öngörülebilirlik tercihidir.
    RULES = ["global_p97", "regime_p97"]

    # w ve eşik kuralı BİRLİKTE, yalnız doğrulamada seçilir.
    all_sweeps, best_per_rule = {}, {}
    for rule in RULES:
        sw = []
        for w_ in np.round(np.arange(0.0, 1.0001, 0.05), 2):
            fc = w_ * zr_vc + (1 - w_) * zh_vc
            fi = w_ * zr_vi + (1 - w_) * zh_vi
            th = thresholds(fc, rule)
            o = operating(y_val, fi - apply_thr(fi, mv_vi, th), 0.0)
            sw.append({"w_res": float(w_), "threshold": th, "precision": o["precision"],
                       "recall": o["recall"], "f1": o["f1"],
                       "pr_auc": float(average_precision_score(y_val, fi))})
        all_sweeps[rule] = sw
        best_per_rule[rule] = max(sw, key=lambda d: d["f1"])

    print(f"\n  {'kural':<13}{'w_res':>7}{'θ duran':>11}{'θ hareketli':>13}"
          f"{'kesinlik':>11}{'geri çağırma':>14}{'F1 (doğrulama)':>16}")
    print("  " + "-" * 76)
    for rule, b in best_per_rule.items():
        t = b["threshold"]
        print(f"  {rule:<13}{b['w_res']:>7.2f}{t['static']:>11.4f}{t['moving']:>13.4f}"
              f"{b['precision']:>11.3f}{b['recall']:>14.3f}{b['f1']:>16.3f}")

    RULE = args.threshold_rule
    sweep = all_sweeps[RULE]
    if args.select_w:
        BEST = best_per_rule[RULE]
        Wres = BEST["w_res"]
        print(f"\n  ⚠ ağırlık DOĞRULAMADA seçildi (--select-w): w_res = {Wres:.2f}")
    else:
        # Önsel sabit: hiçbir değerlendirme kümesi ağırlığı görmez.
        Wres = float(np.round(args.w_res, 2))
        cand = [r for r in sweep if abs(r["w_res"] - Wres) < 1e-9]
        if not cand:
            print(f"HATA: w_res={Wres} tarama ızgarasında yok.", file=sys.stderr)
            return 2
        BEST = cand[0]
        arg = max(sweep, key=lambda d: d["f1"])
        print(f"\n  ağırlık ÖNSEL sabit: w_res = {Wres:.2f} (bildiriden). "
              f"Doğrulamanın en iyisi {arg['w_res']:.2f} olurdu "
              f"(F1 {arg['f1']:.3f} ↔ {BEST['f1']:.3f}) — seçim YAPILMADI.")
    THR = BEST["threshold"]

    fc_best = Wres * zr_vc + (1 - Wres) * zh_vc
    fa_val = float((fc_best > apply_thr(fc_best, mv_v, THR)).mean())
    print(f"  DONDURULDU: kural = {RULE} · w_res = {Wres:.2f}"
          f" ({'doğrulamada seçildi' if args.select_w else 'önsel sabit'})")
    print(f"    duran     θ = {THR['static']:.4f}   (n={int((~mv_v).sum()):,} temiz pencere)")
    print(f"    hareketli θ = {THR['moving']:.4f}   (n={int(mv_v.sum()):,} temiz pencere)")
    if THR["static"] > 0:
        print(f"    hareketli/duran oranı = {THR['moving']/THR['static']:.1f}x"
              + (f" · k = {THR['k']:.2f}" if "k" in THR else ""))
    print(f"  temiz doğrulamada yanlış alarm: %{100*fa_val:.1f}  "
          f"(P{pct:g} kuralının hedefi %{100-pct:.0f})")
    mode = RULE

    # ── TEST: buraya bir kez dokunulur ───────────────────────────────────
    print("\n" + "=" * 78)
    print("TEST — dondurulmuş yapılandırmayla tek geçiş")
    print("=" * 78)
    t0 = time.time()
    y, s_test, per_test = injected("test")
    zr, zh = z("residual", s_test["residual"]), z("raw", s_test["raw"])
    fus = Wres * zr + (1 - Wres) * zh
    print(f"  test pencereleri: {len(y):,}  arızalı {int(y.sum()):,} (%{100*y.mean():.1f})"
          f"   [{time.time()-t0:.0f}s]")

    res = {"Kalıntı LSTM Özk.": metrics(y, s_test["residual"]),
           "Ham LSTM Özk.": metrics(y, s_test["raw"]),
           f"Birleşim ({Wres:.2f}/{1-Wres:.2f})": metrics(y, fus),
           "Birleşim MAX": metrics(y, np.maximum(zr, zh))}

    # ── referans yöntemler: EĞİTİM koşularında uydurulur ──
    if not args.no_baselines:
        rd = spaces["residual"]; cols = rd["cols"]
        ii_int = [cols.index(f"r_int_{j}") for j in range(1, 7)]
        ii_ext = [cols.index(f"r_ext_{j}") for j in range(1, 7)]
        print("\n  Referans yöntemler EĞİTİM koşularının temiz pencerelerinde uyduruluyor...")
        f_tr = win_feats(rd["data"], W, S, ST["train"])
        sca = StandardScaler().fit(f_tr)
        iso = IsolationForest(contamination=0.08, random_state=args.seed, n_jobs=-1)
        iso.fit(sca.transform(f_tr))
        sub = rng.choice(len(f_tr), size=min(args.ocsvm_subsample, len(f_tr)), replace=False)
        svm = OneClassSVM(kernel="rbf", nu=0.05, gamma="scale").fit(sca.transform(f_tr[sub]))
        del f_tr
        norm_sc, if_sc, sv_sc = [], [], []
        for name, cfg in FAULTS.items():
            inj = inject(rd["data"], cols, cfg, "residual",
                         rng=np.random.default_rng(args.seed),
                         scale=rd["scale"], segments=SEG["test"])
            r_tot = inj[:, ii_int].astype(np.float64) + inj[:, ii_ext]
            norm_sc.append(win_mean(np.linalg.norm(r_tot, axis=1), W, S, ST["test"]))
            f = sca.transform(win_feats(inj, W, S, ST["test"]))
            if_sc.append(-iso.score_samples(f))
            sv_sc.append(-svm.decision_function(f))
            del inj, r_tot, f
        for k_, v_ in (("Art. Norm Eşik", norm_sc), ("Isolation Forest", if_sc),
                       ("One-Class SVM", sv_sc)):
            res[k_] = metrics(y, np.concatenate(v_))

    print("\n" + "=" * 78)
    print("TABLO — TEST KÜMESİNDE SIRALAMA KALİTESİ")
    print("=" * 78)
    print(f"  {'Model':<26}{'AUC':>9}{'PR-AUC':>10}{'BestF1':>10}")
    print("  " + "-" * 56)
    for k, v in res.items():
        print(f"  {k:<26}{v['auc']:>9.3f}{v['pr_auc']:>10.3f}{v['best_f1']:>10.3f}")
    print("  BestF1 tüm eşiklerin maksimumudur; etiketleri kullanır, çevrimiçi")
    print("  karşılığı yoktur. Çalışma noktası aşağıda ayrı verilir.")

    # ── çalışma noktası: DONDURULMUŞ eşik ──
    mv_t = np.tile(MOV["test"], len(FAULTS))
    print("\n" + "=" * 78)
    print(f"ÇALIŞMA NOKTASI — w_res={Wres:.2f}, eşik {mode} (doğrulamadan donduruldu)")
    print("=" * 78)
    ops = {}
    singles = {"Yalnız ham (w=0.00)": (zh_vc, zh),
               "Yalnız kalıntı (w=1.00)": (zr_vc, zr)}
    for nm, (clean_v, sc_) in singles.items():
        th = thresholds(clean_v, RULE)
        ops[nm] = operating(y, sc_ - apply_thr(sc_, mv_t, th), 0.0)
        ops[nm]["threshold"] = th
    ops[f"Birleşim (w={Wres:.2f})"] = operating(y, fus - apply_thr(fus, mv_t, THR), 0.0)
    ops[f"Birleşim (w={Wres:.2f})"]["threshold"] = THR
    for nm, o in ops.items():
        th = o["threshold"]
        print(f"  {nm:<26}θ {th['static']:.4f}/{th['moving']:.4f}  "
              f"P {o['precision']:.3f}  R {o['recall']:.3f}  F1 {o['f1']:.3f}  "
              f"TP {o['tp']:>5} FP {o['fp']:>5} FN {o['fn']:>5}")

    # Aynı yapılandırmanın GLOBAL eşikle ne verdiği — rejim ayrımının katkısını
    # ölçmenin tek yolu bu. Makalede yan yana raporlanmalı.
    if RULE != "global_p97":
        gth = thresholds(fc_best, "global_p97")
        og = operating(y, fus - apply_thr(fus, mv_t, gth), 0.0)
        ops["Birleşim (global eşik)"] = dict(og, threshold=gth)
        print(f"  {'Birleşim (global eşik)':<26}θ {THR['global']:.4f}         "
              f"P {og['precision']:.3f}  R {og['recall']:.3f}  F1 {og['f1']:.3f}"
              f"   ← rejim ayrımı olmasaydı")

    # OR: KARAR düzeyinde birleşim. İkili çıktı üretir, sıralama üretmez;
    # AUC/PR-AUC/BestF1 bu satır için TANIMSIZDIR ve raporlanmaz.
    det_r = s_test["residual"] > spaces["residual"]["thr"]
    det_h = s_test["raw"] > spaces["raw"]["thr"]
    det_or = det_r | det_h
    tp = int((det_or & y).sum()); fp = int((det_or & ~y).sum()); fn = int((~det_or & y).sum())
    pr_or = tp / max(tp + fp, 1); rc_or = tp / max(tp + fn, 1)
    ops["OR (karar düzeyi)"] = {"precision": pr_or, "recall": rc_or,
                                "f1": 2 * pr_or * rc_or / (pr_or + rc_or + 1e-12),
                                "tp": tp, "fp": fp, "fn": fn, "note": "AUC/PR-AUC tanımsız"}
    print(f"  {'OR (karar düzeyi)':<26}{'':>10}  P {pr_or:.3f}  R {rc_or:.3f}  "
          f"F1 {ops['OR (karar düzeyi)']['f1']:.3f}   (AUC/PR-AUC tanımsız)")

    # ── arıza tipine göre: AUC ve BestF1 AYRI AYRI ───────────────────────
    # Eski hatta bu tablo BestF1 üretiyor ama makalede AUC diye anılıyordu.
    # 0,443 (BestF1) ile 0,787 (AUC) çok farklı hikâyeler anlatır; ikisi de
    # basılıyor ve sütunlar adlarıyla çağrılıyor.
    print("\n" + "=" * 78)
    print("ARIZA TİPİNE GÖRE — AUC ve BestF1 (aynı sayı DEĞİL)")
    print("=" * 78)
    n_win_t = len(ST["test"])
    per_type = {}
    print(f"  {'Arıza Tipi':<18}{'olay':>6}{'kal.AUC':>9}{'ham.AUC':>9}{'bir.AUC':>9}"
          f"{'kal.F1':>9}{'ham.F1':>9}{'bir.F1':>9}")
    print("  " + "-" * 76)
    for i, (name, cfg) in enumerate(FAULTS.items()):
        sl = slice(i * n_win_t, (i + 1) * n_win_t)
        yy = per_test[name]["labels"]
        if yy.sum() == 0 or yy.all():
            print(f"  {cfg['tr']:<18}  — tek sınıf, ölçüt tanımsız")
            continue
        row = {}
        for tag, sc_ in (("residual", z("residual", per_test[name]["residual"])),
                         ("raw", z("raw", per_test[name]["raw"])),
                         ("fusion", fus[sl])):
            row[tag] = {"auc": float(roc_auc_score(yy, sc_)),
                        "best_f1": best_f1(yy, sc_)[0]}
        per_type[name] = row
        print(f"  {cfg['tr']:<18}{len(SEG['test']):>6}"
              f"{row['residual']['auc']:>9.3f}{row['raw']['auc']:>9.3f}{row['fusion']['auc']:>9.3f}"
              f"{row['residual']['best_f1']:>9.3f}{row['raw']['best_f1']:>9.3f}"
              f"{row['fusion']['best_f1']:>9.3f}")
    print(f"  'olay' = senaryonun enjekte edildiği bağımsız test koşusu sayısı")
    print(f"  (eski protokolde bu sayı senaryo başına 1, toplamda 4 idi)")

    # ── tamamlayıcılık ──
    nf = int(y.sum())
    only_r = int((det_r & ~det_h & y).sum()); only_h = int((det_h & ~det_r & y).sum())
    both = int((det_r & det_h & y).sum()); none_ = int((~det_r & ~det_h & y).sum())
    comp = {"n_fault_windows": nf, "only_residual_pct": 100 * only_r / max(nf, 1),
            "only_raw_pct": 100 * only_h / max(nf, 1), "both_pct": 100 * both / max(nf, 1),
            "neither_pct": 100 * none_ / max(nf, 1),
            "recall_residual": float((det_r & y).sum() / max(nf, 1)),
            "recall_raw": float((det_h & y).sum() / max(nf, 1)),
            "recall_or": float((det_or & y).sum() / max(nf, 1))}
    print("\n" + "=" * 78)
    print("TAMAMLAYICILIK (pencere düzeyinde, her model kendi P97 eşiğiyle)")
    print("=" * 78)
    print(f"  {nf:,} arıza penceresinden: yalnız kalıntı %{comp['only_residual_pct']:.1f} · "
          f"yalnız ham %{comp['only_raw_pct']:.1f} · ikisi %{comp['both_pct']:.1f} · "
          f"hiçbiri %{comp['neither_pct']:.1f}")
    print(f"  geri çağırma — kalıntı {comp['recall_residual']:.3f} · "
          f"ham {comp['recall_raw']:.3f} · OR {comp['recall_or']:.3f}")

    # ── çevrimiçi yapılandırma ──
    cfg_out = {
        "protocol": "run-disjoint v3", "splits": str(args.splits),
        "norm": "minmax_val", "scale": SCALE,
        "w_kal": Wres, "w_ham": round(1.0 - Wres, 4),
        # Düğüm rejim-koşullu eşiği okur; `fused_threshold` eski düğümlerle
        # uyumluluk için global değeri taşır ve rejim alanı varsa KULLANILMAZ.
        "fused_threshold": THR["global"],
        "threshold_by_regime": {
            "static": best_per_rule["regime_p97"]["threshold"]["static"],
            "moving": best_per_rule["regime_p97"]["threshold"]["moving"]},
        "motion_qd_min": args.motion_qd_min,
        "threshold_rule": RULE,
        "regime_threshold": RULE != "global_p97",
        "threshold_percentile": pct,
        "w_source": ("validation" if args.select_w else "a-priori (reference study)"),
        "selected_on": "threshold from validation runs only; weight "
                       + ("selected on validation" if args.select_w else "fixed a priori"),
        "residual": {"threshold": spaces["residual"]["thr"]},
        "raw": {"threshold": spaces["raw"]["thr"]},
        "clean_val_false_alarm": fa_val,
        # Düğüm bu bayrağı okuyup friction_model parametresinin verilip
        # verilmediğiyle karşılaştırmalı.
        "friction_applied": bool(spaces["residual"]["meta"].get("friction_applied")),
    }
    (out_dir / "fusion_config.json").write_text(
        json.dumps(cfg_out, indent=2, ensure_ascii=False), encoding="utf-8")

    np.savez_compressed(out_dir / "scores.npz", y=y,
                        s_residual=s_test["residual"], s_raw=s_test["raw"],
                        fused=fus, starts=ST["test"],
                        clean_train_res=clean["residual"]["train"],
                        clean_val_res=clean["residual"]["val"],
                        clean_test_res=clean["residual"]["test"],
                        clean_train_raw=clean["raw"]["train"],
                        clean_val_raw=clean["raw"]["val"],
                        clean_test_raw=clean["raw"]["test"])
    (out_dir / "results.json").write_text(json.dumps(
        {"protocol": "run-disjoint v3", "seed": args.seed, "parquet": str(pq_path),
         "n_test_windows": int(len(y)), "n_fault_windows": nf,
         "n_fault_events_per_scenario": len(SEG["test"]),
         "split_summary": SP["summary"], "drift": drift,
         "sweep": sweep, "rule_comparison": best_per_rule,
         "frozen": {"w_res": Wres, "threshold": THR, "rule": RULE,
                    "w_source": "validation" if args.select_w else "a-priori"},
         "ranking": res, "operating": ops, "per_fault": per_type,
         "complementarity": comp},
        indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n  → {out_dir}/results.json · fusion_config.json · scores.npz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
