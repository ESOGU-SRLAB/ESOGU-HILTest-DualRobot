#!/usr/bin/env python3
"""
train_ae.py
===========
245.pdf'teki iki LSTM özkodlayıcısını birebir eğitir.

    python train_ae.py --mode residual    # 12 kanal, 128/32,   478.892 param
    python train_ae.py --mode raw         # 24 kanal, 256/64, 1.907.032 param

Bildirideki eğitim yapılandırması (Bölüm III):
    Adam (lr=1e-3, β1=0.9, β2=0.999) · yığın 256 · sönümleme %15
    gradyan kırpma max_norm=1.0 · ReduceLROnPlateau (faktör 0.5, sabır 8)
    erken durdurma sabır 25, maksimum 300 dönem · MSE kaybı
    eşik = doğrulama yeniden yapılanma hatalarının 97. persentili

Bildirinin ulaştığı değerler (karşılaştırma için otomatik raporlanır):
    kalıntı : 129. dönem, val_loss 0.181, θ = 0.420
    ham     :  87. dönem, val_loss 0.320, θ = 0.854
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
import torch.nn as nn
from torch.utils.data import DataLoader

sys.path.insert(0, str(Path(__file__).resolve().parent))
from models import PRESETS, WindowDataset, build, export_onnx  # noqa: E402


def human(s: float) -> str:
    return f"{s:.0f}s" if s < 60 else (f"{s/60:.1f}dk" if s < 3600 else f"{s/3600:.2f}sa")


def main() -> int:
    ap = argparse.ArgumentParser(description="LSTM-AE eğitimi (245.pdf spesifikasyonu)")
    ap.add_argument("--mode", required=True, choices=["residual", "raw", "hybrid"])
    ap.add_argument("--parquet", default=None)
    ap.add_argument("--model-dir", default=None)
    ap.add_argument("--epochs", type=int, default=None)
    ap.add_argument("--batch-size", type=int, default=None)
    ap.add_argument("--lr", type=float, default=None)
    ap.add_argument("--patience", type=int, default=None)
    ap.add_argument("--stride", type=int, default=None)
    ap.add_argument("--dropout", type=float, default=None)
    ap.add_argument("--splits", default="splits.json",
                    help="make_splits.py çıktısı. Eğitim/doğrulama bölmesi KOŞU "
                         "kimliğinden yapılır; satır indeksinden bölmek test "
                         "kümesiyle örtüşen pencereler üretir.")
    ap.add_argument("--norm", choices=["train", "all"], default="train",
                    help="normalizasyon istatistiği kaynağı (varsayılan: sadece eğitim böleni)")
    ap.add_argument("--device", default=None)
    ap.add_argument("--num-workers", type=int, default=4)
    ap.add_argument("--amp", action="store_true")
    ap.add_argument("--limit-rows", type=int, default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-onnx", action="store_true")
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    mode = "residual" if args.mode == "hybrid" else args.mode
    meta = dict(PRESETS[mode])
    parquet = args.parquet or meta["default_parquet"]
    model_dir = Path(args.model_dir or meta["default_model_dir"])
    for k in ("epochs", "batch_size", "lr", "patience", "stride", "dropout"):
        v = getattr(args, k)
        if v is not None:
            meta[k] = v
    model_dir.mkdir(parents=True, exist_ok=True)

    dev = torch.device(args.device) if args.device else torch.device(
        "cuda" if torch.cuda.is_available() else "cpu")

    print("=" * 72)
    print(f"LSTM-AE EĞİTİMİ — {mode.upper()}   (245.pdf)")
    print("=" * 72)
    print(f"  parquet   : {parquet}")
    print(f"  model dir : {model_dir}")
    print(f"  kanal {meta['features']:>3} | pencere {meta['window_size']} / adım {meta['stride']} "
          f"| gizli {meta['hidden_dim']} / saklı {meta['latent_dim']} | sönümleme {meta['dropout']}")
    print(f"  Adam lr={meta['lr']} β={meta['betas']} | yığın {meta['batch_size']} "
          f"| kırpma {meta['grad_clip']} | plateau({meta['plateau_factor']}, {meta['plateau_patience']})")
    print(f"  maks {meta['epochs']} dönem, sabır {meta['patience']} | eşik P{meta['threshold_percentile']}")
    print(f"  cihaz     : {dev}" + (f" ({torch.cuda.get_device_name(0)})" if dev.type == "cuda" else ""))

    # ── veri ──
    print("\nVeri yükleniyor...")
    t0 = time.time()
    import pyarrow.parquet as _pq
    have = set(_pq.ParquetFile(parquet).schema.names)
    aux = [c for c in ("run_id", "valid") if c in have]
    df = pd.read_parquet(parquet, columns=meta["feature_cols"] + aux)
    if args.limit_rows:
        df = df.iloc[:args.limit_rows]
    groups = df["run_id"].to_numpy() if "run_id" in aux else None
    valid = df["valid"].to_numpy(bool) if "valid" in aux else np.ones(len(df), bool)
    data = df[meta["feature_cols"]].to_numpy(dtype=np.float32)
    del df
    finite = np.isfinite(data).all(axis=1)
    if not finite.all():
        print(f"  ⚠ {int((~finite).sum()):,} satırda NaN/Inf → geçersiz işaretlendi")
        valid &= finite
        data = np.nan_to_num(data, nan=0.0, posinf=0.0, neginf=0.0)
    n = len(data)
    print(f"  {n:,} örnek × {data.shape[1]} kanal   ({human(time.time()-t0)})")
    if groups is None:
        print("  ⚠ 'run_id' yok → tüm dizi tek kesintisiz koşu varsayılıyor "
              "(prepare_dataset.py çalıştırılmamış).")
    else:
        print(f"  {len(np.unique(groups)):,} kesintisiz koşu | "
              f"geçerli örnek %{100*valid.mean():.1f}")

    # Eğitim/doğrulama bölmesi KOŞU KİMLİĞİNDEN yapılır — satır indeksinden değil.
    #
    # Eski hat `int(N * 0.8)` ile bölüyordu ve değerlendirme kümesi TÜM pencerelerden
    # kuruluyordu; sonuçta değerlendirme pencerelerinin %79,6'sı modelin eğitimde
    # gördüğü pencerelerdi. Ölçüldü: temiz pencerelerde yeniden yapılanma hatası
    # eğitim bölgesinde 0,007, dışında 0,486. O bölmeyle ölçülen hiçbir genelleme
    # sayısı taşınabilir değil. Test koşularına burada HİÇ dokunulmaz.
    sp_path = Path(args.splits)
    if not sp_path.exists():
        print(f"HATA: {sp_path} yok — önce `python3 make_splits.py` çalıştır.",
              file=sys.stderr)
        return 2
    SP = json.loads(sp_path.read_text(encoding="utf-8"))
    if groups is None:
        print("HATA: parquet'te run_id yok; koşu-ayrık bölme yapılamaz.", file=sys.stderr)
        return 2
    m_tr = np.isin(groups, SP["train"])
    m_va = np.isin(groups, SP["val"])
    m_te = np.isin(groups, SP["test"])
    if not (m_tr.any() and m_va.any()):
        print("HATA: bölme dosyası bu parquet ile eşleşmiyor.", file=sys.stderr)
        return 2
    print(f"  bölme: {sp_path}  eğitim {len(SP['train'])} koşu / {int(m_tr.sum()):,} örnek · "
          f"doğrulama {len(SP['val'])} koşu / {int(m_va.sum()):,} örnek")
    print(f"  test {len(SP['test'])} koşu / {int(m_te.sum()):,} örnek — bu eğitimde HİÇ görülmüyor")

    src = data[m_tr] if args.norm == "train" else data
    mean, std = src.mean(axis=0), src.std(axis=0)
    std[std < 1e-8] = 1.0

    def mk(mask):
        # Maskelenmiş kopya üzerinde pencereler yine run_id'ye göre kurulur, yani
        # bir pencere asla iki koşuya yayılmaz. Bölmeler koşu-ayrık olduğu için
        # eğitim ve doğrulama pencereleri hiçbir örneği paylaşmaz.
        return WindowDataset((data[mask] - mean) / std, meta["window_size"],
                             meta["stride"], groups=groups[mask], valid=valid[mask])

    tr_ds, va_ds = mk(m_tr), mk(m_va)
    print(f"  eğitim {len(tr_ds):,} pencere | doğrulama {len(va_ds):,} pencere")
    if not len(tr_ds) or not len(va_ds):
        print("HATA: pencere üretilemedi.", file=sys.stderr)
        return 2

    pin = dev.type == "cuda"
    nw = args.num_workers
    tr_ld = DataLoader(tr_ds, batch_size=meta["batch_size"], shuffle=True, num_workers=nw,
                       pin_memory=pin, persistent_workers=nw > 0)
    va_ld = DataLoader(va_ds, batch_size=meta["batch_size"], shuffle=False, num_workers=nw,
                       pin_memory=pin, persistent_workers=nw > 0)

    # ── model ──
    model = build(meta).to(dev)
    npar = sum(p.numel() for p in model.parameters())
    meta["total_params"] = npar
    exp = meta.get("expected_params")
    print(f"  parametre : {npar:,}" +
          (f"   (bildiri {exp:,}) {'✅' if npar == exp else '❌ UYUŞMUYOR'}" if exp else ""))

    opt = torch.optim.Adam(model.parameters(), lr=meta["lr"], betas=tuple(meta["betas"]))
    sched = torch.optim.lr_scheduler.ReduceLROnPlateau(
        opt, mode="min", factor=meta["plateau_factor"], patience=meta["plateau_patience"])
    crit = nn.MSELoss()

    # torch sürüm uyumluluğu: torch.amp.GradScaler(device,...) 2.3+ ile geldi
    use_amp = bool(args.amp and dev.type == "cuda")
    try:
        scaler = torch.amp.GradScaler("cuda", enabled=use_amp)
        autocast = lambda: torch.amp.autocast("cuda", enabled=use_amp)  # noqa: E731
    except (AttributeError, TypeError):
        scaler = torch.cuda.amp.GradScaler(enabled=use_amp)
        autocast = lambda: torch.cuda.amp.autocast(enabled=use_amp)     # noqa: E731

    best, bad = float("inf"), 0
    tr_hist, va_hist, lr_hist = [], [], []
    ckpt = model_dir / "best_model.pt"

    print(f"\n{'Dönem':>6} | {'Train':>11} | {'Val':>11} | {'lr':>9} | {'süre':>7} | durum")
    print("-" * 70)
    t_train = time.time()
    epoch = 0
    for epoch in range(1, meta["epochs"] + 1):
        te = time.time()
        model.train()
        s = 0.0
        for b in tr_ld:
            b = b.to(dev, non_blocking=pin)
            opt.zero_grad(set_to_none=True)
            with autocast():
                loss = crit(model(b), b)
            scaler.scale(loss).backward()
            scaler.unscale_(opt)
            nn.utils.clip_grad_norm_(model.parameters(), meta["grad_clip"])
            scaler.step(opt)
            scaler.update()
            s += loss.item() * len(b)
        tr = s / len(tr_ds)

        model.eval()
        s = 0.0
        with torch.no_grad():
            for b in va_ld:
                b = b.to(dev, non_blocking=pin)
                with autocast():
                    s += crit(model(b), b).item() * len(b)
        va = s / len(va_ds)

        lr_now = opt.param_groups[0]["lr"]
        sched.step(va)
        tr_hist.append(tr); va_hist.append(va); lr_hist.append(lr_now)

        if va < best:
            best, bad = va, 0
            torch.save(model.state_dict(), ckpt)
            st = "★ kaydedildi"
        else:
            bad += 1
            st = f"({bad}/{meta['patience']})"
        print(f"{epoch:6d} | {tr:11.6f} | {va:11.6f} | {lr_now:9.2e} | "
              f"{time.time()-te:6.1f}s | {st}")
        if bad >= meta["patience"]:
            print(f"\nErken durdurma — dönem {epoch}.")
            break

    bestep = int(np.argmin(va_hist)) + 1
    print(f"\nEğitim: {human(time.time()-t_train)} | en iyi dönem {bestep} | val_loss {best:.6f}")
    if "paper_epochs" in meta:
        print(f"  bildiri: dönem {meta['paper_epochs']}, val_loss {meta['paper_val_loss']:.3f}")

    # ── eşik (P97) ──
    print("\nEşik hesaplanıyor (doğrulama seti, P97)...")
    model.load_state_dict(torch.load(ckpt, map_location=dev, weights_only=True))
    model.eval()
    errs = []
    with torch.no_grad():
        for b in va_ld:
            b = b.to(dev, non_blocking=pin)
            errs.append(((model(b) - b) ** 2).mean(dim=(1, 2)).float().cpu().numpy())
    errs = np.concatenate(errs)
    thr = float(np.percentile(errs, meta["threshold_percentile"]))
    print(f"  hata: ortalama {errs.mean():.6f}  std {errs.std():.6f}")
    print(f"  θ (P{meta['threshold_percentile']:g}) = {thr:.6f}" +
          (f"   (bildiri θ = {meta['paper_threshold']:.3f})" if "paper_threshold" in meta else ""))

    meta.update({
        "mode": mode, "parquet": str(parquet), "norm_source": args.norm,
        "split_source": str(args.splits), "split_kind": "run-disjoint",
        # Kalıntı sürtünme çıkarılarak üretildiyse düğüm de ÇIKARMAK zorundadır.
        # Eşleşmezse fark sabit bir yanlılık olarak her karara girer ve hiçbir
        # yerde hata gibi görünmez — bu yüzden köken metadata'ya yazılıyor.
        "friction_applied": bool("tau_fric_1" in have),
        "n_train_runs": len(SP["train"]), "n_val_runs": len(SP["val"]),
        "n_test_runs": len(SP["test"]),
        "mean": mean.tolist(), "std": std.tolist(),
        "threshold": thr, "threshold_mean": float(errs.mean()),
        "threshold_std": float(errs.std()),
        "best_val_loss": best, "best_epoch": bestep, "epochs_trained": epoch,
        "n_samples": int(n), "n_train_windows": len(tr_ds), "n_val_windows": len(va_ds),
        "train_losses": tr_hist, "val_losses": va_hist, "lr_history": lr_hist,
        "torch_version": torch.__version__,
    })
    meta["betas"] = list(meta["betas"])
    (model_dir / "metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    np.save(model_dir / "val_window_errors.npy", errs)

    if not args.no_onnx:
        try:
            m = build(meta)
            m.load_state_dict(torch.load(ckpt, map_location="cpu", weights_only=True))
            export_onnx(m.eval(), meta, model_dir / "model.onnx")
            print(f"  ONNX → {model_dir/'model.onnx'}")
        except Exception as e:
            print(f"  ⚠ ONNX export başarısız: {e}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
        ax[0].plot(range(1, len(tr_hist) + 1), tr_hist, label="train")
        ax[0].plot(range(1, len(va_hist) + 1), va_hist, label="val")
        ax[0].axvline(bestep, color="r", ls="--", lw=1, label=f"en iyi ({bestep})")
        if "paper_epochs" in meta:
            ax[0].axvline(meta["paper_epochs"], color="g", ls=":", lw=1,
                          label=f"bildiri ({meta['paper_epochs']})")
        ax[0].set_xlabel("dönem"); ax[0].set_ylabel("MSE"); ax[0].set_yscale("log")
        ax[0].set_title(f"Eğitim kaybı — {mode}"); ax[0].legend(); ax[0].grid(alpha=.3)
        ax[1].plot(range(1, len(lr_hist) + 1), lr_hist, color="darkorange")
        ax[1].set_xlabel("dönem"); ax[1].set_ylabel("lr"); ax[1].set_yscale("log")
        ax[1].set_title("ReduceLROnPlateau"); ax[1].grid(alpha=.3)
        fig.tight_layout(); fig.savefig(model_dir / "training_loss.png", dpi=130)
    except Exception as e:
        print(f"  (grafik atlandı: {e})")

    print(f"\nTAMAM → {model_dir}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
