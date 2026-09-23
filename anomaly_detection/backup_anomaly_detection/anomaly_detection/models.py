"""
models.py
=========
245.pdf — "FMU Tabanlı Kalıntı Ayrıştırma ve İkili LSTM Özkodlayıcı Birleşimi"
bildirisindeki İKİ modelin birebir tanımı.

    kalıntı (residual) : 12 kanal (r_ic,1–6 + r_dis,1–6),  128/32  →   478.892 param
    ham     (raw)      : 24 kanal (q, q̇, τ, KTS × 6),     256/64  → 1.907.032 param

Her ikisi de T=100 örneklik pencere, iki katmanlı LSTM gizyazar → saklı vektör →
RepeatVector → simetrik LSTM gizçözer. Katmanlar arasında %15 sönümleme.

Parametre sayıları bildiriyle BİREBİR doğrulandı (`python models.py` ile kontrol edilir).
"""

from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset


# ─────────────────────────── Model ───────────────────────────

class LSTMEncoder(nn.Module):
    def __init__(self, input_dim, hidden_dim, latent_dim, num_layers, dropout=0.0):
        super().__init__()
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers=num_layers,
                            batch_first=True, dropout=dropout if num_layers > 1 else 0.0)
        self.fc = nn.Linear(hidden_dim, latent_dim)

    def forward(self, x):
        _, (h_n, _) = self.lstm(x)
        return self.fc(h_n[-1])


class LSTMDecoder(nn.Module):
    def __init__(self, latent_dim, hidden_dim, output_dim, seq_len, num_layers, dropout=0.0):
        super().__init__()
        self.seq_len = seq_len
        self.fc = nn.Linear(latent_dim, hidden_dim)
        self.lstm = nn.LSTM(hidden_dim, hidden_dim, num_layers=num_layers,
                            batch_first=True, dropout=dropout if num_layers > 1 else 0.0)
        self.out = nn.Linear(hidden_dim, output_dim)

    def forward(self, z):
        # RepeatVector: saklı vektörü T adıma kopyala
        h = self.fc(z).unsqueeze(1).repeat(1, self.seq_len, 1)
        out, _ = self.lstm(h)
        return self.out(out)


class LSTMAutoencoder(nn.Module):
    def __init__(self, input_dim, hidden_dim, latent_dim, seq_len, num_layers, dropout=0.0):
        super().__init__()
        self.encoder = LSTMEncoder(input_dim, hidden_dim, latent_dim, num_layers, dropout)
        self.decoder = LSTMDecoder(latent_dim, hidden_dim, input_dim, seq_len, num_layers, dropout)

    def forward(self, x):
        return self.decoder(self.encoder(x))


def window_starts(n: int, window_size: int, stride: int,
                  groups: np.ndarray | None = None,
                  valid: np.ndarray | None = None) -> np.ndarray:
    """
    Pencere başlangıç indeksleri.

    `groups` verilirse (ör. `run_id`) pencereler asla iki koşuya YAYILMAZ — kesintisiz
    500 Hz koşulara bölünmüş veride bu şart, yoksa LSTM iki ayrı kayıt oturumunu tek
    zaman serisi sanır. `valid` verilirse (ör. Savitzky-Golay kenar payı) içinde
    geçersiz örnek bulunan pencereler elenir.
    """
    if groups is None:
        segs = [(0, n)]
    else:
        g = np.asarray(groups)
        edges = np.flatnonzero(np.diff(g) != 0) + 1
        b = np.concatenate([[0], edges, [n]])
        segs = [(int(a), int(c - a)) for a, c in zip(b[:-1], b[1:])]

    out = []
    for a, L in segs:
        if L < window_size:
            continue
        out.append(a + np.arange(0, L - window_size + 1, stride))
    if not out:
        return np.empty(0, dtype=np.int64)
    starts = np.concatenate(out).astype(np.int64)

    if valid is not None:
        v = np.asarray(valid, dtype=bool)
        cs = np.concatenate([[0], np.cumsum(v)])
        keep = (cs[starts + window_size] - cs[starts]) == window_size
        starts = starts[keep]
    return starts


class WindowDataset(Dataset):
    """T uzunluğunda kayan pencere; koşu sınırlarına ve geçerlilik maskesine saygılı."""

    def __init__(self, data: np.ndarray, window_size: int, stride: int,
                 groups: np.ndarray | None = None, valid: np.ndarray | None = None):
        self.data = np.ascontiguousarray(data, dtype=np.float32)
        self.window_size = window_size
        self.stride = stride
        self.starts = window_starts(len(data), window_size, stride, groups, valid)
        self.n_windows = len(self.starts)

    def __len__(self):
        return self.n_windows

    def __getitem__(self, idx):
        s = int(self.starts[idx])
        return torch.from_numpy(self.data[s:s + self.window_size])


# ─────────────────────── Kanal tanımları ───────────────────────

# Ham model: 24 kanal — q, q̇, τ, KTS (her biri 6). TCP konumu DAHİL DEĞİL.
RAW_COLS = (
    [f"q_{j}" for j in range(1, 7)] +
    [f"qd_{j}" for j in range(1, 7)] +
    [f"tau_{j}" for j in range(1, 7)] +
    ["fx", "fy", "fz", "tx", "ty", "tz"]
)

# Kalıntı model: 12 kanal — içsel (r_int) + dışsal (r_ext)
RESIDUAL_COLS = [f"r_int_{j}" for j in range(1, 7)] + [f"r_ext_{j}" for j in range(1, 7)]

# Ortak eğitim hiperparametreleri (245.pdf, Bölüm III)
_COMMON = dict(
    window_size=100, stride=25, num_layers=2, dropout=0.15,
    epochs=300, patience=25, batch_size=256, lr=1e-3,
    betas=(0.9, 0.999), grad_clip=1.0,
    plateau_factor=0.5, plateau_patience=8,
    threshold_percentile=97.0, train_ratio=0.8,
)

PRESETS = {
    "residual": dict(_COMMON, feature_cols=RESIDUAL_COLS, features=12,
                     hidden_dim=128, latent_dim=32,
                     expected_params=478_892,
                     paper_epochs=129, paper_val_loss=0.181, paper_threshold=0.420,
                     paper_auc=0.908, paper_pr_auc=0.695, paper_best_f1=0.692,
                     default_parquet="ur10e_features.parquet",
                     default_model_dir="residual_ae_v2"),
    "raw": dict(_COMMON, feature_cols=RAW_COLS, features=24,
                hidden_dim=256, latent_dim=64,
                expected_params=1_907_032,
                paper_epochs=87, paper_val_loss=0.320, paper_threshold=0.854,
                paper_auc=0.952, paper_pr_auc=0.761, paper_best_f1=0.698,
                default_parquet="ur10e_features.parquet",
                default_model_dir="raw_ae_v2"),
}
PRESETS["hybrid"] = PRESETS["residual"]        # eski isimle uyumluluk


def build(meta: dict) -> LSTMAutoencoder:
    return LSTMAutoencoder(
        input_dim=meta["features"], hidden_dim=meta["hidden_dim"],
        latent_dim=meta["latent_dim"], seq_len=meta["window_size"],
        num_layers=meta["num_layers"], dropout=meta.get("dropout", 0.0),
    )


def export_onnx(model: LSTMAutoencoder, meta: dict, path) -> None:
    """Canlı çıkarım için ONNX. Dinamik batch ekseni; sekans uzunluğu sabit."""
    model = model.eval().cpu()
    dummy = torch.zeros(1, meta["window_size"], meta["features"], dtype=torch.float32)
    torch.onnx.export(
        model, dummy, str(path),
        input_names=["window"], output_names=["recon"],
        dynamic_axes={"window": {0: "batch"}, "recon": {0: "batch"}},
        opset_version=17, do_constant_folding=True,
    )


# ────────────────── Bildiriye karşı doğrulama ──────────────────

if __name__ == "__main__":
    print(f"{'model':<12}{'kanal':>7}{'gizli/saklı':>13}{'parametre':>12}"
          f"{'bildiri':>12}{'':>5}")
    print("-" * 62)
    allok = True
    for name in ("residual", "raw"):
        m = PRESETS[name]
        n = sum(p.numel() for p in build(m).parameters())
        ok = n == m["expected_params"]
        allok &= ok
        print(f"{name:<12}{m['features']:>7}{m['hidden_dim']:>8}/{m['latent_dim']:<4}"
              f"{n:>12,}{m['expected_params']:>12,}{'  ✅' if ok else '  ❌':>5}")
    print("-" * 62)
    print("Parametre sayıları 245.pdf ile birebir uyuşuyor." if allok
          else "UYUŞMAZLIK VAR — mimari kontrol edilmeli.")
