"""
inject_faults.py
================
245.pdf Bölüm II.D — dört sentetik arıza senaryosu.

Gerçek zemin doğrusu etiketi olmadığı için değerlendirme, kontrollü arıza
enjeksiyonuyla yapılır. Arıza her iki temsil uzayına (ham ve kalıntı) kendi
genliğiyle ayrı ayrı enjekte edilir.

    (i)   Motor kayması   : Eklem 3 (dirsek) torkuna doğrusal rampa
                            ham 15 Nm · kalıntı 25 Nm · %30–38
    (ii)  Çarpışma        : tüm KTS kanallarına Gauss darbe
                            ham 30 N · kalıntı 40 N · merkez %65, genişlik %4
    (iii) Gizyazar hatası : Eklem 5 (bilek 2) pozisyonuna basamak
                            ham 1,5 rad · kalıntı 8 Nm · başlangıç %92
    (iv)  Sensör gürültüsü: tüm KTS kanallarına Gauss gürültü
                            ham 3,5 N · kalıntı 0,08 Nm · %15–23

Sensör gürültüsü senaryosu, iki model arasındaki tamamlayıcılığı sınamak için
kasıtlı olarak yalnızca ham sinyali belirgin etkileyecek genlikte ayarlanmıştır.

Pencere etiketi: pencere örneklerinin en az %10'u arıza maskesiyle örtüşüyorsa
o pencere anomali sayılır.
"""

from __future__ import annotations

import numpy as np

# Her senaryo: maske aralığı (oran) + her uzay için (kanal seçici, genlik)
FAULTS = {
    "motor_kaymasi": {
        "tr": "Motor kayması", "en": "motor drift",
        "kind": "ramp",
        "span": (0.30, 0.38),
        "raw": {"cols": ["tau_3"], "amp": 15.0},          # Nm, Eklem 3 torku
        "residual": {"cols": ["r_int_3"], "amp": 25.0},   # Nm, içsel kalıntı
    },
    "carpisma": {
        "tr": "Çarpışma", "en": "collision",
        "kind": "gauss_pulse",
        "center": 0.65, "width": 0.04,
        "raw": {"cols": ["fx", "fy", "fz", "tx", "ty", "tz"], "amp": 30.0},
        "residual": {"cols": [f"r_ext_{j}" for j in range(1, 7)], "amp": 40.0},
    },
    "gizyazar_hatasi": {
        "tr": "Gizyazar hatası", "en": "encoder glitch",
        "kind": "step",
        "span": (0.92, 1.00),
        "raw": {"cols": ["q_5"], "amp": 1.5},             # rad, Eklem 5 pozisyonu
        "residual": {"cols": ["r_int_5"], "amp": 8.0},    # Nm
    },
    "sensor_gurultusu": {
        "tr": "Sensör gürültüsü", "en": "sensor noise",
        "kind": "gauss_noise",
        "span": (0.15, 0.23),
        "raw": {"cols": ["fx", "fy", "fz", "tx", "ty", "tz"], "amp": 3.5},
        "residual": {"cols": [f"r_ext_{j}" for j in range(1, 7)], "amp": 0.08},
    },
}


def fault_span(cfg: dict) -> tuple[float, float]:
    """Senaryonun maske aralığını (başlangıç, bitiş) oran olarak verir."""
    if cfg["kind"] == "gauss_pulse":
        return (cfg["center"] - cfg["width"], cfg["center"] + cfg["width"])
    return cfg["span"]


def as_segments(n: int, segments: list[tuple[int, int]] | None
                ) -> list[tuple[int, int]]:
    """
    Arızanın enjekte edileceği (başlangıç, uzunluk) parçaları.

    `segments is None` ESKİ davranıştır: tüm veri seti TEK bir parça sayılır, yani
    her senaryonun veri setinde tek bir bitişik arıza olayı olur. Bildirinin
    protokolü budur ve erratum yeniden üretimi için korunmalıdır — ama bir
    genelleme ölçümü için kullanılamaz: senaryo başına etkin bağımsız olay sayısı
    41.688 pencerede 4'tür, ve maskenin veri setindeki SABİT konumu arıza tipini
    kaydın konumuyla eşleştirir (ör. %92–100 aralığındaki gizyazar hatası yalnız
    son dilime düşer). Koşu bazlı enjeksiyon için `segments` verilir.
    """
    return [(0, n)] if segments is None else list(segments)


def fault_mask(n: int, cfg: dict,
               segments: list[tuple[int, int]] | None = None) -> np.ndarray:
    """Arıza maskesi. `segments` verilirse her parça KENDİ içinde oranlanır."""
    a, b = fault_span(cfg)
    m = np.zeros(n, dtype=bool)
    for s0, L in as_segments(n, segments):
        m[s0 + int(a * L): s0 + int(b * L)] = True
    return m


def build_scale(cols: list[str], a: list[float] | None) -> dict[str, float]:
    """
    Bildirideki genlikler Nm cinsinden; kanallarımız ise ölçüm (akım) uzayında.
    Tork kaynaklı kanallar için Nm → akım dönüşümü a_j ile yapılır.
    Konum (rad) ve KTS (N) kanalları zaten fiziksel birimde, dokunulmaz.

    a = None ise (artıklar Nm cinsindense) hiçbir dönüşüm uygulanmaz.
    """
    sc = {c: 1.0 for c in cols}
    if a is None:
        return sc
    for j in range(1, 7):
        for pre in ("tau_", "r_int_", "r_ext_", "r_total_"):
            k = f"{pre}{j}"
            if k in sc:
                sc[k] = float(a[j - 1])
    return sc


def inject(data: np.ndarray, cols: list[str], cfg: dict, space: str,
           rng: np.random.Generator | None = None,
           scale: dict[str, float] | None = None,
           segments: list[tuple[int, int]] | None = None) -> np.ndarray:
    """
    `data` (N, D) kopyasına arızayı enjekte eder ve yeni diziyi döndürür.

    `cols` parquet kanal isimleri; `space` "raw" veya "residual".
    `scale` kanal başına genlik çarpanı (Nm → ölçüm uzayı); bkz. build_scale().
    `segments` verilirse arıza HER parçaya ayrı ayrı, o parçanın kendi uzunluğuna
    oranlanarak enjekte edilir; verilmezse tüm dizi tek parça sayılır (eski
    davranış, bkz. as_segments).
    """
    rng = rng or np.random.default_rng(0)
    out = data.copy()
    n = len(out)
    spec = cfg[space]
    names = [c for c in spec["cols"] if c in cols]
    idx = [cols.index(c) for c in names]
    if not idx:
        raise KeyError(f"{space}: {spec['cols']} kanalları bulunamadı")
    sc = np.array([1.0 if scale is None else scale.get(c, 1.0) for c in names])
    amp = spec["amp"]
    a, b = fault_span(cfg)

    for s0, L in as_segments(n, segments):
        s, e = s0 + int(a * L), s0 + int(b * L)

        if cfg["kind"] == "ramp":
            if e > s:
                out[s:e, idx] += np.linspace(0.0, amp, e - s)[:, None] * sc

        elif cfg["kind"] == "step":
            # Basamak parçanın SONUNA kadar sürer; parça sınırını aşmaz, yoksa
            # bir koşuya enjekte edilen arıza komşu koşuyu da kirletir.
            out[s:s0 + L, idx] += amp * sc

        elif cfg["kind"] == "gauss_pulse":
            c = s0 + cfg["center"] * L
            sigma = cfg["width"] * L / 2.0       # maske ±2σ'yı kapsar
            t = np.arange(s0, s0 + L, dtype=np.float64)
            pulse = amp * np.exp(-0.5 * ((t - c) / sigma) ** 2)
            out[s0:s0 + L, idx] += pulse[:, None] * sc

        elif cfg["kind"] == "gauss_noise":
            if e > s:
                out[s:e, idx] += rng.normal(0.0, amp, size=(e - s, len(idx))) * sc

        else:
            raise ValueError(cfg["kind"])

    return out


def window_labels(n_samples: int, cfg: dict, window: int, stride: int,
                  overlap: float = 0.10,
                  segments: list[tuple[int, int]] | None = None,
                  starts: np.ndarray | None = None) -> np.ndarray:
    """Pencerenin >= %overlap'i arıza maskesine denk geliyorsa anomali."""
    m = fault_mask(n_samples, cfg, segments).astype(np.float64)  # float32 riskli
    cs = np.concatenate([[0.0], np.cumsum(m)])
    if starts is None:
        n_win = (n_samples - window) // stride + 1
        starts = np.arange(n_win) * stride
    starts = np.asarray(starts)
    return ((cs[starts + window] - cs[starts]) / window) >= overlap


if __name__ == "__main__":
    N = 1_124_432
    W, S = 100, 25
    nw = (N - W) // S + 1
    print(f"N={N:,}  W={W} S={S}  → pencere {nw:,}   ×4 senaryo = {4*nw:,}  (bildiri 179.896)")
    tot = 0
    print(f"\n{'senaryo':<20}{'tip':<14}{'aralık':<16}{'arıza pencere':>14}{'oran':>8}")
    print("-" * 74)
    for k, c in FAULTS.items():
        y = window_labels(N, c, W, S)
        tot += int(y.sum())
        a, b = fault_span(c)
        print(f"{c['tr']:<20}{c['kind']:<14}%{100*a:.0f}–%{100*b:.0f}{'':<8}"
              f"{y.sum():>14,}{100*y.mean():>7.1f}%")
    print("-" * 74)
    print(f"{'TOPLAM':<20}{'':<14}{'':<16}{tot:>14,}  (bildiri 14.402)")
