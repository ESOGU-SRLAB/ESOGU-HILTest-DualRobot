#!/usr/bin/env python3
"""
makale_figurler.py
==================
ESOGÜ MMF makalesinin şekillerini üretir -> `makale_figurleri/sekil*.png`.

    python3 makale_figurler.py

Stil, anomali makalesinin şekilleriyle aynı tutuldu: çerçeveli panel, kalın
çizgi, 9-10,5 pt punto, 300 dpi. Dergi iki sütunlu ve gövde Cambria olduğu için
şekil yazıları serif (Liberation Serif) seçildi; sayfa üzerinde metinle aynı
aileden görünür.

Şekil 5 ve 6, gerçek hücrede 20 Ağustos 2026'da çekilen `figures/
gemini_pick_place.mp4` kaydından üretilir; kayıt yoksa o iki şekil atlanır ve
makale onlarsız derlenir (Builder eksik şekli uyarıyla geçer).
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle

HERE = Path(__file__).resolve().parent
OUT = HERE / "makale_figurleri"
RAPOR_FIG = HERE / "figures"
VIDEO = RAPOR_FIG / "gemini_pick_place.mp4"

OUT.mkdir(exist_ok=True)

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Liberation Serif", "DejaVu Serif"],
    "font.size": 9.5,
    "axes.linewidth": 1.2,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
})

INK = "#111111"
ACC = "#1E517B"     # vurgu (mavi)
BAD = "#B03A2E"     # hata / reddedilen
GOOD = "#1E7A46"    # kabul edilen
GREY = "#8A8F98"


def _kaydet(fig, ad: str) -> None:
    yol = OUT / ad
    fig.savefig(yol, facecolor="white")
    plt.close(fig)
    print(f"  {ad}")


# ──────────────────────────────────────────────────────────────────────
# Şekil 1 — işlem hattı
# ──────────────────────────────────────────────────────────────────────
def sekil1_mimari() -> None:
    """Derinlikten kavrama pozuna giden zincir.

    Şeklin anlatması gereken tek şey şu: modelden çıkan bilgi 2B bir noktadır
    ve poz o noktadan SONRA, ölçümle kurulur. Bu yüzden ER 2 kutusu zincirin
    ortasında durur, sonunda değil.
    """
    fig, ax = plt.subplots(figsize=(7.0, 2.75))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 40)
    ax.axis("off")

    kutular = [
        (2.0, 21.5, 16.0, "ToF derinlik\n(SICK V-T Mini)", "#EDF1F5"),
        (20.5, 21.5, 13.5, "kabartma\nrender", "#EDF1F5"),
        (36.5, 21.5, 16.0, "Gemini\nRobotics ER 2", "#DCE8F2"),
        (55.0, 21.5, 15.5, "2B nokta\n$(u,v)$, 0–1000", "#DCE8F2"),
        (75.5, 21.5, 22.5, "doğal dil komutu\n(operatör)", "#F5F0E6"),
        (2.0, 3.5, 18.0, "ışın bandı ile\nyama ayıklama", "#E8F3EC"),
        (22.5, 3.5, 19.0, "SVD düzlem +\ndüzlemsellik kapısı", "#E8F3EC"),
        (44.0, 3.5, 17.5, "normal → poz\n(TF, world)", "#E8F3EC"),
        (64.0, 3.5, 34.0, "MoveIt 2 → UR10e + VGC10 vakum", "#F3E6E6"),
    ]
    for x, y, w, etiket, renk in kutular:
        ax.add_patch(FancyBboxPatch(
            (x, y), w, 10.5, boxstyle="round,pad=0.30,rounding_size=1.2",
            linewidth=1.6, edgecolor=INK, facecolor=renk))
        ax.text(x + w / 2, y + 5.25, etiket, ha="center", va="center",
                fontsize=8.8, linespacing=1.35)

    def ok(x1, y1, x2, y2, renk=INK, lw=1.8, ls="-"):
        ax.add_patch(FancyArrowPatch(
            (x1, y1), (x2, y2), arrowstyle="-|>", mutation_scale=13,
            linewidth=lw, color=renk, linestyle=ls, shrinkA=0, shrinkB=0))

    # üst sıra: derinlik → render → model → nokta
    ok(18.3, 26.75, 20.2, 26.75)
    ok(34.3, 26.75, 36.2, 26.75)
    ok(52.8, 26.75, 54.7, 26.75)
    # Komut, GÖRÜNTÜYLE BİRLİKTE modele girer - çıktı noktasına değil. Ok bu
    # yüzden kutuların üstünden dolaşıp ER 2'nin tepesine iner.
    ax.add_patch(FancyArrowPatch(
        (86.5, 32.2), (44.5, 32.2), arrowstyle="-|>", mutation_scale=12,
        linewidth=1.5, color=GREY, linestyle=(0, (4, 2)),
        shrinkA=0, shrinkB=0, connectionstyle="arc3,rad=0.20"))
    ax.text(65.5, 36.4, "metin", ha="center", fontsize=8.4, color=GREY)

    # 2B noktadan ölçüm zincirine dönüş: makalenin bütün konusu bu ok
    ax.add_patch(FancyArrowPatch(
        (60.0, 21.5), (13.0, 14.5), arrowstyle="-|>", mutation_scale=14,
        linewidth=2.0, color=ACC, shrinkA=0, shrinkB=0,
        connectionstyle="arc3,rad=0.20"))
    ax.text(37.0, 15.9, "yalnızca piksel — poz YOK", ha="center",
            fontsize=8.8, color=ACC, style="italic")

    # alt sıra: ölçüm zinciri
    ok(20.3, 8.75, 22.3, 8.75)
    ok(41.8, 8.75, 43.8, 8.75)
    ok(61.8, 8.75, 63.8, 8.75)

    # nokta bulutu, derinlikten doğrudan yamaya
    ax.add_patch(FancyArrowPatch(
        (5.0, 21.5), (5.0, 14.5), arrowstyle="-|>", mutation_scale=13,
        linewidth=1.6, color=GREY, shrinkA=0, shrinkB=0))
    ax.text(6.4, 17.4, "nokta bulutu", fontsize=8.4, color=GREY)

    ax.add_patch(Rectangle((0.2, 0.2), 99.6, 39.6, fill=False,
                           edgecolor=INK, linewidth=1.3))
    _kaydet(fig, "sekil1_mimari.png")


# ──────────────────────────────────────────────────────────────────────
# Şekil 2 — render karşılaştırması
# ──────────────────────────────────────────────────────────────────────
def sekil2_render() -> None:
    """Aynı sahnenin iki render'ı.

    Normal gölgelemesinde kutunun YÜZÜ ile bandın yüzü aynı yöne bakar, yani
    aynı griye boyanır; görünen tek şey kenar. Kabartma render'da yükseklik
    doğrudan renge gittiği için yüz ayrı bir bloğa dönüşür.
    """
    a = RAPOR_FIG / "fig_conveyor_normals.png"
    b = RAPOR_FIG / "fig_conveyor_relief.png"
    if not (a.exists() and b.exists()):
        print("  ! sekil2 atlandı (kaynak render yok)")
        return

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 3.05))
    for ax, yol, etiket in ((axes[0], a, "(a) normal gölgeleme"),
                            (axes[1], b, "(b) kabartma (relief)")):
        ax.imshow(plt.imread(yol))
        ax.set_xticks([])
        ax.set_yticks([])
        for kenar in ax.spines.values():
            kenar.set_linewidth(1.4)
            kenar.set_edgecolor(INK)
        ax.set_xlabel(etiket, fontsize=9.5, labelpad=5)

    # kutunun yerini iki panelde de göster
    for ax in axes:
        ax.add_patch(Rectangle((318, 262), 130, 82, fill=False,
                               edgecolor="#E67E22", linewidth=1.8))
    fig.subplots_adjust(wspace=0.06)
    _kaydet(fig, "sekil2_render.png")


# ──────────────────────────────────────────────────────────────────────
# Şekil 3 — yama, ışın bandı ve düzlem
# ──────────────────────────────────────────────────────────────────────
def sekil3_yama() -> None:
    """Bandın ne işe yaradığı.

    Sol panel bandsız durumu gösterir: yamaya arka plan karışır ve SVD, iki
    yüzeyin ORTASINA bakan bir normal döndürür. Sağ panelde aynı yamadan
    bakış ışını boyunca bir bant kesilir, arka plan düşer ve normal yüzeyin
    kendi normali olur.
    """
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.95))
    rng = np.random.default_rng(11)

    # Temsilî yama: 12 px yarıçaplı bir diskin yan kesiti. Üst yüz + yamanın
    # kenarına giren birkaç arka plan (bant) noktası. Oranlar ölçülen değerleri
    # yeniden üretmek için değil, mekanizmayı göstermek için seçildi; ölçülen
    # sayılar makalenin tablosundadır.
    half, step = 0.021, 0.029
    x_ust = rng.uniform(-half, half * 0.55, 420)
    y_ust = 0.845 + rng.normal(0, 0.0006, x_ust.size)
    x_arka = rng.uniform(half * 0.62, half, 10)
    y_arka = 0.845 - step + rng.normal(0, 0.0006, x_arka.size)

    for ax, bantli in zip(axes, (False, True)):
        if bantli:
            kx, ky = x_ust, y_ust
            ax.scatter(x_arka, y_arka, s=16, facecolor="none",
                       edgecolor=GREY, linewidth=1.0, zorder=3)
        else:
            kx = np.concatenate([x_ust, x_arka])
            ky = np.concatenate([y_ust, y_arka])
            ax.scatter(x_arka, y_arka, s=26, color=BAD, zorder=5)
        ax.scatter(x_ust, y_ust, s=5, color=ACC, zorder=4)

        # SVD düzlem
        P = np.column_stack([kx, ky])
        c = P.mean(axis=0)
        _, _, vh = np.linalg.svd(P - c, full_matrices=False)
        n = vh[1]
        if n[1] < 0:
            n = -n
        egim = math.degrees(math.atan2(abs(n[0]), abs(n[1])))
        t = np.array([-n[1], n[0]])
        s = np.linspace(-0.040, 0.040, 2)
        ax.plot(c[0] + t[0] * s, c[1] + t[1] * s, color=INK, linewidth=2.2,
                zorder=5)
        ax.add_patch(FancyArrowPatch(
            (c[0], c[1]), (c[0] + n[0] * 0.020, c[1] + n[1] * 0.020),
            arrowstyle="-|>", mutation_scale=14, linewidth=2.4,
            color=BAD if not bantli else GOOD, zorder=6))

        if bantli:
            ax.axhspan(0.845 - 0.025, 0.845 + 0.025, color="#E8F3EC",
                       zorder=1)
            ax.text(0.0285, 0.8665, "bant $\\pm\\,\\delta$", fontsize=8.6,
                    color=GOOD, va="center", ha="right")

        # Eşit en-boy olmadan normal oku ile düzlem çizgisi arasındaki açı
        # ekranda YANLIŞ görünür; şeklin bütün anlattığı o açı olduğu için
        # burada eşit ölçek zorunlu.
        ax.set_xlim(-0.028, 0.030)
        ax.set_ylim(0.8125, 0.8705)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("yama içinde yanal konum (m)", fontsize=9.2)
        ax.tick_params(labelsize=8.4)
        for kenar in ax.spines.values():
            kenar.set_linewidth(1.2)
        baslik = ("(a) bantsız: düzlem iki yüzeye birden oturuyor "
                  "($\\theta \\approx %.0f^\\circ$)" % egim if not bantli
                  else "(b) ışın bandı ile: yüzeyin kendi normali "
                       "($\\theta \\approx %.1f^\\circ$)" % egim)
        ax.set_title(baslik, fontsize=8.8, pad=6)

    axes[0].set_ylabel("yükseklik (m)", fontsize=9.2)
    axes[1].set_yticklabels([])
    fig.subplots_adjust(wspace=0.08)
    _kaydet(fig, "sekil3_yama.png")


# ──────────────────────────────────────────────────────────────────────
# Şekil 4 — eğik normalin bedeli
# ──────────────────────────────────────────────────────────────────────
def sekil4_egim() -> None:
    """Düzlem eğiminin yaklaşma pozunu ne kadar kaydırdığı.

    Yaklaşma pozu temas noktasından ölçülen normal boyunca `a` kadar geride
    kurulur; normal $\\theta$ kadar eğikse poz yanal olarak $a\\tan\\theta$
    kaydırılır. Temas noktası kaymaz, o yüzden belirti yalnız inişte görünür.
    """
    fig, ax = plt.subplots(figsize=(7.0, 2.75))
    theta = np.linspace(0, 20, 400)

    for a, ls, lw in ((0.15, "-", 2.6), (0.10, "--", 2.0), (0.06, ":", 2.0)):
        ax.plot(theta, 1000 * a * np.tan(np.radians(theta)), ls,
                linewidth=lw, color=ACC if a == 0.15 else GREY,
                label=f"$a$ = {int(a*100)} cm")

    # ölçülen iki kirlenme noktası
    olcum = [(8.0, 21.0, "%4 kirlenme\nMAD 2,9 / RMS 5,3 mm"),
             (10.9, 29.0, "%6 kirlenme\nMAD 4,0 / RMS 6,2 mm")]
    for th, mm, etiket in olcum:
        ax.plot([th], [mm], "o", markersize=7.5, color=BAD, zorder=6)
        ax.annotate(etiket, (th, mm), textcoords="offset points",
                    xytext=(-8, 16), ha="right", fontsize=8.4, color=BAD,
                    linespacing=1.3)

    ax.axvline(15.0, color=GOOD, linewidth=1.8, linestyle=(0, (5, 2)))
    ax.text(15.3, 4, "oturtma eşiği\n$\\theta_{snap}=15^\\circ$", fontsize=8.6,
            color=GOOD, linespacing=1.3)

    ax.set_xlabel("düzlem normalinin eğimi $\\theta$ (derece)", fontsize=9.5)
    ax.set_ylabel("yaklaşma pozunun\nyanal kayması (mm)", fontsize=9.5)
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 58)
    ax.grid(alpha=0.28, linewidth=0.8)
    ax.legend(fontsize=8.8, loc="upper left", frameon=True, framealpha=1.0,
              edgecolor=INK)
    ax.tick_params(labelsize=8.6)
    for kenar in ax.spines.values():
        kenar.set_linewidth(1.2)
    _kaydet(fig, "sekil4_egim.png")


# ──────────────────────────────────────────────────────────────────────
# Şekil 5 — zincirin RViz'deki çıktısı: temas küresi, normal oku, etiket
# ──────────────────────────────────────────────────────────────────────
# Kayıttaki RViz penceresi ekranın sol üst köşesindedir; 3B görünüm alanı
# (x 545..795, y 95..330) küçük olduğu için kare LANCZOS ile büyütülür.
# t = 13,0 s: operatör görünümü kavrama noktasına yakınlaştırmıştır, yani
# marker'ların en okunaklı olduğu andır. markers.py'ye göre marker takımı
# şudur: 4 cm'lik CAMGÖBEĞİ küre = kolun gerçekte gideceği temas noktası,
# TURUNCU ok = ölçülen yüzey normali, BEYAZ yazı = etiket + düzlemsellik
# kalıntısı (mm). RViz görünüm alanı dar olduğu için yazının sonu kadrajın
# dışında kalmıştır; bu yüzden metinde sayısal değer alıntılanmaz.
_MARKER_T = 13.0
_MARKER_KUTU = (95, 330, 545, 795)   # y0, y1, x0, x1


def sekil5_markerlar() -> None:
    if not VIDEO.exists():
        print("  ! sekil5 atlandı (video yok)")
        return
    try:
        import cv2
    except ImportError:
        print("  ! sekil5 atlandı (cv2 yok)")
        return

    cap = cv2.VideoCapture(str(VIDEO))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(_MARKER_T * fps))
    ok, kare = cap.read()
    cap.release()
    if not ok:
        print("  ! sekil5 atlandı (kare okunamadı)")
        return

    y0, y1, x0, x1 = _MARKER_KUTU
    kare = kare[y0:y1, x0:x1]
    kare = cv2.resize(kare, None, fx=4, fy=4,
                      interpolation=cv2.INTER_LANCZOS4)
    rgb = cv2.cvtColor(kare, cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]

    fig, ax = plt.subplots(figsize=(3.30, 3.30 * h / w))
    ax.imshow(rgb)
    ax.set_xticks([])
    ax.set_yticks([])
    for kenar in ax.spines.values():
        kenar.set_linewidth(1.2)
        kenar.set_edgecolor(INK)

    # Görüntü koordinatları 4x büyütülmüş kırpma içindir.
    ok_stil = dict(arrowstyle="-", lw=0.9, color=INK,
                   shrinkA=0, shrinkB=2)
    kutu = dict(boxstyle="round,pad=0.18", fc="white", ec=INK, lw=0.6,
                alpha=0.94)
    # Çapa noktaları kırpılmış karede elle ölçüldü (genişlik/yükseklik oranı).
    isaretler = [
        ((0.558, 0.338), (0.045, 0.520), "temas noktası\n(camgöbeği küre)"),
        ((0.560, 0.268), (0.660, 0.360), "ölçülen yüzey\nnormali (turuncu ok)"),
        ((0.548, 0.212), (0.045, 0.055), "etiket + düzlemsellik\nkalıntısı"),
        ((0.480, 0.348), (0.560, 0.680), "parçanın üst yüzeyinin\nnokta bulutu"),
    ]
    for (hx, hy), (tx, ty), yazi in isaretler:
        ax.annotate(yazi, xy=(hx * w, hy * h), xytext=(tx * w, ty * h),
                    fontsize=6.9, color=INK, ha="left", va="center",
                    bbox=kutu, arrowprops=ok_stil)

    _kaydet(fig, "sekil5_markerlar.png")


# ──────────────────────────────────────────────────────────────────────
# Şekil 6 — gerçek hücrede görev: dört AYRI kare
# ──────────────────────────────────────────────────────────────────────
# Kayıttaki ekran görüntüsünde terminal/RViz penceresi sol üsttedir: y < 435
# satırlarında hücre görüntüsü ancak x > 780'den sonra başlar. Kırpma kutuları
# bu sınıra uyar. Dördünün de en/boy oranı 0,84'tür (dik): Şekil 6 tam sayfa
# yerleşiyor, 2x2 ızgarada sütun genişliği sabit ve bu oran iki satırı yazı
# alanının boyuna oturtuyor. Oran panelden panele değişirse satır yükseklikleri
# tutmaz ve aralarında bant bant boşluk kalır.
#
# Zamanlar kayıttan tek tek doğrulandı (0,5 s adımla tarandı): kap 66,5-68,5 s
# arasında parçanın üstünde duruyor, parça 69,0 s'de kalkmaya başlıyor.
#
# Bu şeklin panelleri TEK BİR resim olarak birleştirilmez; dördü ayrı dosyaya
# yazılır ve makalede kenarlıksız bir 2x2 tabloya tek tek gömülür. Böylece
# yazar Word'de her kareyi ayrı ayrı değiştirebilir, kırpabilir, kaydırabilir.
_KARELER = [
    ("sekil6a_scan.png", 28.0, (440, 1080, 689, 1227),
     "(a) SCAN[toolkit] pozu, ER 2 sorgusu    t = 28 s"),
    ("sekil6b_temas.png", 68.0, (300, 1080, 1185, 1840),
     "(b) TOUCH: kap parçanın üstünde    t = 68 s"),
    ("sekil6c_kalkis.png", 70.0, (300, 1080, 1185, 1840),
     "(c) vakum kuruldu, parça banttan kalktı    t = 70 s"),
    ("sekil6d_birakma.png", 101.0, (100, 880, 1230, 1885),
     "(d) parça, üstten açık gözün üstünde    t = 101 s"),
]

# Kare kenarına ince bir çerçeve çizilir; baskıda panelin nerede bittiği
# görünsün diye. Yazar kareyi değiştirirse çerçeve de gider, sorun değil.
_CERCEVE_PX = 3


def sekil6_kareler() -> None:
    """Şekil 6'nın dört karesini AYRI dosyalara yazar."""
    if not VIDEO.exists():
        print("  ! sekil6 atlandı (video yok)")
        return
    try:
        import cv2
    except ImportError:
        print("  ! sekil6 atlandı (cv2 yok)")
        return

    cap = cv2.VideoCapture(str(VIDEO))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    for ad, t, (y0, y1, x0, x1), _etiket in _KARELER:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(t * fps))
        ok, kare = cap.read()
        if not ok:
            print(f"  ! {ad} atlandı (kare okunamadı)")
            continue
        kare = kare[y0:y1, x0:x1]
        k = _CERCEVE_PX
        kare = cv2.copyMakeBorder(kare, k, k, k, k, cv2.BORDER_CONSTANT,
                                  value=(0x11, 0x11, 0x11))
        cv2.imwrite(str(OUT / ad), kare)
        print(f"  {ad}")
    cap.release()


# ──────────────────────────────────────────────────────────────────────
# Şekil 7 — görev zaman çizelgesi
# ──────────────────────────────────────────────────────────────────────
# Aşama sınırları, kaydın kendi düğüm loglarındaki EPOCH damgalarından okundu
# (gemini_pick_place.mp4, gerçek hücre, 20 Ağustos 2026). Damgalar
# 1787219765,7 çıkarılarak görev saatine çevrildi; bu sabit, kayıttaki en geç
# "son satır" damgasından belirlendi, dolayısıyla mutlak sıfır ±0,5 s
# belirsizdir - aşama SÜRELERİ ise damga farkı olduğu için tamdır.
_ASAMALAR = [
    ("yer hedefi sorgusu: ER 2 bulamadı", 3.4, 9.6, "#DCE8F2"),
    ("SCAN[toolkit] pozuna geçiş (+ sarma açma)", 9.6, 21.9, "#D9E2EC"),
    ("yer hedefi için render + ER 2 çağrısı", 21.9, 36.7, "#DCE8F2"),
    ("APPROACH_PICK ve TOUCH inişi", 36.7, 68.6, "#E8F3EC"),
    ("vakumun kurulması", 68.6, 68.8, "#CFE6D8"),
    ("LIFT_TO_APPROACH", 68.8, 73.2, "#E8F3EC"),
    ("APPROACH_PLACE (1. deneme başarısız)", 73.2, 100.2, "#E8F3EC"),
    ("RELEASE", 100.2, 103.4, "#CFE6D8"),
    ("RETREAT", 103.4, 105.5, "#E8F3EC"),
    ("HOME pozuna dönüş", 105.5, 117.2, "#D9E2EC"),
]


def sekil7_zaman() -> None:
    fig, ax = plt.subplots(figsize=(7.0, 2.9))
    for i, (ad, t0, t1, renk) in enumerate(_ASAMALAR):
        y = len(_ASAMALAR) - 1 - i
        sure = t1 - t0
        ax.barh(y, max(sure, 0.45), left=t0, height=0.64, color=renk,
                edgecolor=INK, linewidth=1.3)
        ax.text(t1 + 1.6, y, f"{sure:.1f} s".replace(".", ","), va="center",
                fontsize=8.4, color=INK)
    ax.set_yticks(range(len(_ASAMALAR)))
    ax.set_yticklabels([ad for ad, *_ in reversed(_ASAMALAR)], fontsize=8.5)
    ax.set_xlabel("ilk tespitten itibaren geçen görev süresi (s)", fontsize=9.5)
    ax.set_xlim(0, 132)
    ax.tick_params(labelsize=8.6)
    ax.grid(axis="x", alpha=0.28, linewidth=0.8)
    ax.set_axisbelow(True)
    for kenar in ax.spines.values():
        kenar.set_linewidth(1.2)
    _kaydet(fig, "sekil7_zaman.png")


def main() -> None:
    print(f"şekiller -> {OUT}")
    sekil1_mimari()
    sekil2_render()
    sekil3_yama()
    sekil4_egim()
    sekil5_markerlar()
    sekil6_kareler()
    sekil7_zaman()
    print("bitti")


if __name__ == "__main__":
    main()
