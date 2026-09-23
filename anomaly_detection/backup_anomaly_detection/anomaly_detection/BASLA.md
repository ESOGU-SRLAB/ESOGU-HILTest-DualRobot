# UR10e Anomali Tespiti — Ubuntu Çalıştırma Kılavuzu

245.pdf'teki **ikili LSTM özkodlayıcı birleşimi** çalışmasını sıfırdan, çalışan bir
ters dinamik modeliyle yeniden üretir.

Adımları **sırayla** uygula. Her adımın sonunda ne görmen gerektiğini yazdım;
tutmuyorsa devam etme.

---

## Neden sıfırdan yapıyoruz

Mevcut `ur10e_hybrid_residual.parquet` geçersiz: τ_model satırların **%100'ünde tam
sıfır** çıkıyor. Sebep dinamik modelin kendisi değil — **onu Windows'ta
çalıştıramamak**:

```
resources/ur10_solver_py.cpython-310-x86_64-linux-gnu.so
  → ELF 64-bit, x86-64, Linux, CPython 3.10
Windows'ta Python 3.12 ile:  import ur10_solver_py  →  başarısız
```

`resources/model.py` bu hatayı `except ImportError` ile yutuyor, `self.solver = None`
kalıyor, `_update_outputs` ilk satırda geri dönüyor ve τ = 0 yazılıyor. FMU log'unun
hiç gelmemesi de bundandı.

**Dinamik modele dokunulmadı.** Fizik doğrulanmış; tek yaptığımız onu kendi
ortamında (Linux + Python 3.10) çalıştırmak. Bonus: ROS 2 Humble de Ubuntu 22.04 /
Python 3.10, yani burada doğruladığımız import yolu robota birebir taşınacak.

---

## Adım 0 — Klasörü hazırla

Bu zip'i aç ve **yanına şu dosyaları koy** (Windows bölümünden veya USB ile):

| dosya | nereden |
|---|---|
| `ros-joint-states.csv` | `AnomalyDetection_v1/` (3,7 GB) |
| `UR10e_InverseDynamics.fmu` | `AnomalyDetection_v1/` |
| `modelDescription.xml` | `AnomalyDetection_v1/` |
| `ur10e_jacobian.py` | `AnomalyDetection_v1/` |
| `resources/` (klasör) | `AnomalyDetection_v1/` |
| `binaries/` (klasör) | `AnomalyDetection_v1/` |

Sonuç şöyle görünmeli:

```
~/siu2026/
├── BASLA.md                 ← bu dosya
├── 00_ortam_kontrol.sh
├── requirements.txt
├── fmu_backend.py  check_fmu.py  generate_residuals.py  verify_fmu_residual.py
├── extract_raw_features.py  models.py  train_ae.py  inject_faults.py
├── evaluate_fusion.py  evaluate_ae.py  compare_models.py
├── belgeler/
├── ros-joint-states.csv          ← sen kopyalayacaksın
├── UR10e_InverseDynamics.fmu     ← sen kopyalayacaksın
├── modelDescription.xml          ← sen kopyalayacaksın
├── ur10e_jacobian.py             ← sen kopyalayacaksın
├── resources/                    ← sen kopyalayacaksın
└── binaries/                     ← sen kopyalayacaksın
```

```bash
cd ~/siu2026
chmod +x resources/ur10_solver_py*.so
bash 00_ortam_kontrol.sh
```

Bu script hiçbir şey değiştirmez, sadece teşhis koyar: işletim sistemi, Python
sürümleri, solver'ın import edilip edilmediği, GPU, eksik dosyalar, eksik paketler.
**Çıktısını bana yolla**, birlikte bakalım.

---

## Adım 1 — Python 3.10 (solver için zorunlu)

`00_ortam_kontrol.sh` "python3.10 mevcut" diyorsa bu adımı atla.

Ubuntu 22.04'te `python3` zaten 3.10'dur. 24.04 kullanıyorsan `python3` 3.12'dir ve
solver yüklenmez; 3.10'u yanına kurman gerekir:

```bash
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt update
sudo apt install -y python3.10 python3.10-venv python3.10-dev
```

Sonra solver için ayrı bir sanal ortam:

```bash
python3.10 -m venv ~/venv310
source ~/venv310/bin/activate
pip install --upgrade pip
pip install numpy pandas pyarrow scipy fmpy pyzmq
deactivate
```

> Bundan sonra **Adım 3–5** (`check_fmu`, `generate_residuals`, `verify`) bu ortamda,
> **Adım 6–7** (eğitim) normal `python3` ile çalışacak. Kılavuzda hangi yorumlayıcının
> kullanılacağı her komutta belirtildi.

---

## Adım 2 — Bağımlılıklar (eğitim ortamı)

```bash
cd ~/siu2026
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
python3 -m pip install torch --index-url https://download.pytorch.org/whl/cu124
python3 -c "import torch;print(torch.__version__, torch.cuda.is_available())"
```

Son satır `True` demeli. `False` diyorsa NVIDIA sürücüsü eksiktir:

```bash
ubuntu-drivers devices          # önerilen sürücüyü gösterir
sudo ubuntu-drivers autoinstall
sudo reboot
```

---

## Adım 3 — Ham özellikleri çıkar

```bash
python3 extract_raw_features.py --max-chunks 3 --out raw_smoke.parquet
python3 extract_raw_features.py --out ur10e_raw_features.parquet
```

**Beklenen:** `1,124,432 satır, 27 kanal, parse hatası 0`, ~30–60 saniye.

Model bu 27 kanalın **24'ünü** kullanacak (q, q̇, τ, KTS). TCP konumu bildirinin ham
tanımında yok, ileride lazım olur diye dosyada duruyor.

---

## Adım 4 — Sağlık kontrolü ⚠️ EN KRİTİK ADIM

```bash
source ~/venv310/bin/activate        # python3 zaten 3.10 ise bu satırı atla
python3 check_fmu.py --resources resources --fmu UR10e_InverseDynamics.fmu
```

Altı kontrol:

1. **Ortam** — Linux x86-64 / CPython 3.10 uyumu
2. **`so` arka ucu** — `ur10_solver_py` import ediliyor mu, sınıf örnekleniyor mu
3. **`fmu` arka ucu** — fmpy ile FMU örnekleniyor mu
4. **Sıfır testi** — bilinen bir duruşta τ sıfırdan farklı mı
5. **Fizik testi** — kol yatay uzatıldığında yerçekimi: düşey eksenli **Eklem 1'de
   tork ≈ 0**, `shoulder_lift`'te büyük tork. Bu, "sıfır değil ama uydurma" bir
   çıktıyı da yakalar.
6. **Eşdeğerlik** — `so` ve `fmu` yolları 200 rastgele durumda aynı sayıyı veriyor mu

**Bu adım ✅ vermeden kesinlikle devam etme.** Çıktısını bana yolla.

---

## Adım 5 — Artıkları üret

Önce 20.000 satırla iki arka ucu karşılaştır:

```bash
python3 generate_residuals.py --backend so  --limit-rows 20000 --out check_so.parquet
python3 generate_residuals.py --backend fmu --limit-rows 20000 --out check_fmu.parquet
python3 -c "
import pandas as pd, numpy as np
a=pd.read_parquet('check_so.parquet'); b=pd.read_parquet('check_fmu.parquet')
c=[f'tau_model_{j}' for j in range(1,7)]
print('so vs fmu maks fark:', np.abs(a[c].to_numpy()-b[c].to_numpy()).max())"
```

Fark ~0 (≈1e-12) çıkmalı. Çıkarsa tam üretim:

```bash
python3 generate_residuals.py --backend so --out ur10e_hybrid_residual.parquet
```

Fark ~0 **çıkmazsa** `--backend fmu` kullan (daha yavaş ama orijinal yol).

`so` arka ucu senin aynı C++ binary'ni çağırır, sadece UniFMU/zmq sarmalayıcısını
atlar — fizik birebir aynı, ~1000× hızlı. Eşdeğerlik yukarıdaki testle kanıtlanıyor.

**Beklenen:** ~5–10 dakika. Script τ_model sıfır çıkarsa **durur** — eski hattaki
sessiz başarısızlık artık mümkün değil.

---

## Adım 6 — Doğrula

```bash
python3 verify_fmu_residual.py --residual ur10e_hybrid_residual.parquet --raw ur10e_raw_features.parquet
deactivate                            # venv310'dan çık
```

Beş maddenin de ✅ olması gerekiyor. Özellikle:

* `[3] τ_model` — "tamamen sıfır satır oranı = %0,00"
* `[5] fizik testi` — Eklem1/Eklem2 RMS oranı < 0,25

---

## Adım 7 — İki modeli eğit

```bash
python3 models.py        # parametre sayıları: 478.892 ve 1.907.032 → ikisi de ✅

python3 train_ae.py --mode residual --parquet ur10e_hybrid_residual.parquet \
        --model-dir residual_ae_model --amp --num-workers 0

python3 train_ae.py --mode raw --parquet ur10e_raw_features.parquet \
        --model-dir raw_ae_model --amp --num-workers 0
```

İlk ekranda kontrol et:

```
parametre : 478.892   (bildiri 478.892) ✅
eğitim 35.978 pencere | doğrulama 8.992 pencere
(bildiri: 35.978 / 8.992)
```

Süre: kalıntı ~3–5 dk, ham ~30–45 dk (RTX 4060).

> **Beklenti ayarı:** kalıntı modelinin sonuçları bildirininkilerden farklı çıkacak.
> Bildirinin sayıları da bozuk parquet'ten (τ_model = 0) geldiği için, artık hedef
> "bildiriyle aynı sayı" değil **doğru sonuç**. Ham model bundan etkilenmiyor,
> onun bildiriye yakın çıkması beklenir (AUC ~0,952).

---

## Adım 8 — Birleşim değerlendirmesi

```bash
python3 evaluate_fusion.py --residual-dir residual_ae_model --raw-dir raw_ae_model --out fusion_results
```

Bu tek komut bildirinin tüm sonuç bölümünü üretir:

* **Tablo II** — genel performans (AUC / PR-AUC / BestF1), bildirinin değerleri yanında
* **Tablo III** — arıza tipine göre BestF1 (kalıntı / ham / birleşim)
* **Ağırlık taraması** — w_kal ∈ [0,1] adım 0,05
* **MAX** ve **OR** birleşim stratejileri
* **Referans yöntemler** — Kalıntı Norm Eşiği, Isolation Forest, One-Class SVM
* **Tamamlayıcılık analizi** — arıza pencerelerinin yüzde kaçı yalnızca bir model
  tarafından yakalanıyor
* Şekil 2 (ROC), Şekil 3 (tamamlayıcılık + zamansal), Şekil 4 (ağırlık duyarlılığı)

Ek teşhis (bildiride yok, ROS 2 için gerekli — çıkarım gecikmesi, CUSUM, sınıflandırma):

```bash
python3 evaluate_ae.py --mode residual --model-dir residual_ae_model
python3 evaluate_ae.py --mode raw      --model-dir raw_ae_model
python3 compare_models.py --raw raw_ae_model --hybrid residual_ae_model
```

---

## Komut özeti

```bash
cd ~/siu2026
bash 00_ortam_kontrol.sh                                    # 0

python3 -m pip install -r requirements.txt                  # 2
python3 -m pip install torch --index-url https://download.pytorch.org/whl/cu124

python3 extract_raw_features.py --out ur10e_raw_features.parquet   # 3

source ~/venv310/bin/activate                               # 4-6 (3.10 ortamı)
python3 check_fmu.py --resources resources --fmu UR10e_InverseDynamics.fmu
python3 generate_residuals.py --backend so --out ur10e_hybrid_residual.parquet
python3 verify_fmu_residual.py --residual ur10e_hybrid_residual.parquet --raw ur10e_raw_features.parquet
deactivate

python3 models.py                                           # 7
python3 train_ae.py --mode residual --parquet ur10e_hybrid_residual.parquet --model-dir residual_ae_model --amp --num-workers 0
python3 train_ae.py --mode raw --parquet ur10e_raw_features.parquet --model-dir raw_ae_model --amp --num-workers 0

python3 evaluate_fusion.py --residual-dir residual_ae_model --raw-dir raw_ae_model --out fusion_results   # 8
```

---

## Dosyalar ne işe yarıyor

| dosya | ne yapar |
|---|---|
| `00_ortam_kontrol.sh` | ortam teşhisi; hiçbir şey değiştirmez |
| `fmu_backend.py` | ters dinamiğe iki erişim yolu (`so` / `fmu`); **solver yüklenemezse sessizce sıfır döndürmez, hata fırlatır** |
| `check_fmu.py` | ortam + import + sıfır + fizik + eşdeğerlik kontrolü |
| `extract_raw_features.py` | CSV → 27 kanallı ham parquet (modeller 24'ünü kullanır) |
| `generate_residuals.py` | hibrit artık üretimi; q̈ tek seferde, Jacobian vektörleştirilmiş, τ_model sıfırsa durur |
| `verify_fmu_residual.py` | üretilen parquet'in gerçekten FMU çıktısı içerdiğini kanıtlar |
| `models.py` | iki modelin tanımı; tek başına çalıştırılınca parametre sayılarını bildiriyle karşılaştırır |
| `train_ae.py` | eğitim (dropout %15, gradyan kırpma 1,0, ReduceLROnPlateau, patience 25, eşik P97) |
| `inject_faults.py` | dört sentetik arıza senaryosu; tek başına çalıştırılınca 14.402 pencere doğrulaması yapar |
| `evaluate_fusion.py` | bildirinin tam değerlendirme protokolü |
| `evaluate_ae.py` | ek teşhis: çıkarım gecikmesi, CUSUM, anomali sınıflandırma |
| `compare_models.py` | iki modeli tek tabloda karşılaştırır |
| `belgeler/UYUMLULUK_DENETIMI.md` | bildiriyle neyin kanıtlandığı / neyin çıkarım olduğu |

---

## Sorun çıkarsa

| belirti | bak |
|---|---|
| `ModuleNotFoundError: ur10_solver_py` | Python 3.10 kullanıyor musun? `resources/` kopyalandı mı? `chmod +x` yapıldı mı? |
| `check_fmu` [5] fizik testi ⚠ | çıktıyı bana yolla — taban yönelimi/konvansiyon meselesi olabilir |
| `so` ile `fmu` farkı büyük | `--backend fmu` ile devam et, farkı bana bildir |
| eğitimde pencere sayısı tutmuyor | parquet satır sayısı 1.124.432 değil → Adım 3'ü tekrarla |
| `torch.cuda.is_available()` False | NVIDIA sürücüsü: `sudo ubuntu-drivers autoinstall && sudo reboot` |
| CUDA out of memory | `--batch-size 128` ekle |

Takıldığın yerde çıktıyı olduğu gibi yapıştır, bakarım.
