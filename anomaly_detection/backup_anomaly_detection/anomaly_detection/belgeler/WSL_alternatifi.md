# Sıfırdan kurulum — adım adım

## Neden sıfırdan

`ur10e_hybrid_residual.parquet` geçersiz: τ_model satırların **%100'ünde tam sıfır**.
Sebep, dinamik modelin kendisi değil — **onu çalıştıramamak**:

```
resources/ur10_solver_py.cpython-310-x86_64-linux-gnu.so
  → ELF 64-bit, x86-64, Linux, CPython 3.10

Bu makine: Windows + Python 3.12.   Windows uzantısı (.pyd) yok.
```

`resources/model.py` içindeki `import ur10_solver_py` Windows'ta yükleneme­z, hata
`except ImportError` ile yutulur, `self.solver = None` kalır ve `_update_outputs`
daha ilk satırda geri döner. Sonuç: τ = 0, `r_top = τ_ölç`, ayrıştırma hiç yapılmamış.
Beklediğin FMU log'unun gelmemesi de aynı sebepten.

> **Dinamik modele dokunulmuyor.** Fizik doğrulanmış; tek yaptığımız onu kendi
> ortamında (Linux + Python 3.10) çalıştırmak. Bonus: ROS 2 Humble = Ubuntu 22.04 =
> Python 3.10, yani burada doğruladığımız import yolu robota birebir taşınacak.

---

## Genel plan

| adım | nerede | süre |
|---|---|---|
| 0. WSL2 + Ubuntu 22.04 kur | Windows | ~10 dk (tek seferlik) |
| 1. Bağımlılıklar | WSL | ~3 dk |
| 2. Dosyaları WSL'e kopyala | WSL | ~1 dk |
| 3. **Sağlık kontrolü** — FMU gerçekten çalışıyor mu | WSL | ~1 dk |
| 4. Artıkları üret | WSL | ~5–10 dk |
| 5. Doğrula | WSL | ~1 dk |
| 6. Parquet'i Windows'a geri kopyala | WSL | ~1 dk |
| 7. İki modeli eğit | Windows (RTX 4060) | ~1 sa |
| 8. Birleşim değerlendirmesi | Windows | ~10 dk |

Eğitim Windows'ta kalıyor çünkü torch + CUDA ortamın zaten hazır ve modellerin
FMU'ya ihtiyacı yok. WSL yalnızca artık üretimi için gerekli.

---

## Adım 0 — WSL2 + Ubuntu 22.04

**Yönetici olarak** PowerShell aç:

```powershell
wsl --install -d Ubuntu-22.04
```

Bilgisayarı yeniden başlat, Ubuntu açılınca kullanıcı adı/parola belirle.
Zaten WSL'in varsa sürümü kontrol et — **22.04 olmalı** (Python 3.10 onunla gelir):

```powershell
wsl -l -v
```

Farklı bir dağıtım varsa yanına 22.04 kurabilirsin:

```powershell
wsl --install -d Ubuntu-22.04
```

---

## Adım 1 — Bağımlılıklar (WSL içinde)

```bash
python3 --version          # 3.10.x görmelisin
sudo apt update
sudo apt install -y python3-pip
pip3 install numpy pandas pyarrow scipy fmpy pyzmq
```

---

## Adım 2 — Dosyaları WSL'e kopyala

WSL'den Windows diskine doğrudan erişmek büyük dosyalarda çok yavaş; çalışma
dosyalarını WSL'in kendi diskine alıyoruz.

```bash
mkdir -p ~/siu2026 && cd ~/siu2026
WIN="/mnt/c/Users/DELL/Desktop/İş/Yayın/SIU2026"

cp -r "$WIN/AnomalyDetection_v1/resources" .
cp -r "$WIN/AnomalyDetection_v1/binaries" .
cp "$WIN/AnomalyDetection_v1/UR10e_InverseDynamics.fmu" .
cp "$WIN/AnomalyDetection_v1/modelDescription.xml" .
cp "$WIN/AnomalyDetection_v1/ur10e_jacobian.py" .
cp "$WIN/AnomalyDetection_v1/ur10e_raw_features.parquet" .

cp "$WIN/AnomalyDetection_Live/fmu_backend.py" .
cp "$WIN/AnomalyDetection_Live/check_fmu.py" .
cp "$WIN/AnomalyDetection_Live/generate_residuals.py" .
cp "$WIN/AnomalyDetection_Live/verify_fmu_residual.py" .

chmod +x resources/ur10_solver_py*.so
ls -la
```

`İş`/`Yayın` klasör adlarında Türkçe karakter var; kopyalama hata verirse yolu
tırnak içinde bıraktığından emin ol.

---

## Adım 3 — Sağlık kontrolü ⚠️ EN ÖNEMLİ ADIM

```bash
cd ~/siu2026
python3 check_fmu.py --resources resources --fmu UR10e_InverseDynamics.fmu
```

Altı şeyi kontrol eder:

1. **Ortam** — Linux x86-64 / CPython 3.10 uyumu
2. **`so` arka ucu** — `ur10_solver_py` import ediliyor mu, sınıf örnekleniyor mu
3. **`fmu` arka ucu** — fmpy ile FMU örnekleniyor mu
4. **Sıfır testi** — bilinen bir duruşta τ sıfırdan farklı mı
5. **Fizik testi** — kol yatay uzatıldığında yerçekimi: düşey eksenli **Eklem 1'de
   tork ≈ 0**, shoulder_lift'te büyük tork. Bu, "sıfır değil ama uydurma" bir çıktıyı
   da yakalar.
6. **Eşdeğerlik** — `so` ve `fmu` yolları 200 rastgele durumda aynı sayıyı veriyor mu

Bu adım geçmeden **kesinlikle** devam etme. Çıktıyı bana yolla.

---

## Adım 4 — Artıkları üret

Önce küçük bir doğrulama koşusu, iki arka uçla:

```bash
python3 generate_residuals.py --backend so  --limit-rows 20000 --out check_so.parquet
python3 generate_residuals.py --backend fmu --limit-rows 20000 --out check_fmu.parquet
python3 -c "
import pandas as pd, numpy as np
a=pd.read_parquet('check_so.parquet'); b=pd.read_parquet('check_fmu.parquet')
c=[f'tau_model_{j}' for j in range(1,7)]
print('so vs fmu maks fark:', np.abs(a[c].to_numpy()-b[c].to_numpy()).max())"
```

Fark ~0 çıkmalı. Çıkarsa tam üretim (`so` arka ucu 25× hızlı, aynı C++ binary):

```bash
python3 generate_residuals.py --backend so --out ur10e_hybrid_residual.parquet
```

Fark ~0 çıkmazsa `--backend fmu` kullan (daha yavaş ama orijinal yol).

Script τ_model sıfır çıkarsa **durur** — eski hattaki sessiz başarısızlık artık mümkün değil.

---

## Adım 5 — Doğrula

```bash
python3 verify_fmu_residual.py --residual ur10e_hybrid_residual.parquet --raw ur10e_raw_features.parquet
```

Bu sefer beş maddenin de ✅ olması gerekiyor. Özellikle:

* `[3] τ_model` — sıfır satır oranı %0
* `[5] fizik testi` — Eklem1/Eklem2 RMS oranı < 0,25

---

## Adım 6 — Parquet'i Windows'a geri al

```bash
cp ur10e_hybrid_residual.parquet "/mnt/c/Users/DELL/Desktop/İş/Yayın/SIU2026/AnomalyDetection_v1/"
```

---

## Adım 7 — İki modeli eğit (Windows, PowerShell)

```powershell
$py = "C:\Users\DELL\AppData\Local\Programs\Python\Python312\python.exe"
$live = "C:\Users\DELL\Desktop\İş\Yayın\SIU2026\AnomalyDetection_Live"
cd "C:\Users\DELL\Desktop\İş\Yayın\SIU2026\AnomalyDetection_v1"

& $py "$live\train_ae.py" --mode residual --parquet ur10e_hybrid_residual.parquet --model-dir residual_ae_model --amp --num-workers 0
& $py "$live\train_ae.py" --mode raw --parquet ur10e_raw_features.parquet --model-dir raw_ae_model --amp --num-workers 0
```

Kalıntı modelinin sonuçları bu sefer **öncekinden farklı çıkacak** — çünkü ilk
koşuda girdi `τ_ölç − J^T F` idi, şimdi gerçek `τ_ölç − τ_model − J^T F` olacak.
Bildirinin kendi sayıları da bozuk parquet'ten geldiği için birebir eşleşme
beklemiyorum; artık hedef doğru sonuç.

---

## Adım 8 — Birleşim değerlendirmesi

```powershell
& $py "$live\evaluate_fusion.py" --residual-dir residual_ae_model --raw-dir raw_ae_model --out fusion_results
```

---

## Özet — çalıştıracağın komutlar

```
WSL:      check_fmu.py            →  ✅ almadan devam etme
WSL:      generate_residuals.py   →  (önce --limit-rows 20000 ile iki arka uç)
WSL:      verify_fmu_residual.py  →  beş madde de ✅
Windows:  train_ae.py ×2
Windows:  evaluate_fusion.py
```

---

## Yeni/değişen dosyalar

| dosya | ne yapar |
|---|---|
| `fmu_backend.py` | `so` ve `fmu` arka uçları; **solver yüklenemezse sessizce sıfır döndürmez, hata fırlatır** |
| `check_fmu.py` | ortam + import + sıfır + fizik + eşdeğerlik kontrolü |
| `generate_residuals.py` | artık üretimi; CSV yerine raw parquet okur, q̈'yı tek seferde alır, Jacobian vektörleştirilmiş, τ_model sıfırsa durur |
| `verify_fmu_residual.py` | 5. testteki 0/0 açığı kapatıldı (her şey sıfırken yanlışlıkla ✅ veriyordu) |

Jacobian vektörleştirmesini senin `ur10e_jacobian.py` referansına karşı 300 rastgele
duruşta doğruladım: **maks fark 0.000e+00**, hızlanma **25×** (1,1 milyon örnek için
209 s → 8 s). Üretim sırasında bu kontrol her koşuda otomatik tekrarlanıyor.
