# v3 modellerinin kökeni

Bu dizindeki artefaktlar `run_pipeline_v3.sh` ile üretildi. Kısaca:

- **Bölme**: koşu-ayrık (`make_splits.py`), 489/16/95 koşu. Eski hatta değerlendirme
  pencerelerinin %79,6'sı eğitim penceresiydi; şimdi bölmeler arası paylaşılan
  örnek sayısı sıfır.
- **Kalıntı**: sürtünme terimi çözücünün DIŞINDA, `r_top = τ_ölç − τ̂_model − b − τ̂_f(q̇)`.
  Katsayılar yalnız 489 eğitim koşusundan (`friction_model.json`).
  **FMU'ya dokunulmadı.**
- **Ağırlık**: w_kal = 0,95 **önsel sabit** (bildiriden). Hiçbir değerlendirme
  kümesinden seçilmedi.
- **Eşik**: rejim-koşullu (duran / hareketli), doğrulama koşularının temiz
  pencerelerinden P97.
- **Tohum**: 1 / 5. Seçim yalnız DOĞRULAMA F1'ine göre yapıldı (0,788); test
  kümesi görülmedi.

Ayrıntılı sayılar ve tohumlar arası dağılım için hat dizinindeki
`fusion_v3_summary.json` ve `results.json` dosyalarına bakın.

v2 artefaktları bildirinin erratum yeniden üretimi için pakette KALIYOR;
dağıtım varsayılanı v3'tür.
