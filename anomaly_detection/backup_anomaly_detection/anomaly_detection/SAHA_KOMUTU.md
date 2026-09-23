# Gerçek hücre oturumu — dağıtım yapılandırması

Seçim doğrulama kümesinden yapıldı (test kümesi görülmedi): **tohum 1**, doğrulama F1 0,788.

## Düğümü başlatma

v3 modelleri artık **pakette**. Elle yol vermek gerekmiyor:

```bash
cd ~/colcon_ws
git pull
colcon build --packages-select anomaly_detection
source install/setup.bash
ros2 launch anomaly_detection detector.launch.py
```

Bildirinin erratum yeniden üretimi için: `model_gen:=v2`.

### Başlarken tek kontrol

Düğüm şu üç satırı basmalı:

```
birleşim: w_kal=0.95 w_ham=0.05 θ_duran=0.10456 θ_hareketli=0.85526
          (|q̇|>0.02 rad/s ile seçilir; global θ=0.67514 kullanılmıyor)
Köken: .../kosu_<zaman>.json  (w=0.95, sürtünme=var, eşik=rejim)
```

`sürtünme=yok` veya `eşik=global` görürseniz **oturumu durdurun** — kurulum
bayat demektir. Bayat kurulumda düğüm ayrıca stderr'e gürültülü bir uyarı basar.

26.08.2026 oturumu tam olarak böyle kaybedildi: kod güncelken paket yalnız v2
artefaktlarını taşıyordu, launch dosyası da model yollarını v2'de sabitliyordu.
Ölçüm eski modellerle, sürtünmesiz ve global eşikle alındı; bu ancak köken
dosyasından anlaşıldı. Artık sürtünme, kalibrasyon ve model yolları tek
kaynaktan (`model_gen`) türüyor, karışması yapısal olarak mümkün değil.

## Dondurulmuş yapılandırma

| alan | değer | nereden |
|---|---|---|
| w_kal / w_ham | 0,95 / 0,05 | **önsel sabit** (bildiri); hiçbir değerlendirme kümesinden seçilmedi |
| eşik kuralı | regime_p97 | doğrulama |
| θ duran | 0,1046 | doğrulamanın duran temiz pencereleri, P97 |
| θ hareketli | 0,8553 | doğrulamanın hareketli temiz pencereleri, P97 |
| hareket eşiği | \|q̇\| > 0,02 rad/s | çevrimdışı ve çevrimiçi aynı büyüklük |
| sürtünme | açık | Fc/Fv yalnız 489 eğitim koşusundan |

Her oturum artık yanına `kosu_<zaman>.json` yazar: hangi modeller (SHA-256),
hangi eşikler, sürtünme var mı, git commit'i. Bu dosya olmadan kayıt altı ay
sonra çözümlenemez — makalenin gerçek hücre bölümündeki model uyuşmazlığı tam
olarak bu boşluktan doğmuştu.

Uyarlanabilir kural launch dosyasında zaten **kapalı** (`adaptive:=false`
varsayılan): kural çevrimdışı yardımcı oluyor ama 2026-08-21
oturumunda doğrulanmış hiçbir olayı yakalamadı ve taban çizgisi dondurması
564 saniyelik tek bir bloğa yol açtı. Kapalıyken operatörün etiketleri doğrudan
rejim eşiğinin alarmlarıyla eşleşir. `hit_uyarlanabilir` yine de kaydedildiği
için kural sonradan çevrimdışı değerlendirilebilir.

## Oturumda dikkat

1. **Düğümü robot HAREKET EDERKEN başlat.** 2026-08-21'de dururken başlatıldı,
   uyarlanabilir taban çizgisi hareketsiz gürültüden öğrenildi ve 564 saniyelik
   tek alarm bloğu gerçek bir sıkışmayı yuttu.
2. **En az beş tam muayene çevrimi** koştur. Yanlış alarmlar çevrime kilitli
   olduğu için gözlem birimi geçen süre değil çevrim sayısı.
3. Her provoke edilen anomaliyi arayüzden **etiketle** — `analyze_session.py`
   gerçek/yanlış ayrımını o dosyadan yapıyor.
4. Pick-and-place senaryosunu da koştur: 2026-08-21'de 12 yanlış alarmın 10'u
   oradan geldi, karşılaştırma noktası olarak lazım.

## Oturum sonrası

```bash
python3 ~/Desktop/backup_anomaly_detection/anomaly_detection/analyze_session.py \
    --dir ~/anomali_kayit --labels ~/anomali_kayit/etiketler.json
```
