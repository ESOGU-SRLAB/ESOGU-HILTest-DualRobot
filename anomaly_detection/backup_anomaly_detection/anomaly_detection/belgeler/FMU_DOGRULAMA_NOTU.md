# FMU doğrulaması neden ölçek farkını yakalayamadı

Soru: *"Biz bu FMU'yu gerçek robottan gelen effort değerlerine göre dinleyip nasıl
doğru dedik o zaman?"*

Kaynak: Kahraman ve ark., *FMU-Based Multi Criteria Trajectory Validation of
Industrial Robots*, Bölüm III-H "Sim-to-Real Torque Validation", Tablo 15.

---

## Makalenin yorumu

> *"the UR10e controller internally compensates for gravitational loads before
> publishing the `effort` field in `/joint_states`. Consequently, τ_real represents
> the post-compensation residual (friction, inertia, Coriolis/centrifugal), whereas
> τ_FMU from the RNEA solver computes the full inverse dynamics including gravity.
> This produces a systematic offset g_off = τ̄_FMU^static − τ̄_real^static."*

Yani fark **toplamsal bir sabit** kabul edilmiş: J2 için 70,47 Nm, J3 için 21,95 Nm,
tek bir nominal statik duruşta (q2 ≈ −2,34, q3 ≈ −1,36) ölçülmüş.

Doğrulama sonucu: bilek eklemlerinde NRMSE < %2,2 (τ_max'a göre), J2/J3'te %11,25 ve
%7,14, korelasyon r = 0,913 ve 0,853.

---

## Bu yorum veriyle test edilebilir — ve tutmuyor

1,1 milyon örnekte üç bağımsız test:

### Test 1 — Toplamsal ofset varyansı değiştirmez

| eklem | std(τ_real) | std(τ_FMU) | artık: **ofset** modeli | artık: **ölçek** modeli | eğim a |
|---|---:|---:|---:|---:|---:|
| J2 shoulder_lift | 3,48 | 37,45 | **34,13** | **0,99** | 0,0891 |
| J3 elbow | 1,80 | 15,67 | **13,97** | **0,57** | 0,1088 |

Sabit çıkarmak standart sapmayı değiştirmez. τ_FMU'nun genliği τ_real'ın 10 katıysa,
hiçbir sabit bu farkı kapatamaz. Ölçek modeli J2'de **34× daha küçük** artık bırakıyor.

### Test 2 — Makalenin kendi tablosu bunu zaten gösteriyor

```
Tablo 15, "Comp. RMSE" (gravity-compensated):   J2 = 37,12 Nm
Bizim ölçtüğümüz  std(τ_FMU):                   J2 = 37,45 Nm
```

Kompanzasyon sonrası kalan hata, **modelin tüm sinyal genliğine eşit**. τ_max = 330 Nm
ile normalize edilince %11,25 görünüyor; ama sinyalin kendisine göre **%91**.

### Test 3 — Ofset sonrası artık, τ_FMU ile −1,000 korelasyonlu

```
korel(artık_ofset, τ_FMU) = −1,000  (J2)   −0,999  (J3)
korel(τ_real,      τ_FMU) = +0,959  (J2)   +0,948  (J3)
```

Birincisi: çıkarma işlemi pratikte hiçbir şey kaldırmamış, geriye tam olarak −τ_FMU
kalmış. İkincisi daha kritik: **kontrolcü yerçekimini gerçekten çıkarsaydı**, τ_real
sadece sürtünme + atalet olurdu ve yerçekimi baskın bir sinyalle 0,96 korelasyon
gösteremezdi. Yüksek pozitif korelasyon, yerçekiminin effort içinde **hâlâ var
olduğunu** kanıtlıyor — sadece 1/11 ölçekte.

---

## Doğrulama ölçütü neden geçti

NRMSE, τ_max ile normalize edilmiş. Sinyalin kendisiyle karşılaştırınca:

| eklem | τ_max | NRMSE % | → hata Nm | sinyal std | **hata/sinyal** |
|---|---:|---:|---:|---:|---:|
| J1 | 330 | 0,67 | 2,21 | 3,84 | 0,58× |
| J2 | 330 | 11,25 | 37,12 | 37,45 | **0,99×** |
| J3 | 150 | 7,14 | 10,71 | 15,67 | 0,68× |
| J4 | 56 | 2,16 | 1,21 | 0,94 | **1,29×** |
| J5 | 56 | 0,87 | 0,49 | 0,18 | **2,72×** |
| J6 | 56 | 1,10 | 0,62 | 0,068 | **9,01×** |

"hata/sinyal > 1" olan satırlarda **model sıfır tahmin etseydi de testi geçerdi.**
Makalenin "wrist-joint prediction error remains below 2.2% of maximum torque capacity"
diye öne çıkardığı eklemler tam olarak bunlar: J6'da izin verilen hata, sinyalin
9 katı.

Özetle doğrulama, ölçek hatasının **en görünmez olduğu yerde** yapılmış: bilekler
yerçekimi yükü taşımadığı için sinyalleri zaten çok küçük, ve τ_max ile normalize
edilen bir ölçüt orada her şeyi geçirir.

---

## Gerçek ilişki

`effort` alanı Nm değil, **motor akımı**. UR'ın kendi ROS 2 sürücü belgesi:

> *"The effort field contains the currents reported by the joints and not the actual
> efforts in a physical sense."*

Dolayısıyla doğru model çarpımsal:

```
i_ölç = a·τ_FMU + b          a = 1/(K_t · redüktör)
```

Ölçülen: a = 0,0891 (J2), 0,1088 (J3) → K_t·redüktör ≈ 11,2 ve 9,2 Nm/A.

Sürtünme terimi de eklenirse model neredeyse tamamlanıyor:

| eklem | R² (yalnız ölçek) | R² (ölçek + sürtünme) | a değişimi |
|---|---:|---:|---|
| J2 shoulder_lift | 0,919 | **0,950** | 0,0923 → 0,0914 |
| J3 elbow | 0,899 | **0,963** | 0,1099 → 0,1084 |
| J6 wrist_3 | 0,000 | **0,908** | — |

`a` katsayısının sürtünme eklenince neredeyse hiç değişmemesi, onun uydurma bir
düzeltme değil **fiziksel bir sabit** olduğunun göstergesi. Bileklerde yerçekimi yok,
sinyal tamamen sürtünme — bu yüzden ölçek tek başına hiçbir şey açıklamıyor ama
sürtünme eklenince R² 0,91'e çıkıyor.

---

## Bu neyi geçersiz kılmıyor

* **FMU modelinin fiziği doğru.** Her iki analiz de aynı şeyi söylüyor: RNEA gerçek
  torkun şeklini çok iyi yakalıyor (r ≈ 0,95). Ayrıca resmi `ur_description`
  kütleleriyle bağımsız hesapladığım yerçekimi torku (kol yatayken 121 Nm), FMU'nun
  ürettiği ~112 Nm tepe değeriyle örtüşüyor.
* **Makalenin çerçeve sonuçları büyük ihtimalle etkilenmiyor.** Makale bunu kendisi
  söylüyor: *"The torque ratio metric ρ_max = τ/τ_max used in the Dynamics FMU is
  formulated as a normalized margin rather than an absolute comparison."* Normalize
  edilmiş bir marj metriği kullanıldığı için, sabit bir ölçek faktörü PASS/FAIL
  kararlarını sistematik olarak kaydırır ama sıralamayı bozmaz.

Etkilenen tek şey, τ_FMU ile effort'un **mutlak** karşılaştırıldığı yer — ki bizim
artık üretimimiz tam olarak orası. `r_top = τ_ölç − τ_model` çıkarması ancak ikisi
aynı birimdeyse anlamlı.

---

## Bizim için sonuç

`generate_residuals.py`'deki eklem başına afin kalibrasyon (`τ_ölç ≈ a·τ_model + b`)
bu sorunu çözüyor ve artığı ters dinamiğin çıkarması gereken kadar küçültüyor:

```
r_int std (J2)  :  kalibresiz 34,61  →  kalibre 4,28
r_int std (J3)  :  kalibresiz 14,51  →  kalibre 2,66
motor kayması   :  1,7σ  →  9,4σ
```

Katsayılar `residual_calibration.json`'a yazılıyor; ROS 2 node'u da
`dynamic_joint_states`'ten okuduğu effort'a aynı dönüşümü uygulayacak.

### Öneri

Bu bulguyu Kahraman ve ark. makalesine iletmeye değer. İki küçük değişiklik
doğrulamayı çok daha güçlü yapar:

1. NRMSE'yi τ_max yerine **sinyalin standart sapmasına** göre normalize etmek
   (bir modelin sıfır tahmin ederek testi geçmesini engeller).
2. Sim-to-real karşılaştırmasında eğimi serbest bırakmak — regresyon zaten
   ~1/10 çıkacak ve akım/tork dönüşümünü doğrudan ölçmüş olacaklar.
