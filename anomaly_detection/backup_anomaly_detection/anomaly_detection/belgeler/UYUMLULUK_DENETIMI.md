# 245.pdf uyumluluk denetimi

Soru: "her şeyin bildiriyle uyumlu olduğundan emin misin?"
Dürüst cevap üç kademede.

---

## ✅ Kademe 1 — Sayısal olarak KANITLANMIŞ (birebir eşleşme)

Bunlar tahmin değil; hesaplayıp bildirinin yazdığı sayıyla karşılaştırdım.

| doğrulanan | hesabım | bildiri | |
|---|---|---|---|
| Kalıntı model parametresi (D=12, H=128, Z=32) | 478.892 | 478.892 | ✅ |
| Ham model parametresi (D=24, H=256, Z=64) | 1.907.032 | 1.907.032 | ✅ |
| Pencere sayısı (N=1.124.432, W=100, S=25) | 44.974 | — | |
| Test penceresi (4 senaryo × 44.974) | 179.896 | 179.896 | ✅ |
| Eğitim penceresi (%80 örnek bölmesi) | 35.978 | 35.978 | ✅ |
| Doğrulama penceresi | 8.992 | 8.992 | ✅ |
| Arıza penceresi (4 senaryo, ≥%10 örtüşme) | 14.402 | 14.402 | ✅ |

Parametre sayısının **birebir** tutması mimarinin (katman sayısı, gizli/saklı boyut,
LSTM bias düzeni, doğrusal katmanlar) doğru olduğunu kanıtlar — tek bir katman farkı
olsa sayı tutmazdı.

14.402'nin tutması ise arıza aralıklarını (%30–38, %61–69, %92–100, %15–23) ve
"≥%10 örtüşme" etiketleme kuralını doğru çözdüğümü kanıtlar.

---

## 📄 Kademe 2 — Bildiride AÇIKÇA yazan, harfiyen uyguladıklarım

| | bildiri | kodda |
|---|---|---|
| Kalıntı girdi | 12 kanal (r_ic,1–6 + r_dis,1–6) | ✔ |
| Ham girdi | 24 kanal (q, q̇, τ, KTS × 6) | ✔ |
| Pencere / adım | 100 / 25 (%75 örtüşme) | ✔ |
| Mimari | 2 katmanlı LSTM gizyazar → saklı → RepeatVector → simetrik gizçözer | ✔ |
| Sönümleme | %15, katmanlar arasında | ✔ |
| İyileştirici | Adam, lr=1e-3, β₁=0,9, β₂=0,999 | ✔ |
| Yığın boyutu | 256 | ✔ |
| Gradyan kırpma | max_norm = 1,0 | ✔ |
| Çizelgeleyici | ReduceLROnPlateau (faktör 0,5, sabır 8) | ✔ |
| Erken durdurma | sabır 25, maksimum 300 dönem | ✔ |
| Kayıp | MSE, yalnız normal veri | ✔ |
| Eşik | doğrulama hatalarının 97. persentili | ✔ |
| Birleşim | min–max sonrası ağırlıklı ortalama | ✔ |
| Ağırlık taraması | w_kal ∈ [0,1], adım 0,05, BestF1 üzerinden | ✔ |
| Ek stratejiler | MAX, OR | ✔ |
| Referans yöntemler | Kalıntı Norm Eşiği · Isolation Forest (kont. 0,08) · One-Class SVM (RBF, ν=0,05) | ✔ |
| Arıza genlikleri | 15/25 Nm · 30/40 N · 1,5 rad / 8 Nm · 3,5 N / 0,08 Nm | ✔ |
| Arıza aralıkları | %30–38 · merkez %65 gen. %4 · %92 · %15–23 | ✔ |
| Pencere etiketi | ≥ %10 örtüşme | ✔ |

---

## ⚠️ Kademe 3 — Bildiride YAZMAYAN, benim karar verdiğim yerler

Bunlar sonucu etkileyebilir. Şeffaf olmak istiyorum.

### 3.1 Kalıntı uzayında hangi kanala enjekte edildiği — **en kritik belirsizlik**

Bildiri hedefi yalnız ham uzay için söylüyor ("Eklem 3 torku", "Eklem 5 pozisyonu"),
kalıntı uzayı için sadece genliği veriyor (25 Nm, 8 Nm).

**Seçimim:** motor kayması → `r_int_3`, gizyazar hatası → `r_int_5`
(ikisi de içsel arıza → içsel kalıntı). Çarpışma ve sensör gürültüsü → `r_ext_1..6`
(KTS kaynaklı → dışsal kalıntı).

**Destekleyen kanıt:** bildirinin kendi gelecek çalışma maddesi
*"(i) Jacobian aracılığıyla yayılan darbe modelinin uygulanması"* — yani darbeyi
Jacobian üzerinden yaymayı **yapmamışlar**, doğrudan kalıntı kanallarına enjekte
etmişler. Benim yaklaşımım bununla uyumlu. Ama tam kanal numarası çıkarım.

*Etkisi:* Tablo III'teki kalıntı sütununu değiştirebilir.

### 3.2 Referans yöntemlerin öznitelik temsili — **ikinci kritik belirsizlik**

Bildiri sadece *"Bu yöntemler kalıntı öznitelikleri üzerinde eğitilmiştir"* diyor.

**Seçimim:** pencere başına 12 kanalın ortalaması + standart sapması = 24 boyut.
Alternatifler: düzleştirilmiş pencere (12×100 = 1200 boyut) ya da başka bir özet.

*Etkisi:* Isolation Forest ve One-Class SVM satırları. LSTM ve birleşim satırlarını
etkilemez.

### 3.3 Diğer karar noktaları

| konu | bildiri | seçimim | etkisi |
|---|---|---|---|
| Gauss darbe σ | "genişlik %4" | maske = merkez ± %4 (14.402 ile doğrulandı), σ = genişlik·N/2 → maske ±2σ | darbe şekli, maske değil |
| Gauss gürültü genliği | "3,5 N" | standart sapma olarak | küçük |
| Normalizasyon kaynağı | belirtilmemiş | yalnız eğitim böleni (`--norm all` ile değişir) | küçük |
| Kalıntı Norm Eşiği | "toplam kalıntı vektörünün L2 normu" | pencere içi ortalama ‖r_top‖ (maks. da olabilirdi) | o satır |
| One-Class SVM eğitim boyutu | belirtilmemiş | 8.000 pencere alt örneklem (O(n²) olduğu için) | o satır |
| OR satırının AUC'si | OR ikili bir karar, ROC eğrisi yok | MAX skor eğrisi kullanıldı — bildiride OR ve MAX satırları **birebir aynı** (0,976/0,889/0,824), demek ki onlar da öyle yapmış | yok |
| Sönümleme yerleşimi | "katmanlar arasında" | `nn.LSTM(dropout=0.15)` | parametre sayısı tuttuğu için doğrulanmış sayılır |
| q̈ tahmincisi | belirtilmemiş | mevcut parquet neyle üretildiyse o (savgol 51,3) | yok — aynı dosyayı kullanıyoruz |

### 3.4 Metodolojik bir gözlem

179.896 = 4 × **tüm** 44.974 pencere. Yani test kümesi eğitim pencerelerini de
kapsıyor. Aritmetik tartışmasız (başka türlü 179.896 çıkmıyor), ben de öyle uyguladım —
ama not düşüyorum.

---

## 🔧 Bu turda düzelttiğim gerçek hata

Tablo III'te birleşim sütununu **senaryo bazında** yeniden min–max normalleştiriyordum.
Bu yanlış: ağırlıklar global ölçekte optimize ediliyor, senaryo bazında yeniden
normalleştirmek w_kal'ın anlamını bozuyor. Artık global normalize edilmiş skorlar
senaryo bazında dilimleniyor.

(Tekil model sütunları monoton dönüşüme duyarsız olduğu için onlar etkilenmiyordu.)

Ayrıca Şekil 2'ye referans yöntemlerin ROC eğrileri de eklendi — bildiri
*"Tüm modellerin ROC eğrileri"* diyor.

---

## Henüz doğrulanmamış tek şey

`ur10e_hybrid_residual.parquet`'in **satır sayısı 1.124.432 mi?** Cihazdaki Linux VM'de
pyarrow olmadığı için okuyamadım. Ama buna gerek yok: `train_ae.py` çalışırken

```
eğitim 35.978 pencere | doğrulama 8.992 pencere
(bildiri: 35.978 / 8.992)
```

satırını yazdırıyor. **İlk eğitimin ilk saniyelerinde bu iki sayı tutuyorsa
veri tabanı da bildiriyle aynı demektir.** Tutmuyorsa haber ver, parquet'i
yeniden üretmemiz gerekir.

---

## Sonuçlar tutmazsa nereye bakılır (sırayla)

1. **Parametre sayısı** — `python models.py` ✅ vermiyorsa mimari sorunu.
2. **Pencere sayıları** — eğitim çıktısındaki 35.978 / 8.992 satırı.
3. **Yakınsama dönemi** — bildiri 129 (kalıntı) ve 87 (ham). Çok sapıyorsa
   normalizasyon (`--norm all` dene) veya tohum farkı.
4. **val_loss** — bildiri 0,181 ve 0,320. Bunlar tutuyorsa eğitim doğru.
5. **Eşik** — bildiri θ=0,420 ve θ=0,854. val_loss tutup eşik tutmuyorsa
   persentil hesabında sorun var.
6. **Tekil AUC** — 0,908 ve 0,952. Buraya kadar tutup AUC tutmuyorsa sorun
   arıza enjeksiyonunda (§3.1).
7. **Referans yöntemler** — sadece IF/OCSVM sapıyorsa §3.2.

`train_ae.py` ve `evaluate_fusion.py` bildirinin değerlerini **kendi çıktılarının
yanında** yazdırıyor, dolayısıyla bu kontrolü ayrıca yapmana gerek yok.

---

## Özet

Mimari, veri bölme, pencereleme, eğitim yapılandırması ve arıza maskeleri konusunda
**eminim** — dördü sayısal olarak birebir doğrulandı, gerisi bildiride açıkça yazıyor.

Emin olmadığım iki nokta var: **kalıntı uzayındaki enjeksiyon kanalı** (§3.1) ve
**referans yöntemlerin öznitelik temsili** (§3.2). İkisi de bildiride yazmıyor.
İlk koşudan sonra sayılar sapıyorsa ilk bakacağımız yerler bunlar; ikisi de tek
satırlık değişikliklerle denenebilir.
