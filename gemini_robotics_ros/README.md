# gemini_robotics_ros

Gemini Robotics **ER 2** (embodied reasoning) ile UR10e + VGC10 vakum kavrayıcı
arasında köprü. Doğal dil komutu → 3D hedef + yüzey normali → MoveIt hareketi.

Donanım hedefi: **UR10e + OnRobot VGC10 vakum** ve **SICK Visionary-T Mini
V3S145-1A** ToF kamera. Sim tarafı `hil_test_whole_unified.launch.py
use_vacuum_gripper:=true`.

## Önemli: ER 2 ne yapar, ne yapmaz

ER 2 **robot aksiyonu üretmez**. Girdi olarak görüntü + metin alır, çıktı olarak
metin/koordinat verir. Aksiyon üreten model (Gemini Robotics 1.5 VLA) yalnızca
seçili partnerlere açık. Mimari bu yüzden zorunlu olarak şu:

```
derinlik (ToF) ──► render ──► ER 2 ──► [y,x] 0-1000 nokta
                                          │
                          bulut yaması + PCA düzlem oturtma
                                          │
                            konum + YÜZEY NORMALİ (optik frame)
                                          │
                              TF ──► world  (nokta VE yön ayrı ayrı)
                                          │
                       MoveIt + pymoveit2_sim ──► UR10e + VGC10
```

## Neden RGB değil, derinlik

Gerçek **SICK Visionary-T Mini'nin RGB modu yok**. Yayınladıkları:
`depth` (16UC1, mm), `intensity` (16UC1, yakın-IR genlik), `statemap`,
`points`, `camera_info`. Gazebo'nun `rgbd_camera`'sı RGB üretiyor ama gerçek
donanımda böyle bir görüntü hiç olmayacak — sim'de RGB ile çalışıp gerçeğe
geçmek, üzerinde çalışılan modaliteyi tamamen değiştirmek demek.

Bu yüzden ER 2'ye verilen görüntü, iki dünyada da var olan tek şeyden üretilir:
**derinlik**. Her iki taraf da önce metreye çevrilir, sonra **aynı sabit metrik
aralıkla** (`render_min_m`..`render_max_m`) normalize edilip aynı şekilde
render edilir. Parite buradan gelir; testlerde sim (32FC1/m) ve gerçek
(16UC1/mm) render'ları piksel başına ≤1 fark veriyor.

| `render_mode` | Ne | Nerede |
|---|---|---|
| `normals` (varsayılan) | Yüzey normali + Lambert gölgeleme. Mat gri 3B render gibi; kenar, eğrilik, düz yüzeyler doğru okunur. VLM için en doğal olanı. | sim + gerçek |
| `turbo` | Sabit aralıkta renklendirilmiş derinlik. Renk = MESAFE. | sim + gerçek |
| `gray` | Ters normalize gri derinlik. En yalın. | sim + gerçek |
| `intensity` | Kameranın kendi IR genlik görüntüsü. Gerçekte en "fotoğraf gibi" olan. | **yalnız gerçek** |

Prompt'a modaliteyi açıklayan bir bağlam bloğu ekleniyor (`IMAGE_CONTEXT`);
bu olmadan ER 2 görüntüyü olağan bir fotoğraf sanıp renk/doku üzerinden akıl
yürütmeye çalışıyor.

**Dürüst sınır:** hiçbir derinlik render'ı ER 2 için dağılım-içi girdi değil.
Nesne KATEGORİSİ tanımak RGB'ye göre zayıf olacaktır. Buna karşılık geometrik
sorgular ("düz panel", "kutunun üstü", "en yakın nesne") iyi çalışır — ve
vakumlu kavrama zaten geometrik bir problem.

## Neden yüzey normali

Emme kabı yüzeye **dik** oturmak zorunda, yoksa sızdırır ve parça hiç tutulmaz.
ER 2 yalnız 2D nokta verdiği için oryantasyon buradan gelir: gösterilen pikselin
çevresindeki bulut noktalarına PCA ile düzlem oturtulur.

- En küçük özvektör → **yüzey normali** (kameraya doğru yönlendirilir)
- En küçük özdeğerin karekökü → **düzlemsellik artığı**. `max_surface_rms`'i
  aşarsa (kenar, köşe, delik) kavrama **hiç denenmez**.
- Yaklaşma, oturtma ve kaldırma dünya Z'si boyunca değil **normal boyunca**
  yapılır; dikey bir yüzeyden parça çekmek ancak böyle doğru olur.

Yamanın derinlik bandı, ER'nin işaret ettiği pikselin kendi derinliğine
çapalanır — yamanın medyanına değil. Yama iki yüzeye yaklaşık eşit bölündüğünde
(yani bandın var olma sebebi olan kenar durumunda) medyan iki yüzeyin arasındaki
boşluğa düşüyor ve her iki yüzey birden eleniyor.

## VGC10 vakum: eklem yok

- URDF'te `vacuum_gripper` ve `suction_cup` **sabit eklemlerle** bağlı; vakum
  SRDF'inde `gripper` planlama grubu **yok**. `MoveIt2Gripper` bu donanımda
  tanımsızdır.
- Kavrama Modbus üzerinden: `vgc10_msgs/OnRobotVGOutput`
  (`rmca` 0=Release/1=Grip/2=Idle, `rvca` hedef vakum %, **80'i aşmayın**).
  Durum: `OnRobotVGInput` (`gvca`, bağıl vakumun 1/1000'i).
- Uç link **`sim_ur10e_suction_cup`**, `tool0` değil. `suction_cup_joint` ve
  `vacuum_gripper_joint` ikisi de `rpy 0 0 0` olduğu için emme ekseni
  `tool0`'ın +Z'si ile aynı hizada.

## Kurulum

```bash
pip install --upgrade google-genai

cd ~/colcon_ws
colcon build --packages-select gemini_robotics_ros
source install/setup.bash
```

## API anahtarı

Anahtarı <https://aistudio.google.com/apikey> adresinden alın (ücretsiz katman
var) ve **ev dizinindeki anahtar dosyasına** yazın:

```bash
mkdir -p ~/.config/gemini
printf '%s\n' 'ANAHTARINIZ' > ~/.config/gemini/api_key
chmod 600 ~/.config/gemini/api_key
```

Bir kez yapılır; yeniden başlatmadan sonra tekrar gerekmez.

Kod anahtarı şu sırayla arar:

1. `GEMINI_API_KEY` ortam değişkeni
2. `GOOGLE_API_KEY` ortam değişkeni
3. `~/.config/gemini/api_key`
4. `/etc/gemini/api_key`

**Neden dosya:** ortam değişkeni yalnızca onu export eden kabuktan başlatılan
süreçlere geçer. Düğüm masaüstü kısayolundan, bir IDE terminalinden, systemd
biriminden ya da `.bashrc` düzenlenmeden önce açılmış bir terminalden
başlatılırsa anahtar yoktur — ve belirti "her açılışta anahtarı yeniden girmek
gerekiyor" gibi görünür. Dosya bu bağlamların hepsinde okunur.

Ortam değişkenini de isterseniz, anahtarı `.bashrc`'ye kopyalamak yerine
dosyadan okutun; böylece anahtarın tek bir kaynağı olur:

```bash
# ~/.bashrc
[ -r "$HOME/.config/gemini/api_key" ] && export GEMINI_API_KEY="$(cat "$HOME/.config/gemini/api_key")"
```

`config/gemini_params.yaml`'a anahtar **KOYMAYIN**; o dosya repoya girer.
Anahtar hiçbir yerde bulunamazsa düğüm sessizce devam etmez, başlangıçta
nereye bakılacağını söyleyen bir hatayla düşer.

## Backend'ler

| backend | Gereksinim | Ne için |
|---|---|---|
| `gemini` (varsayılan) | `GEMINI_API_KEY` | ER 2 (`gemini-robotics-er-2-preview`) |
| `vertex` | GCP projesi (`vertex_project`) | Aynı model, Vertex AI üzerinden |

## Kullanım

```bash
ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py \
    use_vacuum_gripper:=true
```

### 1. Sadece algı (robot hareket etmez)

Algı düğümü zaten her zaman birlikte kalkıyor; robotu tamamen durdurmak için
`gemini_params.yaml` içinde `dry_run: true` yapın ya da tek başına çalıştırın:

```bash
ros2 run gemini_robotics_ros perception_node --ros-args \
    --params-file $(ros2 pkg prefix gemini_robotics_ros)/share/gemini_robotics_ros/config/gemini_params.yaml \
    --params-file $(ros2 pkg prefix gemini_robotics_ros)/share/gemini_robotics_ros/config/mode_sim.yaml

ros2 topic pub --once /gemini/query std_msgs/String "{data: 'the flat top face of the box'}"
ros2 topic echo /gemini/detections
```

RViz'de ekleyin:
- **`/gemini/er_image`** (Image) — ER 2'nin **gerçekte gördüğü** kare. "Model
  neden orayı gösterdi" sorusunun tek cevabı bu.
- **`/gemini/markers`** (MarkerArray) — tespit küreleri, etiket + düzlemsellik
  artığı, ve turuncu **normal okları**.

### 2. Pick and place

```bash
ros2 launch gemini_robotics_ros gemini_pick_place.launch.py            # sim
ros2 launch gemini_robotics_ros gemini_pick_place.launch.py mode:=real # gerçek kamera

ros2 topic pub --once /gemini/command std_msgs/String \
    "{data: 'düz kutunun üstünden tut ve konveyöre koy'}"
ros2 topic echo /gemini/status
```

Launch'ın **tek argümanı `mode`** var, çünkü sim ile gerçek arasındaki tek fark
kamera (ve saat):

| | `mode:=sim` (varsayılan) | `mode:=real` |
|---|---|---|
| bulut / derinlik | `/sim/pointcloud`, `/sim/depth/image` | `/points`, `/depth` |
| kodlama | 32FC1, metre | 16UC1, milimetre |
| `use_sim_time` | `true` | `false` |
| MoveIt | `/move_action` | `/move_action` — **aynı** |

Dosyaları: `config/mode_sim.yaml`, `config/mode_real.yaml`. İkisi de yalnızca
kamera/saat anahtarları içerir; başka ayar oraya konmamalı.

Robotu durdurmadan denemek için `gemini_params.yaml` içinde `dry_run: true`:
tüm algı ve normal hesabı çalışır, hedefler ve kuaterniyonlar loglanır, robot
hareket etmez.

### 3. API bağlantı testi (ROS'suz)

```bash
ros2 run gemini_robotics_ros er_probe --query "the flat surfaces" \
    --source render --render-mode normals
```

Derinlik + camera_info'dan tek kare üretir, ER 2'ye sorar, işaretli sonucu
`er_probe_out.png` olarak kaydeder. Gerçek robotta:
`--depth-topic /depth --info-topic /camera_info`.

### 4. Uç eleman geometrisini ölçmek

```bash
ros2 run gemini_robotics_ros measure_tcp
```

`tool_approach_vector` (emme yönü) ve `tool_tip_offset` (kap ağzının yeri)
**elle girilmez** — hatası doğrudan hareket mesafesine yazılır ve belirtisi
algı hatasına benzer: kol yanlış yerde durur, insan da kamerayı suçlar.
(11 Ağu 2026: değer 156 mm girilmişti, mesh'te ölçülen 84.2 mm; kap her
hedefin 71.8 mm üstünde duruyordu.)

Komut URDF'i `/robot_state_publisher`'dan — yani MoveIt'in planladığı modelden —
alır, link'in mesh'ini çözer, dönel yüzey ekseni uydurur, eksen boyunca en uzak
halkanın merkezini bulur ve yapıştırılacak YAML satırlarını yazar. Kap
değişirse tek yapılacak bu komutu tekrar çalıştırmaktır.

`pick_place_node` açılışta aynı ölçümü yapıp config'tekiyle karşılaştırır;
3 mm'den fazla fark varsa hata seviyesinde log basar (görevi durdurmaz).

## Taşınan parça

Kavranan parça artık robotun bir parçasıdır ve iki yerde birden temsil edilir:

| Nerede | Ne | Nasıl |
|---|---|---|
| Gazebo | Parça gerçekten taşınsın | `DetachableJoint` eklentisi, `/vacuum/attach` ve `/vacuum/detach` |
| MoveIt | Planlayıcı parçayı görsün | Sahneye kutu eklenir ve `ur10e_suction_cup`'a iliştirilir |

MoveIt tarafı **gerçek robotta da gereklidir**: bilinmezse kol parçayı rafa,
konveyöre ya da kendine sürter.

Kutunun boyutu **derinlikten ölçülür** (`payload_measure: true`): uzunluk,
genişlik, yükseklik ve düzlem içi yönelim. Yükseklik en güvenilir olanıdır —
üst yüzün ve altındaki destek düzleminin baskın seviyeleri arasındaki fark,
yüzlerce pikselden hesaplanır.

Ölçemediği şeyler, hepsi **sessizce eksik** sonuç verir:

- Destek düzleminin **altı** — ayaklı ya da oyuk tabanlı parçada siluet ölçülür
- Parça düz bir yüzeyin üstünde durmuyorsa (başka parçanın üstünde, gözün
  içinde) referans düzlem yanlış olur
- Oklüzyonda kalan yüzler → ayak izi eksik çıkar
- ToF'un okuyamadığı malzemeler: mat siyah, parlak metal, şeffaf

Bu yüzden sonuç `payload_margin_m` kadar şişirilir ve ölçüm başarısız olursa
`payload_size`'a düşülür. Log hangisinin kullanıldığını yazar.

## Topic'ler

| Topic | Tip | Yön |
|---|---|---|
| `/gemini/query` | `std_msgs/String` | giriş — pointing sorgusu |
| `/gemini/command` | `std_msgs/String` | giriş — pick and place komutu |
| `/gemini/detections` | `std_msgs/String` (JSON) | çıkış — 3D konum + `surface` (normal, quat, artık) |
| `/gemini/er_image` | `sensor_msgs/Image` | çıkış — ER 2'nin gördüğü kare |
| `/gemini/markers` | `visualization_msgs/MarkerArray` | çıkış — küre + etiket + normal oku |
| `/gemini/status` | `std_msgs/String` (JSON) | çıkış — görev durumu |
| `/OnRobotVGOutput` | `vgc10_msgs/OnRobotVGOutput` | çıkış — vakum komutu |

### Sim → gerçek topic geçişi

`config/gemini_params.yaml` içinde:

```yaml
# SİM (/sim namespace'li köprü)          # GERÇEK (sick_visionary_t_mini_ur)
cloud_topic:       "/sim/pointcloud"     # "/sick_points"
depth_topic:       "/sim/depth/image"    # "/depth"
camera_info_topic: "/sim/camera_info"    # "/camera_info"
intensity_topic:   "/intensity"          # "/intensity"
image_topic:       "/sim/image"          # YOK - RGB yok, er_image_source
                                         #       "render" kalmalı
```

## Ayarlanması gereken parametreler

Sahneye özgü, varsayılanları tahmin — canlı çalıştırmadan önce elden geçirin:

- **`scan_joints`** — bilek kamerasının tezgahı gördüğü poz (7 değer: ray + 6 UR
  eklemi). Kamera kolla hareket ettiği için sorgudan önce sabit bir bakış pozu şart.
- **`workspace_min` / `workspace_max`** — güvenlik kutusu. Hedef dışına düşerse
  hareket **hiç planlanmaz**; kötü derinlik okumasına karşı tek koruma.
- **`render_min_m` / `render_max_m`** — hücrenizin çalışma mesafesi. Sim ve
  gerçekte **aynı** olmalı, yoksa parite bozulur.
- **`max_surface_rms`** — 4 mm varsayılan. Parçalarınız pürüzlüyse gevşetin,
  ama gevşettiğiniz kadar kenardan tutma riski artar.
- **`vacuum_require_confirmation`** — sim'de `false` (VGC10 sürücüsü yok, durum
  mesajı gelmez), **gerçek robotta `true`**: ancak o zaman "tuttu mu" gerçekten
  doğrulanır.

## Bilinen sınırlar

- **Gazebo'da vakum fiziksel olarak tutmaz.** Hücrede attach/detach ya da
  suction eklentisi yok (`my_robot_cell_macro.xacro:217` bunu planlamış ama
  eklenti hiç eklenmemiş). Sim'de doğrulanabilen: algı, normal, poz, hareket ve
  komut akışı. Parçanın gerçekten kalkması yalnız gerçek robotta görülür.
  Sim'de de tutturmak isterseniz Gazebo Harmonic'in `DetachableJoint` sistemi
  eklenebilir.
- ER 2 tek bir 2D nokta verir; kavrama oryantasyonu bizim düzlem oturtmamızdan
  gelir. Yama yarıçapı (`patch_radius_px`) emme kabından küçük olursa normal
  gürültülü, büyük olursa komşu yüzeyler karışır.
- Sorgu başına bir API çağrısı var ve saniyeler sürebiliyor. Kapalı döngü
  görsel servo için uygun değil; "bak → planla → hareket et" döngüsü için uygun.
- `gemini-robotics-er-1.6-preview` 31 Ağustos 2026'da kapanıyor. Bu paket ER 2
  kullanır, etkilenmez.
