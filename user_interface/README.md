# Robotic Testbed as a Service

**ESOGÜ IFARLAB — Control Dashboard**

Bu proje, ESOGÜ IFARLAB hücresindeki robotik senaryoları (HIL — Hardware-in-the-Loop ve
gerçek ortam) başlatmak, durdurmak, elle sürmek ve canlı olarak izlemek için geliştirilmiş
web tabanlı bir arayüzdür. Tek bir Flask + SocketIO sunucusu (`app.py`) dört sekme sunar:
**Home**, **Data Analytics**, **Anomaly** ve **Free Move**.

<img width="1181" height="450" alt="image" src="https://github.com/user-attachments/assets/0c569df9-11c5-4900-becf-462e4a8c86b8" />

> Güncelleme: 12 Eylül 2026. Bu sürüm Free Move ve Anomaly sekmelerini, çalışma kimliği
> (run id) damgalamasını ve Data Analytics panel düzenlerini kapsar.

---

## 🚀 Sekmeler ve Özellikler

### 1. Home — senaryo yönetimi ve canlı izleme

- **Dört kullanım senaryosu.** Her biri arka planda önce birleşik HIL launch dosyasını,
  sonra kendi senaryo launch'ını başlatır:

  | Senaryo | Kullanım etiketi | HIL parametresi | Senaryo komutu |
  |---|---|---|---|
  | Multi-Robot Inspection | `MULTIROBOT_INSPECTION` | — | `multirobot_viewpoint_planner multirobot_inspection.launch.py` |
  | UR10e Inspection | `UR10E_INSPECTION` | — | `viewpoint_planner inspection_execution.launch.py` |
  | Pick & Place | `PICKPLACE` | `use_vacuum_gripper:=true` | `gemini_robotics_ros gemini_pick_place.launch.py` |
  | Human-Robot Collaboration | `HRC` | `use_gripper:=true` | `pymoveit2_real human_robot_collaboration_scenario.launch.py` |

- **Use Fake Hardware anahtarı.** Açıkken HIL'e `use_fake_hardware:=true use_mock_hardware:=true
  fake_sensor_commands:=true` eklenir ve senaryoya doğru bayrak geçirilir: muayene
  senaryolarında `only_sim:=true/false`, pick & place'te ise argüman adı farklı olduğu için
  `mode:=sim|real`.
- **Veri toplama anahtarı (data acquisition).** Açıkken HIL'e `data_acquisition:=true` eklenir;
  ROS 2 → Kafka köprüsü yalnız bu durumda veri akıtır.
- **Senaryo tamamlanmasını izleme.** `ros2 launch` süreci bitmeyen senaryolar var: muayene
  launch'ları kalıcı bir görselleştirici düğümü de başlatır ve turun bitmesine rağmen hiç
  çıkmaz. Arayüz bu yüzden sürecin ölmesini değil, senaryonun kendi tek seferlik yürütücü
  süreçlerini (`inspection_node`, `inspection_executor_node`) izler ve onlar bitince senaryoyu
  "tamamlandı" olarak işaretler. HRC senaryosu kendi launch'ında `on_exit=Shutdown` taşıdığı
  için normal yoldan kapanır.
- **Komut penceresi.** Pick & Place senaryosunda görev serbest metinle verilir; arayüz senaryo
  ayağa kalkınca komut kutusunu açar ve metni `/gemini/command` konusuna yayımlar.
- **Acil durdurma (🛑 STOP ALL).** Üç aşamalı kapatma: `SIGINT` → `SIGTERM` → `SIGKILL`, süreç
  grubu üzerinden.
- **Canlı kamera akışları (MJPEG).** Simülasyon tarafı `gz.transport` ile `/web_camera/image`;
  gerçek taraf OpenCV ile RTSP (`rtsp://192.168.3.51:554/live/0`). RTSP düşerse arayüz geçici
  olarak RViz penceresini yakalamaya düşer.
- **Canlı eklem grafikleri.** `/joint_states` ve `/sim/joint_states` 20 saniyelik kayan
  pencerede karşılaştırmalı çizilir.
- **Sistem logları.** Terminal çıktısı anlık olarak arayüze basılır ve `user_interface/log/`
  altındaki `dashboard_log_<zaman>.txt` dosyasına yazılır.

### 2. Data Analytics — toplanan verinin salt-okunur analizi

Elasticsearch'e yalnızca `_search` istekleri atan bir proxy üzerinden çalışır; üretim veri
hattına hiçbir şey yazılmaz.

- **Üç katman:** *filtreler* (kullanım senaryosu çipleri + serbest filtre hapları, her panele
  uygulanır), *paneller* (her biri `app.py`'deki bir toplama uç noktasına bağlı) ve *düzen*
  (paneller koddan değil veriden gelir; eklenip silinebilir ve `localStorage`'a kaydedilir).
- **Panel tipleri:** line, envelope (min/ortalama/maks bandı), histogram, box, heatmap,
  scatter2d, scatter3d (TCP yolu; şasi mesh'i altlık olarak çizilebilir) ve bar.
- **Dinamik indirgeme.** Büyük seriler `date_histogram` ile kümelenip ortalaması alınarak
  gönderilir; tarayıcı yüz binlerce noktayla boğuşmaz.
- **Otomatik odaklama.** `/api/es/range` ile verinin gerçek min/maks zaman damgası bulunur ve
  grafik oraya odaklanır.
- **Simülasyon gürültüsü filtresi.** Gazebo saati sıfırdan başladığı için oluşan 1970 tarihli
  damgalar sorgu düzeyinde elenir.
- **Birim doğruluğu (11 Eylül 2026 düzeltmesi).** UR10e'nin `effort` alanı iki indekste iki
  ayrı büyüklüktür: gerçek robotta sürücü motor AKIMINI (A) yazar, Gazebo tarafında ise eklem
  TORKU (Nm) gelir. İki panel eskiden ortak bir y ekseni paylaşıyordu ve yüzlerce Nm'lik sim
  torku, birkaç amperlik gerçek sinyali sıfıra yapıştırıyordu. Paneller ayrıldı, başlıklar ve
  birimler düzeltildi; farklı birimli paneller artık hiçbir koşulda eksen paylaşmıyor.

<img width="1917" height="476" alt="image (1)" src="https://github.com/user-attachments/assets/440d1e75-223d-4ab1-b3eb-7d62543f6e70" />
<img width="1917" height="450" alt="image" src="https://github.com/user-attachments/assets/c7a40603-0891-4d25-8c8e-7f3f485b7895" />

### 3. Anomaly — canlı anomali izleme ve etiketleme

- `anomaly_detection` düğümünün kararlarını dinler ve 5 Hz'te arayüze basar.
- **Alarm kenar olarak yakalanır**, seviye olarak değil: dedektör 20 Hz karar üretir, arayüz
  5 Hz beslenir; anlık değere bakmak kısa olayları kaçırır (ölçülen en net olay 0.25 s sürdü).
- **Eşikler canlı mesajdan alınır.** Sabit eşikler kodda yalnızca yedek olarak durur; dedektör
  yeniden kalibre edildiğinde (füzyon eşiği 18.0 → 1.4) sabitlere güvenen bir arayüz alarm
  çizgisini yanlış yere çizerdi.
- **Olay tablosu ve etiketleme.** `~/anomali_kayit/**/olaylar_*.jsonl` dosyalarındaki
  "başladı/bitti" kayıt çiftleri eşleştirilir; tablonun birincil sütunu giriş değeri değil
  TEPE değeridir (tepe yalnız "bitti" kaydında bulunur). Etiketler `etiketler.json` dosyasına
  atomik olarak yazılır (`/api/anomaly/label`).

### 4. Free Move — sürükle-bırak ile kol sürme

- Tarayıcıda three.js ile hücrenin URDF'i yüklenir; TCP hedefi bir gizmo ile sürüklenir,
  arka planda MoveIt plan üretir ve onaydan sonra çalıştırılır.
- **Hayalet (ghost) önizleme:** planlanan poz turuncu hayalet olarak çizilir; yalnızca kolun
  gerçekten hareketli linkleri gösterilir, hücre yapısı hayalete dahil edilmez.
- **Z-yukarı sahne.** Sahne grafiği hiç döndürülmez; kamera `up` vektörü (0,0,1) yapılır.
  Eski sürümdeki -90° döndürülmüş sarmalayıcı, ekrandaki okların ROS eksenleriyle
  uyuşmamasına ve "bir ekseni sürüklerken robotun başka eksende hareket etmesine" yol açıyordu.
- **Eklem kaydırıcıları** URDF'ten okunan gerçek limitlerle sınırlanır.
- **Plan / Execute ayrımı:** `execute()` her zaman saklanan planı çalıştırır. `move_to_pose()`
  ve `move_to_configuration()` sessizce yeniden planladığı için hayalet ile gerçek robot
  birbirinden ayrılıyordu.
- **Ayarlar:** planlama denemesi (10), planlama süresi (5 s), planlayıcı, hız/ivme ölçeği
  (UR 0.1, Kawasaki 0.02) ve çarpışma payı (varsayılan 0.04 m) arayüzden değiştirilebilir.
- Free Move ile senaryolar **aynı anda çalışamaz**: ikisi de HIL'i kendi biçiminde ayağa
  kaldırır, bu yüzden biri açıkken diğeri reddedilir.

---

## 🏷 Kullanım Etiketi ve Çalışma Kimliği (use case / run id)

Arayüz, hangi verinin hangi senaryodan geldiğini ROS 2 tarafına iki latched konudan bildirir:

```text
/testbed/use_case   -> MULTIROBOT_INSPECTION | UR10E_INSPECTION | PICKPLACE | HRC | IDLE
/testbed/run_id     -> senaryo başına üretilen benzersiz çalışma kimliği
```

`use_case` verinin HANGİ senaryodan geldiğini, `run_id` ise HANGİ ÇALIŞMADAN geldiğini söyler.
İkincisi olmadan yedinci pick & place koşusu ilkinden ayırt edilemez ve "dünkü koşuyla bugünküyü
karşılaştır" sorusu — bir test yatağına asıl sorulan soru — ifade edilemez. ROS 2 → Kafka köprüsü
bu iki alanı ilettiği her belgeye damgalar; Data Analytics sekmesindeki kullanım senaryosu
çipleri de aynı alana dayanır.

---

## 📁 Proje Yapısı

```text
user_interface/
├── app.py                  # Flask + SocketIO sunucusu: senaryo yönetimi, ES proxy,
│                           # kamera akışları, joint/anomali toplayıcıları, use case yayını
├── free_move.py            # Free Move'un ROS 2 tarafı: iki kol için MoveIt sarmalayıcı
├── requirements.txt        # Python bağımlılıkları
├── README.md               # Bu dokümantasyon
├── log/                    # Çalışma zamanı log dosyaları (dashboard_log_*.txt)
├── figures/                # Laboratuvar logoları (ifarlab, asrlab, matisse)
├── static/
│   ├── css/style.css
│   └── js/
│       ├── dashboard.js    # Home sekmesi: senaryo kontrolü, canlı grafikler, loglar
│       ├── analytics.js    # Data Analytics: filtreler, paneller, düzen kalıcılığı
│       ├── anomaly.js      # Anomaly: canlı seri, olay tablosu, etiketleme
│       └── freemove.js     # Free Move: three.js sahnesi, gizmo, hayalet önizleme
└── templates/
    ├── index.html          # Dört sekmeli ana şablon
    └── freemove_macros.html
```

---

## 🔌 HTTP ve SocketIO Arayüzü

**Durum ve sağlık:** `GET /api/status`, `GET /api/health`
**Kamera:** `GET /stream/gazebo`, `GET /stream/real`
**Free Move varlıkları:** `GET /freemove/urdf`, `/freemove/mesh/<pkg>/<path>`, `/freemove/localmesh/<path>`
**Anomali:** `GET /api/anomaly/events`, `POST /api/anomaly/label`
**Elasticsearch proxy (salt okunur):** `/api/es/indices`, `/fields`, `/terms`, `/use_cases`,
`/range`, `/time_field`, `/field_summary`, `/query_preview`, `/overview`, `/timeseries`,
`/histogram`, `/percentiles`, `/stats`, `/points`, `/scatter3d`, `/docs`
**Şasi mesh'i (scatter3d altlığı):** `/api/mesh/chassis`

**SocketIO olayları:** `start_scenario`, `confirm_robot`, `send_command`, `stop_all`,
`request_health`, `freemove_start`, `freemove_stop`, `freemove_check`, `freemove_plan`,
`freemove_execute`, `freemove_plan_joint`, `freemove_execute_joint`, `freemove_cancel`,
`freemove_set_params`, `freemove_set_padding`.

---

## 🛠 Kurulum ve Çalıştırma

### 1. Bağımlılıklar

```bash
pip install -r requirements.txt
```

ROS 2 Humble ortamının kurulu, `cv2`, `flask_socketio` ve `gz.transport` paketlerinin
yüklü olması gerekir.

### 2. Çalışma alanını derleyin

```bash
cd ~/colcon_ws
colcon build
source install/setup.bash
```

`app.py` açılışta çalışma alanını kendisi de source etmeye çalışır, ancak senaryoların
güncel kodla koşması için derleme yine de şarttır.

### 3. Arayüzü başlatın

```bash
cd ~/colcon_ws/src/user_interface
python3 app.py
```

Varsayılan port **8080**'dir; `DASHBOARD_PORT` ortam değişkeniyle değiştirilebilir.
Elasticsearch adresi `ES_URL` ile verilir (varsayılan `http://localhost:9200`).

```text
http://localhost:8080
```

Aynı ağdaki başka cihazlardan sunucunun IP adresiyle de erişilebilir.

---

## ⚙️ Mimari Notlar

- **Alt süreç yönetimi.** `ScenarioManager`, ROS 2 komutlarını `/bin/bash` üzerinden, kendi
  süreç grubunda başlatır. Gazebo ve RViz'in düşmemesi için ortam kopyalanır ve OpenCV'nin
  soktuğu `QT_QPA_PLATFORM_PLUGIN_PATH` temizlenir.
- **Dinamik MoveIt yapılandırması.** HIL launch'ına geçilen `use_gripper` / `use_vacuum_gripper`
  bayrakları, arka planda doğru MoveIt paketinin seçilmesini sağlar
  (`real_ifarlab_gripper_moveit_config`, `real_ifarlab_vacuum_moveit_config`, …).
- **Thread modeli.** Flask-SocketIO senkron çalışır; `rclpy.spin`, RTSP yakalama ve log okuma
  işlerinin her biri kendi `threading.Thread`'inde döner. Uzun süren işler (Free Move'un iki
  kolu ayağa kaldırması ~60 s sürebilir) kilidin dışında yapılır ki acil durdurma ve durum
  sorguları bloklanmasın.
- **Salt-okunur veri erişimi.** Elasticsearch'e yalnızca `_search` gider; arayüz üretim veri
  hattına yazmaz.

---

## 🧯 Sık Karşılaşılan Durumlar

- **Senaryo "çalışıyor" görünüyor ama tur bitti.** Muayene launch'ları kalıcı görselleştirici
  düğümü yüzünden hiç çıkmaz; arayüz yürütücü süreçlerini izler. Yine de takılırsa STOP ALL.
- **Free Move açılmıyor.** Bir senaryo çalışıyordur; ikisi aynı anda HIL'e sahip olamaz.
- **Data Analytics panelleri boş.** Veri toplama anahtarı kapalı başlatılmış olabilir
  (`data_acquisition:=true` gerekir) ya da seçili zaman aralığında kayıt yoktur; "fit to data"
  düğmesi veriye odaklar.
- **Kayıtlı panel düzeni eskidi.** Düzen `localStorage`'da (`esogu.analytics.layout.v1`)
  saklanır; hazır panellerde sonradan düzeltilen alanlar (başlık, birim, eksen eşleşmesi)
  yüklenirken yerleşik tanımdan zorla yenilenir.
