# Robotic Testbed as a Service

**ESOGÜ IFARLAB — Control Dashboard**

Bu proje, ESOGÜ IFARLAB sistemindeki robotik senaryoları (HIL - Hardware-in-the-Loop ve gerçek ortam) başlatmak, durdurmak ve canlı olarak izlemek için geliştirilmiş modern, web tabanlı bir arayüzdür. 
<img width="1181" height="450" alt="image" src="https://github.com/user-attachments/assets/0c569df9-11c5-4900-becf-462e4a8c86b8" />


## 🚀 Özellikler

- **Modern ve Responsive Arayüz:** Karanlık tema (Dark Mode) ve Glassmorphism tasarımı ile tarayıcı üzerinden kolay kontrol.
- **Senaryo Yönetimi:** 
  - *Multi-Robot Inspection Scenario*
  - *UR10e Inspection Scenario*
  - *Pick & Place Scenario*
  - *Human-Robot Collaboration Scenario*
- **Süreç Yaşam Döngüsü (Lifecycle) Kontrolü:** Başlatılan ROS 2 düğümlerini ve launch dosyalarını takip eder. Yeni bir senaryo başlatıldığında eskisini güvenli bir şekilde kapatır.
- **Acil Durdurma (🛑 STOP ALL):** Gerçek donanıma zarar gelmemesi için 3 aşamalı güvenli kapatma protokolü kullanır (`SIGINT` -> `SIGTERM` -> `SIGKILL`).
- **Use Fake Hardware:** Gerçek robotlara ve donanımlara bağlı olunmayan durumlarda sadece simülasyon üzerinden test yapmak için arayüzden aktif edilebilen anahtar (`use_fake_hardware:=true`, vb. parametreleri otomatik ekler).
- **Canlı Kamera Akışları (MJPEG Streaming):** 
  - *Simülasyon (Gazebo):* `gz.transport` üzerinden `/web_camera/image` topic'ini okuyarak tarayıcıya yansıtır.
  - *Gerçek Dünya:* OpenCV kullanarak RTSP stream (`rtsp://192.168.3.51:554/live/0`) üzerinden gelen gerçek laboratuvar kamerasını canlı aktarır.
- **Canlı Eklem (Joint) Grafikleri:** `/joint_states` ve `/sim/joint_states` verilerini okuyarak, gerçek robot ve simülasyon verilerini karşılaştırmalı olarak Chart.js üzerinden 20 saniyelik hareketli bir grafikte gösterir.
- **Sistem Logları:** Terminal çıktılarını anlık olarak arayüze yansıtır ve geçmişe dönük hata ayıklama için `user_interface/log/` dizinindeki `.txt` dosyalarına kaydeder.
- **Veri Analitiği (Data Analytics):** Arka planda çalışan `ROS2 → Kafka → Elasticsearch → MariaDB → Grafana` veri hattından beslenen metrikleri, Elasticsearch üzerinden salt-okunur (read-only) proxy ile sorgulayarak arayüzde geçmişe dönük veri analizi ve görselleştirme imkanı sunar.
  - *Güvenli Veri Erişimi:* Elasticsearch veritabanına sadece okuma (`_search`) istekleri atılarak mevcut üretim veri hattının bozulması tamamen engellenir.
  - *Dinamik Zaman Serisi İndirgeme (Downsampling):* Tarayıcıyı yormamak adına büyük veri setleri (`/api/es/timeseries`) `date_histogram` kullanılarak kümelenir ve ortalamaları alınarak gösterilir.
  - *Otomatik Odaklama (Fit to Data):* Kayıtlardaki min/max zaman damgaları (`/api/es/range`) otomatik algılanarak grafiğin sadece verinin olduğu anlamlı bölgeye odaklanması sağlanır.
  - *Simülasyon Gürültüsü Filtreleme:* Gazebo simülasyon saatinin "0" noktasından başlamasıyla oluşan 1970 yılına ait "hatalı" zaman damgaları (timestamp) sorgu düzeyinde filtrelenerek grafiklerin dışına atılır.
<img width="1917" height="476" alt="image (1)" src="https://github.com/user-attachments/assets/440d1e75-223d-4ab1-b3eb-7d62543f6e70" />
<img width="1917" height="450" alt="image" src="https://github.com/user-attachments/assets/c7a40603-0891-4d25-8c8e-7f3f485b7895" />

## 📁 Proje Yapısı

```text
user_interface/
├── app.py                # Ana Flask & SocketIO sunucusu (Backend)
├── requirements.txt      # Gerekli Python bağımlılıkları
├── README.md             # Bu dökümantasyon
├── log/                  # Çalışma zamanında oluşturulan sistem log dosyaları
├── figures/              # Laboratuvar logoları (cem, asrlab, matisse vb.)
├── static/
│   ├── css/
│   │   └── style.css     # Özelleştirilmiş arayüz stilleri
│   └── js/
│       └── dashboard.js  # Arayüz mantığı, grafikler ve SocketIO bağlantıları
└── templates/
    └── index.html        # Ana HTML şablonu
```

## 🛠 Kurulum ve Çalıştırma

### 1. Bağımlılıkların Yüklenmesi
Uygulama, Flask, OpenCV ve `gz.transport` gibi kütüphanelere ihtiyaç duyar.
Çalıştırmadan önce sisteminizde ROS 2 ortamının kurulu ve `cv2`, `flask_socketio` gibi paketlerin yüklü olduğundan emin olun:
```bash
pip install -r requirements.txt
```

### 2. ROS2 Paketlerinin Güncellenmesi
Arayüzün en verimli şekilde çalışabilmesi için ROS2 Paketlerinin arayüz çalıştırılmadan önce
güncellenmesi gerekir.
```bash
cd colcon_ws
colcon build
source install/setup.bash
```

### 2. Uygulamayı Başlatma
Terminalde projenin dizinine gidin ve `app.py` dosyasını çalıştırın:
```bash
cd ~/colcon_ws/src/user_interface
python3 app.py
```

### 3. Arayüze Erişim
Arayüz varsayılan olarak `8080` portunda ayağa kalkacaktır. Herhangi bir web tarayıcısından aşağıdaki adrese giderek dashboard'a erişebilirsiniz:
```
http://localhost:8080
```
*(Eğer aynı ağdaysanız, sunucuyu çalıştıran bilgisayarın IP adresi ile diğer cihazlardan da erişim sağlayabilirsiniz).*

## ⚙️ Alt Sistem Detayları ve Mimari

- **Subprocess Yönetimi:** Uygulama, `app.py` içindeki `ScenarioManager` sınıfı yardımıyla ROS 2 komutlarını alt süreç (subprocess) olarak başlatır. Grafik uygulamalarının (Gazebo, RViz vb.) hata vermemesi için komutlar `/bin/bash` üzerinde `os.environ` kopyalanarak çalıştırılır ve OpenCV'nin neden olabileceği `QT_QPA_PLATFORM_PLUGIN_PATH` uyumsuzlukları otomatik olarak temizlenir.
- **Dinamik MoveIt! Yapılandırması:** `hil_test_whole_unified.launch.py` üzerinden gönderilen argümanlar sayesinde (`use_vacuum_gripper`, `use_gripper`), arka planda doğru MoveIt! konfigürasyon paketi otomatik olarak seçilir ve çalıştırılır.
- **Thread Yönetimi:** Flask-SocketIO senkron çalışırken arka planda bloklanmaları engellemek amacıyla ROS 2 Node dönüşleri (`rclpy.spin`), RTSP stream yakalamaları ve log okuma işlemleri kendi bağımsız `threading.Thread` yapıları içerisinde çalışır.
