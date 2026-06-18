# ESOGÜ Robotics Lab — Scenario Control Dashboard

Bu proje, ESOGÜ Robotics Lab sistemindeki robotik senaryoları (HIL - Hardware-in-the-Loop ve gerçek ortam) başlatmak, durdurmak ve canlı olarak izlemek için geliştirilmiş modern, web tabanlı bir arayüzdür. 

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
  - *Gerçek Dünya:* OpenCV kullanarak RTSP stream (`rtsp://192.168.4.51:554/live/0`) üzerinden gelen gerçek laboratuvar kamerasını canlı aktarır.
- **Canlı Eklem (Joint) Grafikleri:** `/joint_states` ve `/sim/joint_states` verilerini okuyarak, gerçek robot ve simülasyon verilerini karşılaştırmalı olarak Chart.js üzerinden 20 saniyelik hareketli bir grafikte gösterir.
- **Sistem Logları:** Terminal çıktılarını anlık olarak arayüze yansıtır ve geçmişe dönük hata ayıklama için `user_interface/log/` dizinindeki `.txt` dosyalarına kaydeder.
- **Sağlık Kontrolü (Health Monitor):** ROS 2 topic'lerini ve controller_manager'ların çalışıp çalışmadığını periyodik olarak kontrol ederek renkli (Yeşil/Kırmızı) göstergelerle kullanıcıyı bilgilendirir.

## 📁 Proje Yapısı

```text
user_interface/
├── app.py                # Ana Flask & SocketIO sunucusu (Backend)
├── requirements.txt      # Gerekli Python bağımlılıkları
├── README.md             # Bu dökümantasyon
├── log/                  # Çalışma zamanında oluşturulan sistem log dosyaları
├── figures/              # Laboratuvar logoları (ifarlab, asrlab, matisse vb.)
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
