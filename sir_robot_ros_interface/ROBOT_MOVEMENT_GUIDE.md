# Kawasaki Robot Hareket ve Gazebo Senkronizasyonu Kılavuzu

## 🔧 Sorun ve Çözümü

### ❌ Eski Sorun
İki ayrı node kullanıldığında:
- `ROS2KawasakiRobotGazeboJSP` - Robot pozisyonunu okuyor
- `ROS2KawasakiRobotMove` - Robot'a komut gönderiyor

**Problem:** Kawasaki robot protokolü aynı anda sadece **TEK BİR BAĞLANTI** destekliyor!
İkinci node hata alıyordu: `Invalid packet type` - `could not receive correct answer`

### ✅ Yeni Çözüm
Tek bir node içinde hem komut gönder hem pozisyon oku:
- `ROS2KawasakiRobotMove` - Artık her ikisini de yapıyor!

## 📋 Nasıl Kullanılır

### Tek Node ile Çalıştırma (ÖNERİLEN)

```bash
# Terminal 1: Gazebo'yu başlat (eğer kullanıyorsan)
ros2 launch your_gazebo_package gazebo_kawasaki.launch.py

# Terminal 2: Yeni birleşik node'u çalıştır
ros2 run sir_robot_ros_interface ROS2KawasakiRobotMove
```

Bu node:
1. ✅ Gerçek robot'a bağlanır
2. ✅ Waypoint'leri gönderir
3. ✅ Robot hareket ederken pozisyonu okur
4. ✅ Gazebo'ya pozisyonu yayınlar (joint_states ve joint_trajectory)
5. ✅ Hareket bitene kadar döngü devam eder

### Eski Node'u Sadece Monitoring için Kullanma (ALTERNATIF)

Eğer sadece robot'u izlemek istiyorsan (komut göndermeden):

```bash
# Sadece pozisyon okuma ve Gazebo'ya yayınlama
ros2 run sir_robot_ros_interface ROS2KawasakiRobotGazeboJSP
```

⚠️ **DİKKAT:** İki node'u aynı anda ÇALIŞTIRMAYIN!

## 🔍 Yapılan Değişiklikler

### 1. Robot Modu Düzeltmesi
```cpp
// YANLIŞ (eski):
KawasakiRS005LRobot robot(con, logger, nullptr, MPT_TASK, MT_LINEAR);
// MPT_TASK = Kartezyen koordinatlar (X, Y, Z, Roll, Pitch, Yaw)

// DOĞRU (yeni):
KawasakiRS005LRobot robot(con, logger, nullptr, MPT_JOINT, MT_P2P);
// MPT_JOINT = Eklem açıları (Joint angles - derece cinsinden)
// MT_P2P = Point-to-Point hareket
```

**Neden?** Waypoint değerleriniz derece cinsinden eklem açılarıydı:
```cpp
point1 << 15.837, -28.815, -76.154, -154.640, 31.511, 47.310;  // DERECE
```

Eski kodda robot bunları mm/derece Kartezyen koordinat olarak yorumluyordu!

### 2. Gazebo Entegrasyonu Eklendi
```cpp
// ROS2KawasakiRobotMove.cpp içine eklendi:
- sensor_msgs/msg/JointState yayınlama
- trajectory_msgs/msg/JointTrajectory yayınlama
- deg2rad() dönüşüm fonksiyonu
- Hareket sırasında sürekli pozisyon okuma (10 Hz)
```

## 📊 ROS2 Topic'ler

Node şu topic'lere yayın yapar:

```bash
# Joint durumları (monitoring için)
/kawasaki/joint_states              [sensor_msgs/msg/JointState]

# Gazebo kontrolcüsü için komutlar
/kawasaki/kawasaki_controller/joint_trajectory  [trajectory_msgs/msg/JointTrajectory]
```

Topic'leri izlemek için:
```bash
# Joint durumlarını izle
ros2 topic echo /kawasaki/joint_states

# Trajectory komutlarını izle
ros2 topic echo /kawasaki/kawasaki_controller/joint_trajectory
```

## 🐛 Hata Ayıklama

### "Invalid packet type" hatası alıyorsanız:
- ✅ Sadece TEK bir node çalıştığından emin olun
- ✅ Başka bir program robot'a bağlı değil mi kontrol edin
- ✅ Robot IP ve port doğru mu kontrol edin

### Gazebo'da hareket görmüyorsanız:
- ✅ Gazebo kontrolcü çalışıyor mu?
- ✅ Topic isimleri doğru mu? (`ros2 topic list`)
- ✅ QoS ayarları uyumlu mu? (BEST_EFFORT kullanıyoruz)

### Robot beklenmedik hareket ediyorsa:
- ✅ MPT_JOINT modu kullanılıyor mu?
- ✅ Waypoint değerleri derece cinsinden mi?
- ✅ Derece → Radian dönüşümü yapılıyor mu?

## 📝 Örnek Waypoint Tanımlama

```cpp
std::vector<SIRMatrix> createWaypoints()
{
  std::vector<SIRMatrix> waypoints;
  
  // Her nokta 6 eklem açısı içerir (DERECE cinsinden)
  SIRMatrix point1(6, 1);
  point1 << 15.837,   // Joint 1
           -28.815,   // Joint 2
           -76.154,   // Joint 3
          -154.640,   // Joint 4
            31.511,   // Joint 5
            47.310;   // Joint 6
  waypoints.push_back(point1);
  
  return waypoints;
}
```

## 🎯 Özet

| Özellik | Eski Sistem | Yeni Sistem |
|---------|-------------|-------------|
| Node Sayısı | 2 (çakışma!) | 1 ✅ |
| Robot Bağlantısı | Çift bağlantı ❌ | Tek bağlantı ✅ |
| Koordinat Sistemi | TASK (yanlış) ❌ | JOINT (doğru) ✅ |
| Gazebo Sync | Ayrı node | Entegre ✅ |
| Hareket Modu | LINEAR | P2P ✅ |

## 📚 İlgili Dosyalar

- `ROS2KawasakiRobotMove.cpp` - Ana hareket ve senkronizasyon node'u
- `ROS2KawasakiRobotGazeboJSP.cpp` - Sadece monitoring için (opsiyonel)
- `KawasakiRobotTestTrajectoryExecution.cpp` - Örnek test kodu
