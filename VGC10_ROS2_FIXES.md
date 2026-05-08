# VGC10 ROS2 Uyumluluk Düzeltmeleri

## Sorun Özeti

`vgc10_control` ve `vgc10_modbus_tcp` paketleri orijinal olarak ROS1 ve eski pymodbus sürümleri için yazılmıştı. ROS2 Humble + pymodbus v3 ortamında çalışmıyordu.

---

## Yapılan Değişiklikler

### 1. `vgc10_modbus_tcp/vgc10_modbus_tcp/comModbusTcp.py` — Tam yeniden yazım

**Sorun 1:** `pymodbus.client.sync` modülü pymodbus v3'te kaldırıldı.
```
ModuleNotFoundError: No module named 'pymodbus.client.sync'
```

**Sorun 2:** pymodbus v3'te `unit` parametresi `slave` olarak değiştirildi.
```
TypeError: ModbusClientMixin.read_holding_registers() got an unexpected keyword argument 'unit'
```

**Sorun 3:** `ModbusTcpClient` TCP bağlantısı kuruyor ama OnRobot Compute Box'a istek gönderdiğinde yanıt gelmiyor. pymodbus'un MBAP frame oluşturma biçimi Compute Box ile uyumsuz.

**Çözüm:** pymodbus tamamen kaldırıldı. `socket` + `struct` ile ham Modbus TCP implementasyonu yazıldı. Bu yöntem `onrobot_api` (C++/libmodbus) ile birebir aynı frame yapısını kullanıyor.

**Sorun 4:** Slave ID yanlıştı. OnRobot Compute Box'ta Tool1=**65**, Tool2=**66**. Gripper Tool2 portuna bağlıydı → `SLAVE_ID = 66`.

**Sorun 5:** `struct.pack` format string hatası (`BHHB` yerine `BBHHB` olmalıydı).

---

### 2. `vgc10_control/vgc10_control/OnRobotVGTcpNode.py` — Mimari yeniden düzenleme

**Sorun 1:** `threading.Thread(target=rclpy.spin, ...)` her döngü iterasyonunda yeni bir thread açıyordu.
```
ValueError: generator already executing
```

**Sorun 2:** `node.create_rate(0.05)` → 0.05 Hz = 20 saniyede bir döngü. Komut çok geç işleniyordu.

**Sorun 3:** `node.get_parameter(...)` sonucuna `.value` eklenmemişti; `Parameter` nesnesi string olarak IP'ye geçiyordu.

**Çözüm:**
- Subscription callback'i (`on_command`) içinde komut **anında** gönderiliyor.
- Status yayını `node.create_timer(0.1, ...)` ile 10 Hz'de timer'a alındı.
- `rclpy.spin(node)` main thread'de, tek seferlik çalışıyor.
- `.value` eklendi.

---

## Dosya Durumu

| Dosya | Durum |
|-------|-------|
| `vgc10_modbus_tcp/vgc10_modbus_tcp/comModbusTcp.py` | Yeniden yazıldı (pymodbus → ham socket) |
| `vgc10_control/vgc10_control/OnRobotVGTcpNode.py` | Yeniden düzenlendi |
| `vgc10_description/launch/view_robot.launch.py` | Yeni eklendi (URDF görselleştirme) |

---

## Kullanım

### Gripper bringup (gerçek donanım)
```bash
ros2 launch vgc10_control bringup.launch.py
```

### Vakum komutu gönderme
```bash
# Kanal A'yı aç (%60 vakum), Kanal B idle
ros2 topic pub --once /OnRobotVGOutput vgc10_msgs/msg/OnRobotVGOutput \
  "{rmca: 256, rvca: 60, rmcb: 512, rvcb: 0}"

# Tümünü bırak
ros2 topic pub --once /OnRobotVGOutput vgc10_msgs/msg/OnRobotVGOutput \
  "{rmca: 0, rvca: 0, rmcb: 0, rvcb: 0}"
```

### Alan açıklamaları
| Alan | Değer | Anlam |
|------|-------|-------|
| `rmca` / `rmcb` | `0x0000` (0) | Release |
| `rmca` / `rmcb` | `0x0100` (256) | Grip |
| `rmca` / `rmcb` | `0x0200` (512) | Idle |
| `rvca` / `rvcb` | 0–80 | Hedef vakum (%) — sadece Grip modunda geçerli |

### Durum izleme
```bash
ros2 topic echo /OnRobotVGInput
# gvca/gvcb: anlık vakum (1/1000 biriminde, örn. 60000 = %60)
```

### URDF görselleştirme (donanım gerekmez)
```bash
ros2 launch vgc10_description view_robot.launch.py ur_type:=ur10e
```
