# HIL Sistemi Operasyon Raporu

**Sistem:** UR10e + Festo Lineer Eksen (gerçek) | Kawasaki RS005L + AGV OTA (gerçek) | Gazebo Ignition (simülasyon)

---

## Ağ Bağlantı Tablosu

| Donanım | IP | Port | Protokol |
|---|---|---|---|
| UR10e | 192.168.3.5 | — | ur_robot_driver |
| Kawasaki RS005L | 192.168.3.7 | 11111 | SIR TCP (MPT_JOINT) |
| Festo Lineer Eksen | 192.168.3.1 | 502 | Modbus TCP |
| AGV ROS1 PC | 192.168.3.6 | 9090 | Rosbridge WebSocket |

---

## SENARYO 1: İlk Çalıştırma (Kayıt Verisi Yok)

Bu senaryoda `real_ur_trajectories.json` ve `real_kawa_trajectories.json` dosyaları henüz oluşturulmamıştır.
Terminal 3 başlatılmaz; sistem yalnızca izleme ve doğrulama amacıyla çalıştırılır.

### Ön Koşullar

- UR10e Teaching Pendant'ta **REMOTE** moda alınmış olmalı
- Kawasaki kontrol panelinde `pcexecute 3:autostart3.pc` çalışıyor olmalı (KrTerm ile doğrula)
- AGV ROS1 PC açık ve rosbridge çalışıyor olmalı
- Festo sürücü enerjili ve iletişim hazır olmalı

### Adım 1 — Terminal 1: Tüm Altyapıyı Başlat

```bash
cd /home/cem/colcon_ws
source install/setup.bash
ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py
```

**Beklenen başlangıç sırası:**

| Süre | Olay |
|---|---|
| 0s | Gazebo başlar, gerçek UR driver + RSP başlar, RViz açılır |
| 5s | `sim_joint_state_broadcaster` + `kawasaki_joint_state_broadcaster` aktif |
| 7s | `sim_scaled_joint_trajectory_controller` + `kawasaki_controller` + `ota_base_controller` aktif |
| 10s | Gazebo'ya model spawn edilir, sim RSP + bridge node'ları başlar |
| 25s | Sim MoveGroup (`/sim` namespace) başlar |

**Doğrulama:**
```bash
# Controller'ların aktif olduğunu kontrol et
ros2 control list_controllers --controller-manager /sim/controller_manager
# Beklenen: sim_joint_state_broadcaster, kawasaki_controller, ota_base_controller — active

# UR10e gerçek controller
ros2 control list_controllers
# Beklenen: scaled_joint_trajectory_controller — active
```

### Adım 2 — Terminal 2: AGV + MoveGroup + Sim Bridge

```bash
cd /home/cem/colcon_ws
source install/setup.bash
ros2 launch pymoveit2_real hil_control.launch.py
```

**Beklenen başlangıç sırası:**

| Süre | Olay |
|---|---|
| 0s | `agv_bridge_node` + `agv_controller_node` + `real_to_sim_bridge` başlar |
| 8s | `real_robot_moveit_config` MoveGroup başlar |

**Doğrulama:**
```bash
# AGV bağlantısı
ros2 topic echo /agv/odom --once

# Gerçek UR joint state geliyorsa sim'e yansıyor mu?
ros2 topic echo /sim/sim_scaled_joint_trajectory_controller/joint_trajectory --once
```

### Adım 3 — Terminal 3: KawasakiArmNode'u Başlat (sadece izleme)

Bu adımda `real_dual_executor.py` JSON dosyası olmadığı için hata verir ve çıkar.
Sadece `KawasakiArmNode`'un robot'a bağlandığını doğrulamak için başlatılabilir:

```bash
cd /home/cem/colcon_ws
source install/setup.bash
ros2 run sir_robot_ros_interface KawasakiArmNode
```

**Beklenen log:**
```
[Monitor] Robot bağlantısı kuruldu (192.168.3.7:11111)
```

**RViz'de kontrol:**
- Real taraf: UR10e + Kawasaki + AGV görünüyor mu?
- Kawasaki eklem açıları gerçek robotla eşleşiyor mu?
- Gazebo'da Kawasaki pozisyonu güncelleniyor mu?

---

## SENARYO 2: Kayıt Sisteminden Veri Geldi

Bu senaryoda simülasyonda hareket kaydedilmiş, JSON dosyaları hazır.
Tam sistem çalıştırılarak gerçek robotlar bu verileri yürütür.

### 2.1 — Trajectory Dosyalarını Hazırla

Kayıt sistemi şu dosyaları üretir (çalışma dizininde):
- `sim_ur_trajectories.json` — UR10e sim eklem açıları (`sim_ur10e_*` isimleri)
- `sim_kawa_trajectories.json` — Kawasaki + AGV noktaları (`sim_kawasaki_*` isimleri, `world_to_agv`)

**Dönüşüm scripti ile gerçek robot formatına çevir:**

```bash
cd /home/cem/colcon_ws/src
python3 guncelle_dual.py
```

Bu script:
- `sim_ur_trajectories.json` → `real_ur_trajectories.json` (`sim_` ön eki kaldırılır)
- `sim_kawa_trajectories.json` → `real_kawa_trajectories.json` (`sim_kawasaki_` → `joint`, `world_to_agv` korunur)

**JSON formatı doğrulaması:**

```bash
# UR waypoint sayısı
python3 -c "import json; d=json.load(open('real_ur_trajectories.json')); print(len(d), 'UR waypoint')"

# Kawasaki waypoint sayısı + ilk noktanın AGV hedefi
python3 -c "
import json, math
d = json.load(open('real_kawa_trajectories.json'))
print(len(d), 'Kawa waypoint')
for k in sorted(d):
    p = d[k]['points'][-1]['positions']
    print(f'  {k}: AGV={p[0]:.2f}m, J1={math.degrees(p[1]):.1f}°')
"
```

> **ÖNEMLİ:** `real_dual_executor.py` bu dosyaları **çalıştırıldığı dizinde** arar.
> Terminal 3 açılmadan önce dosyaların oraya kopyalandığından emin ol:
> ```bash
> cp real_ur_trajectories.json real_kawa_trajectories.json \
>    /home/cem/colcon_ws/src/pymoveit2_real/examples/
> ```
> veya Terminal 3'ü bu dizinden çalıştır.

### 2.2 — Terminal 1 ve Terminal 2'yi Başlat

Senaryo 1'deki Adım 1 ve Adım 2 aynen uygulanır.
Terminal 2'deki `real_to_sim_bridge` aktif olmalıdır (gerçek UR hareketi sim'e yansısın).

### 2.3 — Terminal 3: Tam Yürütmeyi Başlat

**Terminal 2'deki MoveGroup başladıktan sonra** (yaklaşık 8-10 saniye) başlat:

```bash
cd /home/cem/colcon_ws/src/pymoveit2_real/examples
source /home/cem/colcon_ws/install/setup.bash
ros2 launch pymoveit2_real real_dual_execution.launch.py
```

**İçerdiği node'lar:**

| Node | Görev |
|---|---|
| `real_dual_executor.py` | UR10e → MoveIt2 action, Kawasaki+AGV → topic tabanlı |
| `KawasakiArmNode` | Kawasaki TCP bağlantısı (tek bağlantı), sim sync (10 Hz) |

**Başarılı başlatma logları:**
```
[real_dual_executor]: Loaded N UR + N Kawa waypoints.
[kawasaki_arm_executor]: [Monitor] Robot bağlantısı kuruldu (192.168.3.7:11111)
[real_dual_executor]: [UR] === Loop 1 ===
[real_dual_executor]: [Kawa] === Loop 1 ===
```

**Yürütme döngüsü (sonsuz, Ctrl+C ile durdurulur):**

```
Her döngü adımı:
  ├─ AGV hedef pozisyonu → /agv/goal_position
  ├─ Kawasaki eklem açıları → /kawa/target_joints → KawasakiArmNode
  │    └─ monitor thread: robot.add() + robot.move() (AYNI TCP bağlantısı)
  ├─ UR10e → MoveIt2 → scaled_joint_trajectory_controller
  └─ Adım tamamlanma beklenir (AGV ±0.1m tolerans + kawa "Done")
```

---

## Sık Karşılaşılan Sorunlar

### ❌ `Invalid packet type` hatası
**Neden:** İki eş zamanlı TCP bağlantısı açılmaya çalışıldı.
**Çözüm:** `KawasakiArmNode` tek bağlantı mimarisini kullanıyor (düzeltildi). Başka bir node'un 192.168.3.7:11111'e bağlanmadığından emin ol (`ROS2KawasakiRobotMove` vb. çalışmamalı).

### ❌ Kawasaki Gazebo'da hareket etmiyor
**Kontrol:**
```bash
ros2 topic hz /sim/kawasaki_controller/joint_trajectory
# Beklenen: ~10 Hz
ros2 control list_controllers --controller-manager /sim/controller_manager | grep kawasaki
# Beklenen: kawasaki_controller — active
```

### ❌ `JSON files not found`
**Neden:** `real_dual_executor.py` çalışma dizininde JSON yok.
**Çözüm:** Yukarıdaki `cp` komutunu uygula veya dosyaların bulunduğu dizinden başlat.

### ❌ AGV hareket etmiyor
**Kontrol:**
```bash
ros2 topic echo /agv/mobile_status
ros2 topic echo /agv/odom --once
```
AGV ROS1 PC'nin açık ve rosbridge'in çalıştığından emin ol (192.168.3.6:9090).

### ❌ RViz'de Kawasaki origin'de
**Neden:** `world_to_agv` prismatik joint verisi gelmiyor.
**Kontrol:**
```bash
ros2 topic echo /joint_states | grep world_to_agv
```
`agv_bridge_node`'un çalışıp `/joint_states`'e veri yayınladığından emin ol.

---

## Sistem Durdurma

```
1. Terminal 3: Ctrl+C  → KawasakiArmNode güvenli kapanır (monitor thread join)
2. Terminal 2: Ctrl+C
3. Terminal 1: Ctrl+C  → Gazebo + tüm driver'lar kapanır
```

> Kawasaki robotun güvenli durması için UR ve Kawasaki hareketleri bir adımın sonunda durdurulur (döngü arasında Ctrl+C kullan).
