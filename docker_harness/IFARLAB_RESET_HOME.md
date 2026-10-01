# IFARLAB — Test sonrası robotu home'a döndürme (harness reset)

Bu dosya, IFARLAB sunucusunda (`ifarlab` kullanıcısı, `~/colcon_ws/src`) açılacak bir Claude oturumuna verilmek üzere yazıldı. **Yalnızca IFARLAB'daki Docker harness'i değişiyor, STLC Manager (ASRLAB) tarafına dokunulmuyor.** Bu dosya tek başına yeterli: önceki güncellemenin (`IFARLAB_HARNESS_GUNCELLEME.md`: `sim_robot_goal` import'u ve pytest ile çalıştırma) değişikliklerini de içeriyor. Uygulanmamışsa ayrıca uygulamaya gerek yok.

## 0. 29 Eyl 2026 güncellemesi: canlı yığında test edildi, `reset_home.py` değişti

Bu belgenin ilk sürümüyle build edilen `0.3.0` image'ındaki `reset_home.py` **eski**. Canlı yığında (`use_fake_hardware:=true digital_twin:=true`, reset host'ta doğrudan çalıştırıldı) yapılan testler:

| Test | Sonuç |
|---|---|
| 4.1 servis/action/topic adları | hepsi doğru (cancel_goal servisleri `--include-hidden-services` ile görünüyor) |
| 4.3a home'dayken | `already_home`, 2,3 sn, sapma 0,0068 rad |
| 4.3b uzaklaştırıp reset | `homed tiers=moveit`, 7,0 sn |
| 4.3c çarpışma (cable_channel ↔ stackable_bin) | **eski kodla `failed`**: kaçış ilk geçerli noktada duruyor, robot sınırın <1° dışında kalıyor, MoveIt `-2` (INVALID_MOTION_PLAN) veriyor. **Düzeltildi:** kaçış artık aynı yönde 3 adım pay alıyor (`ESCAPE_MARGIN_STEPS`). Yeni kodla `homed tiers=moveit(99999),escape(shoulder_pan_joint+,6),moveit`, 11,8 sn |
| 4.3d temizlik | testin eklediği kutu siliniyor (`cleanup1`), snapshot'ta olan nesne korunuyor |
| uçtan uca (entrypoint + sonsuz döngülü `sim_robot_joint_goal.py` + SIGTERM) | script KeyboardInterrupt aldı ama kendi `finally` home'u yapamadı (rclpy context SIGINT'te kapanmış, `RCLError` ile exit 1); harness reset `homed` 12,7 sn; container exit 1 (testin kodu) |

Kontrol edilen `check_state_validity` gerçekten temas listesini dolu döndürüyor, kaçış kademesinin dayandığı varsayım doğru.

**Yapıldı:** Güncel `reset_home.py` ve toplam zaman sınırı eklenmiş `entrypoint.sh` ile `0.3.1` build edildi (aşağıda).

### 0.1 `0.3.1` build'i ve testleri (29 Eyl 2026)

**Image:** `ros2-exec-harness:0.3.1` = `sha256:820e1ddbb3b9c2cd7344b7efec75910361571c5f86692197e0a4492dc2860800`. STLC doğrudan bu etikete geçiyor. `0.2.0` ve `0.3.0` olduğu gibi duruyor, hiçbir etiketin üzerine yazılmadı (bkz. 4.5). `robot_capabilities.json` bu ID ile güncellenmeli.

`0.3.0`'dan farkı:
- `reset_home.py`: kaçış payı (`ESCAPE_MARGIN_STEPS = 3`).
- `entrypoint.sh`:
  - STLC kodunun çalışma süresine toplam sınır getirildi (`HARNESS_RUN_TIMEOUT`, 180 sn). Sınır hem düz script'leri hem pytest modunu kapsıyor, snapshot ve reset bu sürenin dışında.
  - Süre dolunca SIGINT gönderiliyor, 30 sn sonra SIGTERM, 10 sn sonra SIGKILL. Ardından reset çalışıyor ve container exit code 124 ile çıkıyor.
  - Zamanlayıcı ayrı bir süreç değil, bekleme döngüsünün kendisi. Test erken biterse arkada `sleep`/`kill` kalmıyor ve `docker stop` sinyallerini ileten `forward()` ile çakışmıyor.
  - pytest'in test başına sınırının varsayılanı 60 sn oldu.
  - `HARNESS_RESET=0` artık yalnızca snapshot ve reset'i kapatıyor. Zaman sınırı bu durumda da geçerli.

Neden toplam sınır gerekli: `pytest-timeout` yalnızca test gövdelerini kapsıyor. Dosya import edilirken (test toplanırken) girilen bir sonsuz döngüyü yakalamıyor. Düz script'lerin ise hiç sınırı yoktu: `~/stlc_runs/exec-6b208ebf…py` dosyasındaki tek satırlık `while True: pass`, container'ı süresiz asılı bırakıyordu. Bu durumda reset de hiç çalışmıyordu.

| Test | Yığın | `[harness]` satırları (container başlangıcından itibaren saniye) | exit |
|---|---|---|---|
| a) `while True: pass`, `HARNESS_RUN_TIMEOUT=10` | kapalı | 15.4 `test timed out after 10s; sending SIGINT` → 15.9 `test finished with exit code 124; resetting robot to home` → 26.1 `reset: status=stack_down … reason=no_joint_states` | 124 |
| b) SIGINT'i yutan script (`while True: try: time.sleep(1) except: pass`), limit 10 | kapalı | 15.9 SIGINT → 46.1 `test ignored SIGINT; sending SIGTERM` → 46.6 `finished with exit code 124` → 56.7 `reset: status=stack_down`. Toplam 56,7 sn. Script SIGTERM ile durdu, çünkü çıplak `except:` SIGTERM'ü yakalayamıyor | 124 |
| b2) SIGINT ve SIGTERM'ü yok sayan script (`signal.SIG_IGN`), limit 5 | kapalı | 10.9 SIGINT → 41.0 SIGTERM → 50.6 `test ignored SIGTERM; sending SIGKILL` → 51.1 `finished with exit code 124` → 61.2 `reset: status=stack_down` | 124 |
| e) 1 sn'de biten script, reset sırasında container içinde `ps` | kapalı | Yalnızca `entrypoint.sh`, `timeout 240` ve `reset_home.py` süreçleri var. Arkada `sleep`/`kill` kalmamış | 0 |
| c) Kaçış (container içinden). JTC ile `[1.0, 0.0, -1.3963, 2.0, -1.5708, 0.0, 0.0]` pozuna gidildi. `check_state_validity`: `valid=False`, temas `sim_ur10e_stackable_bin ↔ sim_ur10e_cable_channel` | açık | `reset: status=homed tiers=no_snapshot,moveit(99999),escape(shoulder_pan_joint+,6),moveit max_err_rad=0.0010 err_rail_m=0.0001 duration=11.8s`. Sonrasında `valid=True` | 0 |
| d) Uçtan uca, `exec-2afd419e…py` | açık | `… contains tests -> running with pytest` → `AttributeError: … no attribute 'set_initial_state'` (STLC kodunun hatası, `ModuleNotFoundError` yok) → `test finished with exit code 1; resetting robot to home` → `reset: status=already_home tiers=- max_err_rad=0.0010 duration=2.5s` | 1 |

Notlar:
- Yığın kapalıyken snapshot (~5 sn) ve reset (~10 sn) her koşuya ~15 sn ekliyor. Yığın açıkken robot home'daysa reset 2–3 sn sürüyor.
- Testlerin sonunda robot home'da kaldı.

### 0.2 `0.3.2`: toplam sınır 600 sn, STLC'nin SSH zaman aşımı kaldırılıyor (30 Eyl 2026)

`0.3.1`'den tek farkı: `HARNESS_RUN_TIMEOUT` varsayılanı **180 → 600 sn**. 180 sn, robotu gerçekten hareket ettiren meşru testleri kesebiliyordu (ölçüm: `sim_robot_joint_goal.py`'nin tek bir hareketi ölçek 0,05'te ~37 sn; `robot_capabilities.json`'a göre 26 hedeflik tur ~3,5 dk). STLC `docker run`'a ortam değişkeni geçmediği için geçerli değer image'daki varsayılan. `reset_home.py`, `Dockerfile`, `conftest.py` değişmedi.

Buna bağlı karar: **STLC'nin `ssh ... docker run` çağrısındaki zaman aşımı kaldırılıyor.** Koşunun süresini harness zaten sınırlıyor: snapshot (≤20 sn) + test (600 sn) + durdurma payı (30 + 10 sn) + reset (≤240 sn) ≈ en kötü durumda **~15 dakika**, sonra çağrı mutlaka döner. STLC bağlantıyı hiç erken kesmediği için bir sonraki test ancak önceki reset bittikten sonra başlar; iki container'ın aynı anda robotu komutlaması riski böylece kapanıyor. Ağ kopmasına karşı süre sınırı yerine SSH canlılık kontrolü kullanılıyor (ASRLab'daki `~/.ssh/config`, `Host ifarlab` bloğu):

```
    ServerAliveInterval 30
    ServerAliveCountMax 4
```

Bağlantı ~2 dk cevap vermezse SSH kendini kapatır; uzun ama canlı bir koşu hiç kesilmez.

`robot_capabilities.json` da güncellendi: `execution_environment.timeouts` (60 / 600 sn), `exit_codes.124`, yeni `post_run_reset` alanı, sonsuz döngü yasağının gerekçesi ve `run_command` etiketi (`0.3.2`). `0.3.2` 30 Eyl 2026'da IFARLAB'da build edildi: `sha256:abe5b70699a0f73cb4903b0a85761e9ea0e7a29c82ac88fa61f70b1ca5231c6b`; bu ID `meta.pinned_sources.image.id`'ye yazıldı.

## 1. Amaç

ASRLAB'dan gönderilen her testten sonra sim UR10e home'da değilse, robot home'a döndürülecek:

```
HOME = [ray 1.0 m, 0, -π/2, 0, -π/2, 0, 0]
eklemler: sim_ur10e_base_to_robot_mount, shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
```

## 2. Tasarım kararları ve gerekçeleri

**Reset, container'ın entrypoint'inde yapılıyor, ayrı bir host servisinde değil.** Entrypoint testi alt süreç olarak çalıştırır, test nasıl biterse bitsin (başarı, hata, timeout, `docker stop`) ardından `reset_home.py`'yi çalıştırır.
- STLC'nin `ssh ... docker run` çağrısı reset bitene kadar dönmez. Bu yüzden bir sonraki test robot hâlâ home'a giderken başlayamaz. Ayrı bir servis kursaydık reset ile yeni test aynı anda kolu komutlayabilirdi.
- Ek bir servis kurulmuyor. Reset kodu image'ın içinde, yani sabit ve tekrarlanabilir.
- **Container'ın exit code'u testin kendi sonucudur.** Reset sonucu yalnızca `[harness] reset: ...` log satırında görünür, test sonucunu asla maskelemez. ASRLab'daki metrik değerlendirmesi bu yüzden etkilenmez.

**Neden sadece doğrudan controller komutu değil.** STLC testleri robotu çarpışma halinde bırakabiliyor. Sim MoveIt'in `ompl_planning.yaml` dosyasında yalnızca `AddTimeOptimalParameterization` adaptörü var, `FixStartStateCollision` yok. Bu durumda MoveIt "start state in collision" deyip hiç plan yapmıyor. Öte yandan home'a tek bir doğrudan JTC komutu göndermek, eklem uzayında düz bir çizgi çizer. Ray 1 m'ye kayarken kolun şaseden ya da masadan geçip geçmediğini kimse kontrol etmez. Bu yüzden reset kademeli:

| Kademe | Ne yapar | Ne zaman |
|---|---|---|
| 0 Hazırlık | `/sim/joint_states` bekle; `/sim/move_action`, `/sim/execute_trajectory` ve JTC'deki **tüm** yarım hedefleri iptal et (test kesilse bile controller son yörüngeyi yürütmeye devam eder); robot durana kadar bekle | her zaman |
| 1 Temizlik | Testten önce alınan snapshot'ta olmayan planning scene nesnelerini sil, attach edilmiş olanları çöz. Yığının kendi nesnelerine dokunulmaz | her zaman |
| — | Robot zaten home'daysa (tolerans: 0.02 rad, ray 1 cm) çık: `already_home` | |
| 2 MoveIt | Çarpışma kontrollü plan ile home, hız/ivme ölçeği 0.1 | home'da değilse |
| 3 Kaçış | MoveIt başarısız olduysa ve `check_state_validity` robotu çarpışmada görüyorsa: 1°/1 cm'lik adımlarla (en fazla 10 adım) çarpışmadan çıkan kısa bir yol aranır, bulununca aynı yönde 3 adım daha pay alınır (hepsi geçerli olmak şartıyla). Önce home yönü, sonra her eklem ±. **Hiçbir ara nokta başlangıçta olmayan yeni bir temas çiftine giremez.** Bulunan kısa yol JTC'ye doğrudan ve yavaş gönderilir, ardından tekrar Kademe 2 | yalnızca çarpışmadayken |
| başarısız | Kaçış bulunamazsa zorlama yok: `status=failed`, çıkış kodu 4. İnsan bakar | |

En fazla 3 MoveIt denemesi yapılır, aralarında gerektiğinde kaçış. Uzun yol her zaman MoveIt'le gider. Kontrolsüz giden tek şey, adım adım sınanmış birkaç derecelik kaçış.

**Kademe 4 (yapılmadı, açık konu):** Simde son çare olarak Gazebo dünyasını resetlemek (robotu fiziksel yol izletmeden ışınlamak) mümkün olabilir. Ama (a) container'da `gz` araçları yok, (b) spawn pozunun home ile aynı olup olmadığı bilinmiyor. Bölüm 5'e bakın.

**Kasıtlı olarak pymoveit2 kullanılmıyor.** Kütüphane beklerken `rclpy.spin_once` ile node'u global executor'a devrediyor. Bilinen bir sessiz sağırlık sorunu var. `reset_home.py` tek thread'li ve doğrudan rclpy kullanıyor.

**Log çıktısı** (stdout, tek satır, makinece okunabilir):
```
[harness] reset: status=homed tiers=cancel1,cleanup2,moveit(-10),escape(toward_home,3),moveit max_err_rad=0.0041 err_rail_m=0.0012 duration=24.7s
```
`status` ∈ `already_home | homed | failed | stack_down | timeout`. `moveit(N)` = MoveIt hata kodu (-10 = START_STATE_IN_COLLISION). `reset_home.py`'nin çıkış kodu: 0 home'da, 3 yığın kapalı, 4 başarısız.

**Ortam değişkenleri** (hepsi isteğe bağlı, `docker run -e ...`):
`HARNESS_RESET=0` snapshot ve reset'i kapatır; **zaman sınırını kapatmaz** · `HARNESS_RUN_TIMEOUT` (600 sn; STLC kodunun toplam çalışma süresi, düz script ve pytest; snapshot/reset hariç; dolunca SIGINT → +30 sn SIGTERM → +10 sn SIGKILL, exit 124) · `HARNESS_RESET_TIMEOUT` (240 sn) · `HARNESS_HOME_TOL_RAD` (0.02) · `HARNESS_HOME_TOL_RAIL` (0.01) · `HARNESS_TEST_TIMEOUT` (pytest test başına, 60 sn) · `HARNESS_RUN_MODE=script` (pytest algılamasını kapatır).

## 3. Dosyalar (`~/colcon_ws/src/docker_harness/`)

Dört dosya: `Dockerfile` (değişiyor), `entrypoint.sh` (değişiyor), `reset_home.py` (yeni), `conftest.py` (önceki güncellemeden; yoksa yeni).

### 3.1 `reset_home.py` (yeni)

```python
#!/usr/bin/env python3
"""
Harness reset: her STLC testinden sonra sim UR10e'yi home pozisyonuna döndürür.

entrypoint.sh bu dosyayı iki kez çağırır:
  1. Testten ÖNCE:  reset_home.py snapshot --out /tmp/scene_before.json
     Planning scene'deki nesnelerin listesini kaydeder. Test sonrası yalnızca
     testin eklediği nesneleri silebilmek için gerekli.
  2. Testten SONRA: reset_home.py reset --snapshot /tmp/scene_before.json

Reset kademeleri (her biri yalnızca bir öncekinin yetmediği durumda devreye girer):
  0. Hazırlık  : yığın ayakta mı, yarım kalan hareketleri iptal et, robot dursun.
  1. Temizlik  : testin eklediği / tutturduğu (attach) nesneleri planning scene'den sil.
  2. MoveIt    : çarpışma kontrollü plan ile home'a git.
  3. Kaçış     : MoveIt "başlangıç durumu çarpışmada" diye plan yapamıyorsa,
                 küçük adımlarla (her ara nokta check_state_validity ile sınanarak)
                 çarpışmadan çıkan kısa bir yol bulunur ve controller'a DOĞRUDAN
                 gönderilir. Sonra tekrar kademe 2.
Uzun yol her zaman MoveIt ile gider. Controller'a doğrudan giden tek şey,
en fazla ~13 derece / 13 cm'lik (10 adım + 3 adım pay), adım adım sınanmış kaçış hareketidir.

Çıktı: stdout'a tek satır, makinece okunabilir sonuç:
  [harness] reset: status=homed tiers=cleanup,moveit max_err_rad=0.0040 ...
Çıkış kodu: 0 = home'da (zaten ya da götürüldü), 3 = yığın kapalı, 4 = başarısız.

Kasıtlı olarak pymoveit2 KULLANILMIYOR: kütüphane beklerken rclpy.spin_once ile
node'u global executor'a devrediyor ve bu da sessiz sağırlığa yol açabiliyor.
Burada tek thread var ve her bekleme spin_until_future_complete ile yapılıyor.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.srv import CancelGoal
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# ---------------------------------------------------------------------------
# Hücreye özgü sabitler (hil_test_whole_unified.launch.py, digital_twin:=true)
# ---------------------------------------------------------------------------

# Eklem sırası: pymoveit2_sim/robots/ur.py ile aynı. İlk eklem lineer ray (metre),
# diğerleri döner eklem (radyan).
JOINTS = [
    "sim_ur10e_base_to_robot_mount",
    "sim_ur10e_shoulder_pan_joint",
    "sim_ur10e_shoulder_lift_joint",
    "sim_ur10e_elbow_joint",
    "sim_ur10e_wrist_1_joint",
    "sim_ur10e_wrist_2_joint",
    "sim_ur10e_wrist_3_joint",
]
RAIL = 0  # JOINTS içinde rayın indeksi

# Home: ray 1 m, kol sensing_robot.py / sim_robot_joint_goal.py'deki home ile aynı.
HOME = [1.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]

GROUP = "sim_ur10e"
JOINT_STATES_TOPIC = "/sim/joint_states"
MOVE_ACTION = "/sim/move_action"
EXECUTE_ACTION = "/sim/execute_trajectory"
JTC_ACTION = "/sim/sim_scaled_joint_trajectory_controller/follow_joint_trajectory"
GET_SCENE_SRV = "/sim/get_planning_scene"
APPLY_SCENE_SRV = "/sim/apply_planning_scene"
VALIDITY_SRV = "/sim/check_state_validity"

# "Home'da mı?" toleransları. Gazebo'daki PID durağan halde bile birkaç mrad
# sapma bırakabildiği için çok sıkı tutulmadı. Ortam değişkeniyle değiştirilebilir.
TOL_RAD = float(os.environ.get("HARNESS_HOME_TOL_RAD", "0.02"))   # ~1.1 derece
TOL_RAIL = float(os.environ.get("HARNESS_HOME_TOL_RAIL", "0.01"))  # 1 cm

# MoveIt ile home'a giderken hız/ivme ölçeği (0-1). Reset hızlı değil güvenli olmalı.
MOVEIT_SCALING = 0.1

# Kaçış (kademe 3) ayarları: her adım en fazla 1 derece / 1 cm, en fazla 10 adım.
ESCAPE_STEP_RAD = math.radians(1.0)
ESCAPE_STEP_RAIL = 0.01
ESCAPE_MAX_STEPS = 10
# Çarpışmadan çıkılan ilk noktada durmak yetmiyor: robot sınırın <1° dışında kalıyor
# ve MoveIt oradan çıkan planı INVALID_MOTION_PLAN (-2) ile reddediyor (canlı yığında
# ölçüldü, 29 Eyl 2026). Aynı yönde, hepsi geçerli olmak şartıyla, bu kadar adım daha
# gidilir. 3 adımlık (3°) pay ile MoveIt home'a planlayabildi.
ESCAPE_MARGIN_STEPS = 3
# Kaçış hareketinin controller'a gönderilirken kullanılacak hızları (yavaş).
ESCAPE_VEL_RAD = 0.1   # rad/s
ESCAPE_VEL_RAIL = 0.05  # m/s


# ---------------------------------------------------------------------------
# Saf fonksiyonlar (ROS'suz; sahte verilerle test edilebilir)
# ---------------------------------------------------------------------------

def home_error(q, home=HOME):
    """(en büyük döner eklem hatası [rad], ray hatası [m]) döndürür."""
    err_rad = max(abs(q[i] - home[i]) for i in range(len(q)) if i != RAIL)
    err_rail = abs(q[RAIL] - home[RAIL])
    return err_rad, err_rail


def is_home(q, home=HOME):
    err_rad, err_rail = home_error(q, home)
    return err_rad <= TOL_RAD and err_rail <= TOL_RAIL


def escape_directions(q0, home=HOME):
    """
    Denenecek kaçış yönlerini sırayla üretir. Her yön, TEK adımlık eklem
    değişimi vektörüdür (bir adımda hiçbir eklem 1 derece / 1 cm'den fazla oynamaz).

    Sıra:
      1. Home'a doğru (en doğal kaçış; işe yararsa yol zaten kısalır).
      2. Her eklem tek başına + ve - yönde (7 eklem x 2 = 14 yön).
         Robot bir yüzeye dayanmışsa genelde tek bir eklemi geri almak yeter.
    """
    n = len(q0)
    steps = [ESCAPE_STEP_RAIL if i == RAIL else ESCAPE_STEP_RAD for i in range(n)]

    # 1) Home'a doğru: en çok değişmesi gereken eklem tam bir adım atacak şekilde ölçekle.
    diff = [home[i] - q0[i] for i in range(n)]
    ratio = max(abs(diff[i]) / steps[i] for i in range(n))
    if ratio > 1e-9:
        yield "toward_home", [d / ratio for d in diff]

    # 2) Tek eklem hareketleri.
    for i in range(n):
        for sign, tag in ((+1, "+"), (-1, "-")):
            vec = [0.0] * n
            vec[i] = sign * steps[i]
            yield f"{JOINTS[i].replace('sim_ur10e_', '')}{tag}", vec


def find_escape(q0, validity_fn, home=HOME, max_steps=ESCAPE_MAX_STEPS,
                margin_steps=ESCAPE_MARGIN_STEPS):
    """
    Çarpışmadaki q0'dan çıkan kısa bir yol arar.

    validity_fn(q) -> (gecerli_mi: bool, temas_ciftleri: set)
        MoveIt'in check_state_validity servisini saran fonksiyon. Testte sahtesi verilir.

    Kural: yol boyunca hiçbir ara nokta, q0'da OLMAYAN yeni bir temas çifti
    (ör. kol<->masa) içeremez. Yani mevcut çarpışmadan çıkarken başka bir
    şeye çarpmıyoruz. İlk geçerli (çarpışmasız) noktaya ulaşınca, aynı yönde
    margin_steps adım daha pay alırız (her biri geçerli olmalı; olmayan ilk
    adımda pay alma biter ve o ana kadarki yol kullanılır).

    Döndürür: (yön_adı, [q1, q2, ..., qk]) ya da bulunamazsa (None, None).
    """
    valid0, pairs0 = validity_fn(q0)
    if valid0:
        return "none_needed", []

    for name, vec in escape_directions(q0, home):
        path = []
        for k in range(1, max_steps + 1):
            qk = [q0[i] + k * vec[i] for i in range(len(q0))]
            valid, pairs = validity_fn(qk)
            if not pairs <= pairs0:
                break  # yeni bir şeye çarpıyor: bu yön kullanılamaz
            path.append(qk)
            if valid:
                for m in range(1, margin_steps + 1):
                    qm = [q0[i] + (k + m) * vec[i] for i in range(len(q0))]
                    if not validity_fn(qm)[0]:
                        break
                    path.append(qm)
                return name, path
    return None, None


def segment_duration(qa, qb):
    """İki nokta arası yavaş hareket süresi (sn): en yavaş eklemi belirler."""
    t = 0.2
    for i in range(len(qa)):
        vel = ESCAPE_VEL_RAIL if i == RAIL else ESCAPE_VEL_RAD
        t = max(t, abs(qb[i] - qa[i]) / vel)
    return t


# ---------------------------------------------------------------------------
# ROS tarafı
# ---------------------------------------------------------------------------

class ResetNode(Node):
    def __init__(self):
        super().__init__("harness_reset_home")
        # Son görülen eklem konumları. /sim/joint_states birden fazla yayıncıdan
        # (UR, Kawasaki...) parça parça gelebildiği için isim bazlı birleştiriyoruz.
        self._positions = {}
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_js, 10)

        self.move_client = ActionClient(self, MoveGroup, MOVE_ACTION)
        self.jtc_client = ActionClient(self, FollowJointTrajectory, JTC_ACTION)
        self.get_scene = self.create_client(GetPlanningScene, GET_SCENE_SRV)
        self.apply_scene = self.create_client(ApplyPlanningScene, APPLY_SCENE_SRV)
        self.validity = self.create_client(GetStateValidity, VALIDITY_SRV)

    # --- yardımcılar -------------------------------------------------------

    def _on_js(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._positions[name] = pos

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, end - time.monotonic()))

    def wait(self, future, timeout):
        """Future'ı bekler; zaman aşımında None döner."""
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def current(self):
        """Güncel eklem konumları (JOINTS sırasıyla) ya da eksik varsa None."""
        if all(j in self._positions for j in JOINTS):
            return [self._positions[j] for j in JOINTS]
        return None

    # --- kademe 0: hazırlık ------------------------------------------------

    def wait_joint_states(self, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.current() is not None:
                return True
            self.spin_for(0.1)
        return False

    def cancel_all(self, action_name):
        """
        Bir action sunucusundaki TÜM hedefleri iptal eder. Testi yarıda kesilmiş
        olsa bile controller, test gönderdiği son yörüngeyi yürütmeye devam eder.
        CancelGoal isteğinde goal_id ve stamp sıfır bırakılırsa "hepsini iptal et"
        anlamına gelir (action_msgs standardı).
        """
        client = self.create_client(CancelGoal, action_name + "/_action/cancel_goal")
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                return 0
            resp = self.wait(client.call_async(CancelGoal.Request()), 3.0)
            return len(resp.goals_canceling) if resp else 0
        finally:
            self.destroy_client(client)

    def wait_stationary(self, timeout=20.0, window=0.5, thresh=1e-3):
        """Robot durana kadar bekler (window saniyede hiçbir eklem thresh'ten fazla oynamaz)."""
        end = time.monotonic() + timeout
        prev = self.current()
        while time.monotonic() < end:
            self.spin_for(window)
            cur = self.current()
            if prev and cur and max(abs(a - b) for a, b in zip(prev, cur)) < thresh:
                return True
            prev = cur
        return False

    # --- kademe 1: planning scene temizliği --------------------------------

    def scene_objects(self):
        """(dünya nesne id'leri, attach edilmiş nesne [(id, link)]) ya da None."""
        if not self.get_scene.wait_for_service(timeout_sec=5.0):
            return None
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        resp = self.wait(self.get_scene.call_async(req), 5.0)
        if resp is None:
            return None
        world = [co.id for co in resp.scene.world.collision_objects]
        attached = [
            (aco.object.id, aco.link_name)
            for aco in resp.scene.robot_state.attached_collision_objects
        ]
        return world, attached

    def cleanup(self, before):
        """
        Testten önce OLMAYIP şimdi olan nesneleri siler. Yığının kendi kalıcı
        nesnelerine (snapshot'ta olanlara) dokunmaz. Silinen id listesini döndürür.
        """
        now = self.scene_objects()
        if now is None:
            return None
        world_now, attached_now = now
        before_world = set(before.get("world", []))
        before_attached = {a[0] for a in before.get("attached", [])}

        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        removed = []

        # Attach edilmiş yeni nesneler: önce koldan çöz, sonra dünyadan da sil
        # (çözülen nesne dünyaya bırakılır; orada kalırsa yine engel olur).
        for obj_id, link in attached_now:
            if obj_id in before_attached:
                continue
            aco = AttachedCollisionObject(link_name=link)
            aco.object.id = obj_id
            aco.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(aco)
            co = CollisionObject(id=obj_id, operation=CollisionObject.REMOVE)
            co.header.frame_id = "world"
            scene.world.collision_objects.append(co)
            removed.append(obj_id)

        for obj_id in world_now:
            if obj_id in before_world or obj_id in removed:
                continue
            co = CollisionObject(id=obj_id, operation=CollisionObject.REMOVE)
            co.header.frame_id = "world"
            scene.world.collision_objects.append(co)
            removed.append(obj_id)

        if removed:
            if not self.apply_scene.wait_for_service(timeout_sec=3.0):
                return None
            resp = self.wait(
                self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)), 5.0
            )
            if resp is None or not resp.success:
                return None
        return removed

    # --- kademe 2: MoveIt ---------------------------------------------------

    def moveit_home(self, timeout=90.0):
        """MoveIt ile home'a planla + yürüt. MoveIt hata kodunu döndürür (1 = SUCCESS)."""
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            return None
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = MOVEIT_SCALING
        req.max_acceleration_scaling_factor = MOVEIT_SCALING
        # is_diff=True: başlangıç = move_group'un gördüğü GÜNCEL durum
        # (attach edilmiş nesneler dahil).
        req.start_state.is_diff = True
        cons = Constraints()
        for name, pos in zip(JOINTS, HOME):
            cons.joint_constraints.append(JointConstraint(
                joint_name=name, position=pos,
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0,
            ))
        req.goal_constraints.append(cons)
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        handle = self.wait(self.move_client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            return None
        result = self.wait(handle.get_result_async(), timeout)
        if result is None:
            # Zaman aşımı: hareket hâlâ sürüyor olabilir, iptal et.
            self.wait(handle.cancel_goal_async(), 3.0)
            return None
        return result.result.error_code.val

    # --- kademe 3: kaçış ---------------------------------------------------

    def check_validity(self, q):
        """(geçerli_mi, {frozenset(gövde1, gövde2), ...}) döndürür."""
        req = GetStateValidity.Request()
        req.group_name = GROUP
        # is_diff=True: sadece UR eklemlerini değiştir; Kawasaki eklemleri ve
        # attach edilmiş nesneler move_group'un güncel durumundan alınır.
        req.robot_state = RobotState(is_diff=True)
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(q)
        resp = self.wait(self.validity.call_async(req), 3.0)
        if resp is None:
            # Servis cevap vermediyse güvenli tarafta kal: geçersiz + "her şeye temas".
            return False, {frozenset(("?", "?"))}
        pairs = {frozenset((c.contact_body_1, c.contact_body_2)) for c in resp.contacts}
        return resp.valid, pairs

    def jtc_execute(self, start, path):
        """Kısa kaçış yolunu controller'a doğrudan gönderir. Başarılıysa True."""
        if not self.jtc_client.wait_for_server(timeout_sec=5.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        t = 0.0
        prev = start
        for q in path:
            t += segment_duration(prev, q)
            sec = int(t)
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=list(q),
                time_from_start=Duration(sec=sec, nanosec=int((t - sec) * 1e9)),
            ))
            prev = q
        handle = self.wait(self.jtc_client.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return False
        result = self.wait(handle.get_result_async(), t + 10.0)
        return result is not None and result.result.error_code == 0


# ---------------------------------------------------------------------------
# Komutlar
# ---------------------------------------------------------------------------

def report(status, t0, tiers, q=None, extra=""):
    parts = [f"status={status}", f"tiers={','.join(tiers) or '-'}"]
    if q is not None:
        err_rad, err_rail = home_error(q)
        parts += [f"max_err_rad={err_rad:.4f}", f"err_rail_m={err_rail:.4f}"]
    parts.append(f"duration={time.monotonic() - t0:.1f}s")
    if extra:
        parts.append(extra)
    print("[harness] reset: " + " ".join(parts), flush=True)


def cmd_snapshot(node, out_path):
    objs = node.scene_objects()
    if objs is None:
        print("[harness] snapshot: planning scene alınamadı (yığın kapalı?)", file=sys.stderr)
        return 3
    world, attached = objs
    with open(out_path, "w") as f:
        json.dump({"world": world, "attached": attached}, f)
    return 0


def cmd_reset(node, snapshot_path):
    t0 = time.monotonic()
    tiers = []

    # --- Kademe 0 ---
    if not node.wait_joint_states():
        report("stack_down", t0, tiers, extra="reason=no_joint_states")
        return 3
    canceled = sum(node.cancel_all(a) for a in (MOVE_ACTION, EXECUTE_ACTION, JTC_ACTION))
    if canceled:
        tiers.append(f"cancel{canceled}")
    if not node.wait_stationary():
        # Durmuyorsa (ör. Gazebo'da fizik titremesi) yine de devam et; MoveIt
        # start_state toleransı (0.1) içinde kalırsa planlayabilir.
        tiers.append("not_stationary")

    # --- Kademe 1 ---
    before = None
    if snapshot_path and os.path.exists(snapshot_path):
        with open(snapshot_path) as f:
            before = json.load(f)
    if before is not None:
        removed = node.cleanup(before)
        if removed is None:
            tiers.append("cleanup_failed")
        elif removed:
            tiers.append(f"cleanup{len(removed)}")
    else:
        # Snapshot yoksa hangi nesnelerin teste ait olduğunu bilemeyiz: hiçbir şey silme.
        tiers.append("no_snapshot")

    node.spin_for(0.3)
    if is_home(node.current()):
        report("already_home", t0, tiers, node.current())
        return 0

    # --- Kademe 2 (+ gerekirse 3, sonra tekrar 2) ---
    # En fazla 3 MoveIt denemesi; aralarında gerekirse kaçış. Son deneme de
    # MoveIt olduğu için kaçıştan sonra her zaman bir plan denemesi yapılır.
    for attempt in range(3):
        tiers.append("moveit")
        code = node.moveit_home()
        node.spin_for(0.5)
        if code == MoveItErrorCodes.SUCCESS and is_home(node.current()):
            report("homed", t0, tiers, node.current())
            return 0
        tiers[-1] = f"moveit({code})"

        # Plan başarısız: robot MoveIt'e göre çarpışmada mı?
        q0 = node.current()
        valid, _ = node.check_validity(q0)
        if valid or attempt == 2:
            continue  # çarpışma yok (planlayıcı bulamadı) ya da son tur: tekrar dene / bitir

        direction, path = find_escape(q0, node.check_validity)
        if path is None:
            report("failed", t0, tiers, q0, extra="reason=no_escape_found")
            return 4
        tiers.append(f"escape({direction},{len(path)})")
        if path and not node.jtc_execute(q0, path):
            report("failed", t0, tiers, node.current(), extra="reason=escape_exec_failed")
            return 4
        node.spin_for(0.5)

    report("failed", t0, tiers, node.current(), extra="reason=moveit_failed")
    return 4


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    sub = parser.add_subparsers(dest="cmd", required=True)
    p_snap = sub.add_parser("snapshot", help="testten önce planning scene nesnelerini kaydet")
    p_snap.add_argument("--out", required=True)
    p_reset = sub.add_parser("reset", help="testten sonra robotu home'a döndür")
    p_reset.add_argument("--snapshot", default=None)
    args = parser.parse_args()

    rclpy.init()
    node = ResetNode()
    try:
        if args.cmd == "snapshot":
            return cmd_snapshot(node, args.out)
        return cmd_reset(node, args.snapshot)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
```

### 3.2 `entrypoint.sh`

```bash
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /harness_ws/install/setup.bash

RESET=/harness_ws/harness_tools/reset_home.py
SNAPSHOT=/tmp/scene_before.json

# Only `python3 <file>.py` runs are STLC test runs. Anything else (bash,
# python3 -c ..., reset_home.py itself) runs as-is with no reset around it.
if [[ ! ( "$1" =~ ^python3?$ && "$2" == *.py && -f "$2" && "$2" != "$RESET" ) ]]; then
    exec "$@"
fi

# STLC Manager always sends `python3 <file>.py`, but most of what it generates
# are pytest files without a `pytest.main()` call. Run with plain python3 they
# only define the test functions and exit 0 -- a pass with zero tests run.
# So if the file defines tests, run it with pytest instead. Plain scripts keep
# running exactly as before. HARNESS_RUN_MODE=script forces the old behaviour.
if [[ "${HARNESS_RUN_MODE:-auto}" != "script" ]] \
    && grep -qE '^(async def test_|def test_|class Test)' "$2"; then
    script="$2"
    shift 2
    echo "[harness] $script contains tests -> running with pytest" >&2
    cmd=(python3 -m pytest "$script" -p no:cacheprovider -rA
         --timeout="${HARNESS_TEST_TIMEOUT:-60}" "$@")
else
    cmd=("$@")
fi

# HARNESS_RESET=0 turns the post-test homing off (e.g. for debugging a test).
# The run time limit below still applies.
reset=1
[[ "${HARNESS_RESET:-1}" != "1" ]] && reset=0

set +e

# Record which planning-scene objects exist before the test, so the reset can
# remove only what the test added. A failure here must not block the test.
if (( reset )); then
    timeout 20 python3 "$RESET" snapshot --out "$SNAPSHOT" \
        || echo "[harness] snapshot failed; reset will not remove scene objects" >&2
fi

# The test is a child, not exec'd, so the reset below always runs after it.
# Bash as PID 1 ignores signals it has no trap for, so forward them: first one
# as SIGINT (Python -> KeyboardInterrupt, the script's own cleanup runs), then
# SIGTERM, then SIGKILL if the test keeps ignoring us.
signals=0
forward() {
    signals=$((signals + 1))
    case $signals in
        1) kill -INT "$child" 2>/dev/null ;;
        2) kill -TERM "$child" 2>/dev/null ;;
        *) kill -KILL "$child" 2>/dev/null ;;
    esac
}
trap forward INT TERM HUP

# Non-interactive bash starts background jobs with SIGINT ignored, and Python
# then never raises KeyboardInterrupt. Restore the default before exec'ing the test.
python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
    "${cmd[@]}" &
child=$!

# Overall limit on the STLC code's own run time (snapshot and reset are outside
# it). pytest-timeout only covers test bodies, not an endless loop hit while the
# file is being imported, and plain scripts have no limit at all. On expiry:
# SIGINT, then SIGTERM 30 s later, then SIGKILL 10 s after that. The timer is
# this polling loop itself, so an early exit leaves no sleep/kill process behind.
# Signals from `docker stop` are handled by forward() independently.
limit="${HARNESS_RUN_TIMEOUT:-600}"
start=$SECONDS
stage=0
while kill -0 "$child" 2>/dev/null; do
    elapsed=$((SECONDS - start))
    if (( stage == 0 && elapsed >= limit )); then
        stage=1
        echo "[harness] test timed out after ${limit}s; sending SIGINT" >&2
        kill -INT "$child" 2>/dev/null
    elif (( stage == 1 && elapsed >= limit + 30 )); then
        stage=2
        echo "[harness] test ignored SIGINT; sending SIGTERM" >&2
        kill -TERM "$child" 2>/dev/null
    elif (( stage == 2 && elapsed >= limit + 40 )); then
        stage=3
        echo "[harness] test ignored SIGTERM; sending SIGKILL" >&2
        kill -KILL "$child" 2>/dev/null
    fi
    sleep 0.5
done
wait "$child"
rc=$?
trap - INT TERM HUP

if (( stage > 0 )); then
    rc=124
fi

if (( ! reset )); then
    exit "$rc"
fi

echo "[harness] test finished with exit code $rc; resetting robot to home" >&2
timeout "${HARNESS_RESET_TIMEOUT:-240}" python3 "$RESET" reset --snapshot "$SNAPSHOT"
reset_rc=$?
if [[ $reset_rc -eq 124 ]]; then
    echo "[harness] reset: status=timeout"
fi

# The container's exit code is the TEST's result; the reset result is reported
# only in the "[harness] reset: ..." line so it never masks a test outcome.
exit "$rc"
```

### 3.3 `Dockerfile`

```dockerfile
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# python3-pytest / python3-pytest-timeout: STLC Manager sends pytest files;
# entrypoint.sh runs them with pytest (see there for why plain python3 is wrong).
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        python3-pytest \
        python3-pytest-timeout \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

WORKDIR /harness_ws/src

COPY pymoveit2_sim             ./pymoveit2_sim
COPY pymoveit2_real            ./pymoveit2_real
COPY pymoveit2_kawasaki_sim    ./pymoveit2_kawasaki_sim
COPY pymoveit2_kawasaki_real   ./pymoveit2_kawasaki_real

WORKDIR /harness_ws

RUN apt-get update && rosdep update && \
    rosdep install --from-paths src -y --ignore-src && \
    rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select \
        pymoveit2_sim \
        pymoveit2_real \
        pymoveit2_kawasaki_sim \
        pymoveit2_kawasaki_real

# STLC-generated code does `from sim_robot_goal import SimCollisionAwareRobotController`.
# That file is an example that CMake installs as a program under lib/pymoveit2_sim,
# which is not on the Python path, so the import failed with ModuleNotFoundError.
# Only the UR package's dir is added: pymoveit2_kawasaki_sim ships a different
# sim_robot_goal.py with the same class name, and two on the path would shadow each other.
# setup.bash (sourced in entrypoint.sh) prepends its own paths and keeps this one.
ENV PYTHONPATH=/harness_ws/install/pymoveit2_sim/lib/pymoveit2_sim

# Loaded by pytest because the generated file is mounted next to it in /harness_ws.
COPY docker_harness/conftest.py /harness_ws/conftest.py

# Post-test homing, run by entrypoint.sh after every test (trusted code, not STLC's).
COPY docker_harness/reset_home.py /harness_ws/harness_tools/reset_home.py

COPY docker_harness/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Required run flags (without them discovery works but no data flows):
#   --network host --ipc=host -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

### 3.4 `conftest.py`

```python
"""pytest setup for STLC-generated tests running in the harness container.

The generated tests construct SimCollisionAwareRobotController() directly, and a
rclpy Node cannot be created before rclpy.init(). Initialise once for the whole
session. pymoveit2 spins the node itself while waiting, so no executor is needed.

Tests that call rclpy.init()/rclpy.shutdown() themselves would otherwise raise
("init called twice") or tear the context down under the remaining tests, so
both become no-ops while the session context is alive.
"""

import pytest
import rclpy

_real_init = rclpy.init
_real_shutdown = rclpy.shutdown


def _init(*args, **kwargs):
    if not rclpy.ok():
        _real_init(*args, **kwargs)


def _shutdown(*args, **kwargs):
    pass


@pytest.fixture(scope="session", autouse=True)
def ros_context():
    _init()
    rclpy.init = _init
    rclpy.shutdown = _shutdown
    yield
    rclpy.init = _real_init
    rclpy.shutdown = _real_shutdown
    if rclpy.ok():
        _real_shutdown()
```

## 4. Uygulama ve doğrulama

Kalıcı yığın **açık** olmalı: `ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py digital_twin:=true use_fake_hardware:=true`. `/sim/move_group` yalnızca `digital_twin:=true` iken başlar. Testleri kısa tutun: her adım tek bir soruya cevap verir.

### 4.1 Varsayımları canlı yığında kontrol et (build'den önce, host'ta)

`reset_home.py` aşağıdaki adları varsayıyor. Kod pymoveit2_sim'deki adlardan türetildi ama canlı yığında hiç denenmedi:

```bash
ros2 service list | grep -E "/sim/(check_state_validity|get_planning_scene|apply_planning_scene)$"
ros2 action list  | grep -E "/sim/(move_action|execute_trajectory)$|/sim/sim_scaled_joint_trajectory_controller/follow_joint_trajectory"
ros2 topic echo --once /sim/joint_states --field name     # sim_ur10e_* 7 eklem görünmeli
```

Eksik ya da farklı bir ad varsa `reset_home.py`'nin başındaki sabitleri düzelt.

### 4.2 Build (yeni etiket, eski image'a dokunmadan)

```bash
cd ~/colcon_ws/src
docker build -t ros2-exec-harness:0.3.2 -f docker_harness/Dockerfile .
docker images --no-trunc ros2-exec-harness:0.3.2   # tam ID -> robot_capabilities.json meta.pinned_sources.image.id
```

Aşağıda `RUN` kısaltması kullanılıyor:

```bash
RUN="docker run --rm --network host --ipc=host -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4"
```

### 4.3 Reset'i tek başına sına

```bash
# a) Zaten home'dayken: status=already_home (ya da küçük bir hareketle homed) beklenir
$RUN ros2-exec-harness:0.3.2 python3 /harness_ws/harness_tools/reset_home.py reset

# b) Robotu home'dan uzaklaştır (host'ta), sonra reset: status=homed tiers=moveit
ros2 action send_goal /sim/sim_scaled_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [sim_ur10e_base_to_robot_mount, sim_ur10e_shoulder_pan_joint, sim_ur10e_shoulder_lift_joint, sim_ur10e_elbow_joint, sim_ur10e_wrist_1_joint, sim_ur10e_wrist_2_joint, sim_ur10e_wrist_3_joint], points: [{positions: [0.6, 0.5, -1.3, 0.3, -1.57, 0.2, 0.0], time_from_start: {sec: 8}}]}}"
$RUN ros2-exec-harness:0.3.2 python3 /harness_ws/harness_tools/reset_home.py reset
```

**c) Kaçış (Kademe 3):** Robotu MoveIt'e göre çarpışmada olan bir poza götür. Bunun için (b)'deki komutla kolu masaya ya da şaseye doğru indiren bir hedef gönder. Önce pozun gerçekten geçersiz olduğunu ve **temas listesinin dolu geldiğini** kontrol et:

```bash
ros2 service call /sim/check_state_validity moveit_msgs/srv/GetStateValidity \
  "{group_name: sim_ur10e, robot_state: {is_diff: true}}"
# valid: false ve contacts: [...] içinde contact_body_1/2 dolu olmalı
```

`contacts` boş gelirse Kademe 3 çalışmaz, çünkü "yeni temas yok" kuralı temas listesine dayanıyor. Bu durumu bildir. Sonra reset'i çalıştır. Beklenen: `tiers=...moveit(-10),escape(<yön>,<adım>),moveit`, `status=homed`.

**d) Temizlik (Kademe 1):** Kapsayıcı bir test gibi davranıp snapshot → kutu ekleme → reset akışını dene:

```bash
$RUN -v /tmp:/tmp ros2-exec-harness:0.3.2 python3 /harness_ws/harness_tools/reset_home.py snapshot --out /tmp/snap.json
ros2 topic pub --once /sim/collision_object moveit_msgs/msg/CollisionObject \
  "{id: harness_test_box, header: {frame_id: world}, operation: 0, primitives: [{type: 1, dimensions: [0.1, 0.1, 0.1]}], primitive_poses: [{position: {x: 3.0, y: 3.0, z: 0.05}, orientation: {w: 1.0}}]}"
$RUN -v /tmp:/tmp ros2-exec-harness:0.3.2 python3 /harness_ws/harness_tools/reset_home.py reset --snapshot /tmp/snap.json
# Beklenen: tiers=cleanup1,... ve kutu RViz'de /sim planning scene'den kaybolmuş olmalı
```

(Kutu robottan uzağa, (3, 3) koordinatına konuyor; yalnızca silinip silinmediği sınanıyor.)

### 4.4 Uçtan uca: gerçek bir STLC dosyasıyla

```bash
$RUN -v ~/stlc_runs/exec-2afd419e9af64ff594589273ca3afd40.py:/harness_ws/generated_script.py:ro \
  ros2-exec-harness:0.3.2 python3 /harness_ws/generated_script.py; echo "exit=$?"
```

Beklenen sırayla:
1. `[harness] ... running with pytest`
2. `ModuleNotFoundError` yok. Test STLC'nin ürettiği kodun kendi hatasıyla başarısız olur (0.3.1 ile ölçülen: `AttributeError: … no attribute 'set_initial_state'`; sınıfta olmayan bir metot).
3. `[harness] test finished with exit code 1; resetting robot to home`
4. `[harness] reset: status=...`
5. `exit=1`: testin kodu, reset'inki değil.

Bir de sonsuz döngülü `~/stlc_runs/generated_script.py`'yi başlatıp birkaç saniye sonra **başka bir terminalden** `docker stop -t 120 <container>` ile durdur. Test `KeyboardInterrupt` ile kendi temizliğini yapmalı, ardından reset çalışmalı. `-t 120` önemli: varsayılan 10 sn'lik süre sonunda Docker her şeyi SIGKILL ile öldürür ve reset yarıda kalır.

### 4.5 STLC'nin kullandığı etikete geçir

**Etiketin üzerine yazılmıyor. STLC doğrudan `0.3.2`'ye geçiyor.**

Bu belgenin ilk sürümü `0.3.0`'ı `0.2.0` etiketinin üzerine yazmayı öneriyordu. Bu yapılmadı. Her etiket tek bir image'ı göstermeye devam ediyor. Böylece hangi koşunun hangi image ile yapıldığı etiketten okunabiliyor ve `robot_capabilities.json`'daki sabitlenmiş image ID'si anlamını koruyor.

| Etiket | Image ID | İçerik |
|---|---|---|
| `0.2.0` | `sha256:4a814e161eb6…` | import + pytest düzeltmesi, reset yok |
| `0.3.0` | `sha256:c13b4440447d…` | reset (eski `reset_home.py`, kaçış payı yok). **Kullanılmamalı** |
| `0.3.1` | `sha256:820e1ddbb3b9c2cd7344b7efec75910361571c5f86692197e0a4492dc2860800` | reset + kaçış payı + `HARNESS_RUN_TIMEOUT` (180 sn) |
| `0.3.2` | `sha256:abe5b70699a0f73cb4903b0a85761e9ea0e7a29c82ac88fa61f70b1ca5231c6b` | `0.3.1` + `HARNESS_RUN_TIMEOUT` varsayılanı 600 sn |

STLC'nin göndereceği komut:

```bash
ssh ifarlab "docker run --rm --network host --ipc=host \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v ~/stlc_runs/generated_script.py:/harness_ws/generated_script.py:ro \
  ros2-exec-harness:0.3.2 python3 /harness_ws/generated_script.py"
```

Geri alma: STLC komutundaki etiketi `0.2.0` yapmak (reset'siz eski davranış).

STLC tarafında ayrıca:
- `robot_capabilities.json`: bu klasördeki güncel sürüm kullanılmalı (timeouts 60 / 600 sn, exit 124, `post_run_reset`, `run_command` `0.3.2`). Build'den sonra `meta.pinned_sources.image.id` tam ID ile doldurulmalı.
- SSH çağrısındaki zaman aşımı **kaldırılmalı**; ASRLab `~/.ssh/config` → `Host ifarlab` bloğuna `ServerAliveInterval 30` ve `ServerAliveCountMax 4` eklenmeli (bkz. 0.2).

## 5. Açık noktalar

- **Süre:** Her test artık reset kadar uzun sürüyor. Robot zaten home'daysa birkaç saniye (DDS keşfi + durma kontrolü), MoveIt ile dönüşte hızın 0.1 olması nedeniyle onlarca saniye. STLC'nin SSH çağrısında zaman aşımı yok (0.2); en kötü durumda bir koşu ~15 dk sürer. Gerekirse `MOVEIT_SCALING` artırılabilir.
- **Home toleransı:** Gazebo'daki durağan hata 0.02 rad / 1 cm'den büyükse robot hep "home'da değil" görünür ve her testten sonra gereksiz bir MoveIt hareketi yapılır. 4.3a'daki `max_err_rad` değerine bakıp gerekirse `HARNESS_HOME_TOL_*` ile ayarla.
- **Kademe 4 (Gazebo reset):** Yapılmadı. Kademe 3'ün bulamadığı durumlar sık görülürse incelenmeli: robotun spawn pozu (ros2_control `initial_positions`) home ile aynı mı, `/world/cem/control` servisi container'dan (gz araçları olmadan) nasıl çağrılabilir?
- **Kawasaki:** Reset yalnızca UR'yi (`sim_ur10e` grubu) home'a götürüyor. JTC iptali de yalnızca UR controller'ına yapılıyor. Kawasaki controller'ına dokunulmuyor.
- **`docker kill` / SIGKILL:** Bu durumda reset çalışmaz. Gerekirse host'ta `docker events --filter event=die` dinleyen küçük bir yedek servis eklenebilir.
- **Gazebo'da fiziksel takılma:** Robot Gazebo'da bir yüzeye fiziksel olarak dayanıyor ama MoveIt onu çarpışmada görmüyorsa (MoveIt ve Gazebo çarpışma modelleri farklıysa), Kademe 3 devreye girmez ve MoveIt planı fiziksel olarak takılabilir. Böyle bir durum görülürse log satırı ve bir ekran görüntüsüyle bildir.
