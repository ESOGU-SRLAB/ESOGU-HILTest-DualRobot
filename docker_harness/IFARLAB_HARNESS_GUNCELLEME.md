# IFARLAB — `ros2-exec-harness` Docker image güncellemesi

**Durum (2026-09-25): uygulandı ve doğrulandı.** Yeni image `ros2-exec-harness:0.2.0` olarak build alındı. `~/stlc_runs` altındaki 87 dosyanın tamamı bu image ile koşturuldu ve **hiçbirinde ortam kaynaklı hata çıkmadı** (bkz. Bölüm 4). STLC Manager (ASRLAB) artık `0.2.0` etiketini kullanacak. `0.1.0` olduğu gibi duruyor ve geri dönüş için hazır.

Değişiklik yalnızca IFARLAB'daki Docker harness'inde yapıldı. STLC tarafında değişen tek şey komuttaki image etiketi.

## 1. Sorun

STLC Manager'ın ürettiği ve container'da çalıştırılan kod şu hatayla çöküyordu:

```
File "/harness_ws/generated_script.py", line 2, in <module>
    from sim_robot_goal import SimCollisionAwareRobotController  # Assuming this is where your class is defined
ModuleNotFoundError: No module named 'sim_robot_goal'
```

### Kök sebepler

1. **`sim_robot_goal` import edilebilir bir modül değildi.** `SimCollisionAwareRobotController` sınıfı `pymoveit2_sim/examples/sim_robot_goal.py` içinde tanımlı. `pymoveit2_sim/CMakeLists.txt` bu dosyayı `install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})` ile *program* olarak kuruyor. Bu yüzden dosya image'da `/harness_ws/install/pymoveit2_sim/lib/pymoveit2_sim/sim_robot_goal.py` konumunda duruyordu ama bu dizin `PYTHONPATH`'te değildi. Host'ta çalışmasının sebebi, script'in `examples/` dizininden başlatılmasıydı: Python, script'in bulunduğu dizini otomatik olarak `sys.path`'e ekler.

2. **STLC'nin ürettiği dosyalar pytest dosyası, ama `python3 dosya.py` ile çalıştırılıyorlardı.** 85 `exec-*.py` dosyasının 82'si `test_*` fonksiyonları içeriyor. `if __name__ == "__main__": pytest.main(...)` olan sadece 5 tanesi var. Import düzeltilip çalıştırma şekli değiştirilmeseydi bu dosyalar fonksiyonları tanımlayıp **hiçbir test koşmadan exit code 0 ile** çıkacak, STLC de bunu "geçti" diye okuyacaktı.

3. **`rclpy.init()` çağrılmıyordu.** Testler `SimCollisionAwareRobotController()` nesnesini doğrudan oluşturuyor. `rclpy.init()` çağrılmadan bir `Node` oluşturulamaz.

Not: pytest (6.2.5) eski image'da zaten vardı. Eksik olan `pytest-timeout` eklentisiydi.

## 2. Yapılan değişiklikler (`~/colcon_ws/src/docker_harness/`)

| Dosya | Değişiklik |
|---|---|
| `Dockerfile` | `python3-pytest`, `python3-pytest-timeout` eklendi. `ENV PYTHONPATH=/harness_ws/install/pymoveit2_sim/lib/pymoveit2_sim` eklendi. `conftest.py` `/harness_ws/` altına kopyalanıyor. |
| `entrypoint.sh` | İçinde test olan dosyalar otomatik olarak pytest ile çalıştırılıyor. Diğer dosyalar eskisi gibi `python3` ile çalışıyor. |
| `conftest.py` (yeni) | Test oturumu başında `rclpy.init()` çağrılıyor. Testlerin kendi `init`/`shutdown` çağrıları zararsız hale getiriliyor. |

Eski `Dockerfile` ve `entrypoint.sh` yedeklendi. Geri dönmek için `0.1.0` etiketini kullanmak yeterli.

### 2.1 `Dockerfile`

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

COPY docker_harness/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Required run flags (without them discovery works but no data flows):
#   --network host --ipc=host -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

### 2.2 `entrypoint.sh`

```bash
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /harness_ws/install/setup.bash

# STLC Manager always sends `python3 <file>.py`, but most of what it generates
# are pytest files without a `pytest.main()` call. Run with plain python3 they
# only define the test functions and exit 0 -- a pass with zero tests run.
# So if the file defines tests, run it with pytest instead. Plain scripts keep
# running exactly as before. HARNESS_RUN_MODE=script forces the old behaviour.
if [[ "${HARNESS_RUN_MODE:-auto}" != "script" && "$1" =~ ^python3?$ && "$2" == *.py && -f "$2" ]] \
    && grep -qE '^(async def test_|def test_|class Test)' "$2"; then
    script="$2"
    shift 2
    echo "[harness] $script contains tests -> running with pytest" >&2
    exec python3 -m pytest "$script" \
        -p no:cacheprovider \
        -rA \
        --timeout="${HARNESS_TEST_TIMEOUT:-300}" \
        "$@"
fi

exec "$@"
```

Davranış:

- `python3 dosya.py` komutu geldiğinde, dosyada sütun 0'da `def test_`, `async def test_` ya da `class Test` varsa dosya **pytest** ile çalışır. Yoksa eskisi gibi `python3` ile çalışır.
- pytest'in exit code'u doğrudan dışarı döner: 0 = hepsi geçti, 1 = başarısız test var, 2 = toplama (import) hatası, 5 = test yok.
- `-p no:cacheprovider`: `.pytest_cache` yazılmaz, çünkü script salt-okunur bağlanıyor.
- `--timeout`: test başına süre sınırı. Varsayılan 300 sn, `-e HARNESS_TEST_TIMEOUT=...` ile değiştirilebilir. **Bu sınır yalnızca pytest ile koşan dosyalar için geçerli** (bkz. Bölüm 5.1).
- `-e HARNESS_RUN_MODE=script`: acil durumda eski davranışa döner.

### 2.3 `conftest.py`

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

Neden executor yok: `pymoveit2_sim/moveit2.py`, beklediği her yerde `rclpy.spin_once(self._node, ...)` ile node'u kendisi döndürüyor. Ayrı bir executor thread'i aynı node'u iki yerden spin eder ve sorun çıkarır.

## 3. Uygulama (yapıldı)

```bash
cd ~/colcon_ws/src
docker build -t ros2-exec-harness:0.2.0 -f docker_harness/Dockerfile .
```

| Image | ID | Oluşturulma |
|---|---|---|
| `ros2-exec-harness:0.2.0` (yeni) | `sha256:4a814e161eb6…` | 2026-09-25 15:28 (+03) |
| `ros2-exec-harness:0.1.0` (eski, dokunulmadı) | `sha256:92e982e481e4…` | — |

STLC `0.2.0` etiketini kullanacağı için eski plandaki "`0.1.0` etiketini yeni image'a yönlendir" adımı **uygulanmadı**. Böylece aynı etiket iki farklı image'ı göstermiyor ve koşuların tekrarlanabilirliği korunuyor.

STLC'nin artık göndermesi gereken komut:

```bash
ssh ifarlab "docker run --rm --network host --ipc=host \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v ~/stlc_runs/generated_script.py:/harness_ws/generated_script.py:ro \
  ros2-exec-harness:0.2.0 python3 /harness_ws/generated_script.py"
```

Geri dönüş: komuttaki etiketi `0.1.0` yapmak yeterli.

## 4. Test sonuçları (2026-09-25)

### 4.1 Hızlı kontroller

| Kontrol | Sonuç |
|---|---|
| `from sim_robot_goal import SimCollisionAwareRobotController` | ✅ `OK <class 'sim_robot_goal.SimCollisionAwareRobotController'>` |
| `python3 -m pytest --version` | ✅ pytest 6.2.5 |
| `--timeout` seçeneği | ✅ var (pytest-timeout kurulu) |
| `/harness_ws/conftest.py` | ✅ image'da mevcut |

### 4.2 `~/stlc_runs` altındaki tüm dosyalar

87 dosyanın hepsi STLC'nin komutunun aynısıyla, yalnızca etiket `0.2.0` yapılarak koşturuldu (`-e HARNESS_TEST_TIMEOUT=60`). Koşu sırasında kalıcı yığın `hil_test_whole_unified.launch.py use_fake_hardware:=true digital_twin:=true` ile açıldı. Gerçek robota komut gitmedi.

**Ortam kaynaklı hata: 0.** Hiçbir dosyada `No module named 'sim_robot_goal'`, eksik pytest/eklenti ya da `rclpy` init hatası görülmedi.

| Sonuç | Adet | Açıklama |
|---|---|---|
| exit 0 | 4 | 1 dosya gerçek pytest testiyle geçti (`exec-eea2ca05…`, `1 passed`). 3'ü test içermeyen düz script (hesap makinesi örneği, `print`). |
| exit 1 | 32 | Test çalıştı ama başarısız oldu: sınıfta olmayan metotlar, yanlış assert'ler, 5 dosyada `fixture '...' not found`. |
| exit 2 | 49 | Toplama hatası: olmayan isimlerin import'u, 2 sözdizimi hatası, image'da kurulu olmayan pytest eklentisi argümanları. |
| askıda kaldı | 2 | `exec-6b208ebf…` (`while True: pass`) ve `generated_script.py` (sonsuz döngü). Bkz. Bölüm 5.1. |

exit 1 ve exit 2'deki hataların **tamamı STLC'nin ürettiği koddan** kaynaklanıyor. Harness artık bu hataları gizlemiyor, doğru şekilde gösteriyor. LLM testleri sınıfın kaynak kodunu görmeden yazmış. `sim_robot_goal`'dan import edilmeye çalışılan ama var olmayan isimler:

| İsim | Görülme |
|---|---|
| `RobotGoal` | 26 dosya |
| `robot_goal` | 5 |
| `input_validation` | 8 |
| `simulate_robot_goal`, `SimRobotGoal`, `login_system` | 2'şer dosya |

Sınıfta gerçekten var olan metotlar şunlar: `move_to_position`, `safe_move_sequence`, `move_home_safe`, `safe_cartesian_path`, `check_planning_scene`.

Sınırda sayılabilecek durum: `exec-30b558…`, `exec-98082d14…` ve `exec-abfe1aff…` dosyaları kendi içlerinde `pytest.main([... '--html=report.html'])` (bir tanesi `--self-contained-no-warnings`) çağırıyor. Bunun için `pytest-html` eklentisi gerekiyor ve bu eklenti image'da yok. Argümanlar STLC'nin kodundan geliyor. İstenirse eklenti image'a eklenebilir.

**Referans dosya:** Hatanın ilk görüldüğü `exec-2afd419e9af64ff594589273ca3afd40.py` artık şu sonucu veriyor:

```
[harness] /harness_ws/generated_script.py contains tests -> running with pytest
E   AttributeError: 'SimCollisionAwareRobotController' object has no attribute 'set_initial_state'
1 failed  (exit=1)
```

Import sorunu çözüldü. Kalan hata, testin sınıfta olmayan `set_initial_state` metodunu çağırmasından kaynaklanıyor. (Bu belgenin önceki sürümü bu dosya için `tc_006_setup` / `process_input` hatası bekliyordu. O beklenti yanlıştı, bu dosyada o isimler geçmiyor.)

### 4.3 Robot hareketi: `generated_script.py`

87 dosyadan gerçek hareket metodu çağıran **tek dosya** bu. `exec-*` dosyalarının hiçbiri robotu hareket ettirmiyor.

- Container içinden MoveIt'e bağlandı. Planning scene aktif.
- **26 hedefin 26'sı "Hareket başarılı!"** ile tamamlandı. Bir tur yaklaşık 3,5 dakika sürdü.
- Script'in `main()` fonksiyonu `while rclpy.ok():` döngüsü kullanıyor. Tur bitince 3 sn bekleyip baştan başlıyor, yani **kendiliğinden hiç bitmiyor** (bkz. 5.1).
- Dışarıdan durdurulduğunda `finally` bloğundaki ikinci `rclpy.shutdown()` çağrısı `RCLError: rcl_shutdown already called` hatası veriyor. Bu script'in kendi hatası, harness'le ilgisi yok.

## 5. Açık noktalar

### 5.1 `while True: pass`: düz script'lerde zaman sınırı yok (ÇÖZÜLMEDİ)

`~/stlc_runs/exec-6b208ebfcbca4a3faf06f235da4e6e06.py` dosyasının tek satırı şu:

```python
while True: pass
```

İçinde test olmadığı için entrypoint bu dosyayı pytest'e değil düz `python3`'e gönderiyor. **pytest'in `--timeout`'u düz script'lere uygulanmıyor**, bu yüzden container sonsuza kadar çalıştı. STLC bu dosyayı göndermiş olsaydı SSH komutu hiç dönmezdi. Aynı durum `generated_script.py`'nin sonsuz döngüsü için de geçerli.

Dışarıdan durdurmak da işe yaramıyor. `entrypoint.sh` `exec` ile Python'u başlattığı için Python container içinde **PID 1** oluyor. Linux, PID 1'e gelen ve handler'ı olmayan SIGTERM/SIGINT sinyallerini yok sayar. Test sırasında `timeout 150 docker run ...` kullanıldı. Süre dolunca docker istemcisi sinyali container'a iletti ama Python durmadı. Container ancak `docker rm -f` ile kapatılabildi. (`generated_script.py` durdu, çünkü rclpy kendi sinyal handler'ını kuruyor. `while True: pass` durmadı.)

**Önerilen çözüm (henüz uygulanmadı):** `entrypoint.sh`'teki son satırı düz script'ler için bir süre sınırıyla sarmak:

```bash
exec timeout --kill-after=10 "${HARNESS_SCRIPT_TIMEOUT:-600}" "$@"
```

- Bu durumda PID 1 `timeout` olur ve Python normal bir alt süreç olarak çalışır. Sinyaller düzgün iletilir, süre dolunca önce SIGTERM, 10 sn sonra SIGKILL gönderilir.
- Süre dolarsa exit code **124** olur. STLC bunu ayrı bir durum olarak ("zaman aşımı") yorumlamalı.
- Varsayılan süre dikkatle seçilmeli: `generated_script.py`'nin tek turu ~3,5 dk sürüyor, 300 sn sınıra yakın. 600 sn makul bir başlangıç. STLC gerekirse `-e HARNESS_SCRIPT_TIMEOUT=...` ile değiştirebilir.
- `generated_script.py` gibi bilerek sonsuz döngüyle yazılmış script'ler bu çözümle her zaman 124 ile bitecek. Bunları "başarılı" saymak için STLC'nin log'daki sonuca bakması ya da üretilen kodun döngüsüz yazılması gerekir.

Alternatif olarak STLC komutuna `--init` eklenebilir (Docker'ın tini'si PID 1 olur). Ama bu STLC tarafında değişiklik gerektirir ve tek başına bir süre sınırı getirmez.

### 5.2 Diğer

- **STLC'nin ürettiği test kodu:** Bölüm 4.2'deki exit 1/2 hataları STLC tarafında düzeltilmeli. LLM'e sınıfın gerçek API'si (metot imzaları) verilmeden anlamlı test üretilemez.
- **Kawasaki:** STLC ileride `pymoveit2_kawasaki_sim`'deki `sim_robot_goal`'ü hedeflerse (aynı sınıf adı, farklı dosya) mevcut `PYTHONPATH` yetmez. İkisi aynı anda path'e konursa biri diğerini gölgeler.
- **Algılama kuralı:** Entrypoint yalnızca sütun 0'daki `def test_` / `class Test` satırlarına bakıyor. Testleri başka bir yapı içinde tanımlayan dosyalar düz `python3` ile çalışır.
- **rclpy yaması:** `conftest.py` yalnızca `rclpy.init` ve `rclpy.shutdown`'u yamalıyor. `from rclpy import init` ile doğrudan import edilen fonksiyonlar yamalanmaz. Mevcut 85 `exec-*` dosyasının hiçbiri rclpy'ı doğrudan kullanmıyor.
- **Canlı yığında hareket:** Hareket komutu gönderen bir script, kalıcı yığın açıkken Gazebo/fake hardware'deki robotu gerçekten hareket ettirir. Yığın `use_fake_hardware:=false` ile açıkken koşu yapılmamalı.
