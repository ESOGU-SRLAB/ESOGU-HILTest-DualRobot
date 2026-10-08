# Task Command penceresinde varsayılan komut

Tarih: 1 Ekim 2026

Arayüzdeki **Task Command** penceresi (`/gemini/command`) artık sınanmış komut
kutuda **gerçek metin** olarak yazılı açılıyor. Eskiden yalnızca gri gölge
(placeholder) olarak görünüyordu ve elle yazmak gerekiyordu. Şimdi pencere
açılınca doğrudan **Confirm**'e (ya da Ctrl+Enter) basmak yetiyor.

Varsayılan komut:

```
Pick up the flat rectangular object lying on the conveyor belt and place it into one of the empty open-top bins in the top row of the separate multi-level bin rack that stands apart from the conveyor.
```

İki dosya değişiyor, ikisi de `user_interface/` altında. `gemini_robotics_ros`
paketine dokunulmuyor, `colcon build` gerekmiyor.

---

## 1) `user_interface/static/js/dashboard.js`

"Task Command Modal (/gemini/command)" başlığının hemen altında.

### Eski

```js
// Son gönderilen komut: pencere yeniden açıldığında kutuda hazır bekler,
// çünkü aynı görev çoğu zaman ufak bir değişiklikle tekrar deneniyor.
let lastCommandText = "";

function openCommandModal() {
    const modal = document.getElementById("command-modal");
    const box = document.getElementById("command-text");
    setCommandStatus("", "");
    if (lastCommandText) box.value = lastCommandText;
    modal.classList.add("visible");
```

### Yeni

```js
// Son gönderilen komut: pencere yeniden açıldığında kutuda hazır bekler,
// çünkü aynı görev çoğu zaman ufak bir değişiklikle tekrar deneniyor.
//
// Başlangıç değeri placeholder DEĞİL, gerçek metin: pencere açılınca doğrudan
// Confirm'e basılabilsin. Bu cümle kayıtlı koşularda sınandı - ayırt edici
// bilgi ("separate multi-level bin rack") bırakma isim öbeğinin içinde olduğu
// için planner kısaltırken atmıyor.
const DEFAULT_COMMAND_TEXT =
    "Pick up the flat rectangular object lying on the conveyor belt and place " +
    "it into one of the empty open-top bins in the top row of the separate " +
    "multi-level bin rack that stands apart from the conveyor.";
let lastCommandText = DEFAULT_COMMAND_TEXT;

function openCommandModal() {
    const modal = document.getElementById("command-modal");
    const box = document.getElementById("command-text");
    setCommandStatus("", "");
    box.value = lastCommandText;
    modal.classList.add("visible");
```

Değişen üç şey:

1. `DEFAULT_COMMAND_TEXT` sabiti eklendi.
2. `let lastCommandText = "";` -> `let lastCommandText = DEFAULT_COMMAND_TEXT;`
3. `if (lastCommandText) box.value = lastCommandText;` -> `box.value = lastCommandText;`

Dosyanın geri kalanı (`sendCommand`, `closeCommandModal`, ...) aynı kalıyor.

---

## 2) `user_interface/templates/index.html`

"TASK COMMAND MODAL (/gemini/command)" bloğundaki `<textarea id="command-text">`.
Yalnızca `placeholder` özniteliği değişiyor; bu metin artık sadece kutu
tamamen silinirse görünür.

### Eski

```html
            <textarea id="command-text" class="modal-textarea" rows="5" spellcheck="false"
                      placeholder="There is a thing on the conveyor belt. I want you to pick that up and place it in one of the empty compartments in the toolkit's top row"></textarea>
```

### Yeni

```html
            <textarea id="command-text" class="modal-textarea" rows="5" spellcheck="false"
                      placeholder="Pick up the flat rectangular object lying on the conveyor belt and place it into one of the empty open-top bins in the top row of the separate multi-level bin rack that stands apart from the conveyor."></textarea>
```

---

## Uyguladıktan sonra

1. Sözdizimi kontrolü (isteğe bağlı):

   ```bash
   node --check user_interface/static/js/dashboard.js
   ```

2. Tarayıcıda **Ctrl+Shift+R** ile sayfayı önbelleği atlayarak yenileyin.
   `dashboard.js` sürüm parametresi olmadan yükleniyor
   (`<script src="/static/js/dashboard.js">`), normal yenileme eski dosyayı
   gösterebilir.

3. Task Command penceresini açın: komut kutuda yazılı (gri değil) gelmeli.

## Davranış notları

- Metni değiştirip gönderirseniz pencere bir sonraki açılışta **son
  gönderdiğinizi** gösterir (eski davranış korunuyor).
- Sayfa yenilenince varsayılan komuta geri döner.
- Varsayılanı değiştirmek için tek yer `dashboard.js` içindeki
  `DEFAULT_COMMAND_TEXT`; `index.html`'deki placeholder'ı da aynı tutun.

---
---

# EK: `_spin_or_sleep` düzeltmesi (pymoveit2_real) — HRC'de yeşil buton algılanmıyor

Tarih: 1 Ekim 2026

Bu bölüm yukarıdaki Task Command değişikliğinden **bağımsızdır**; yalnızca aynı
dosyayla diğer bilgisayara taşınsın diye buraya eklendi.

## Belirti

`ros2 launch pymoveit2_real human_robot_collaboration_scenario.launch.py`
gerçek robotta çalışırken robot `firstTop` noktasına gelip
`>>> OPERATÖR BEKLENİYOR: yeşil butona basın (DIN7) <<<` yazıyor, butona
basılıyor ama vidalama başlamıyor, robot orada kalıyor. Terminalde şunlardan
biri ya da ikisi görünüyor:

```
rclpy._rclpy_pybind11.RCLError: Failed to get number of ready entities for action client: wait set index for status subscription is out of bounds, at ./src/rcl_action/action_client.c:623
```

```
io_states 10.6 sn'dir guncellenmiyor; dijital girisler BAYAT. Buton basilsa bile gorulmez.
```

Buton, kablo ve sürücü sağlam; sorun yazılımda.

## Sebep

Senaryo node'u `main()` içinde arka planda bir `MultiThreadedExecutor` thread'i
ile spin ediliyor (`io_states` aboneliği ve `set_io` yanıtları bu thread'den
geliyor). `pymoveit2_real/moveit2.py` içindeki bekleme döngüleri ise ana
thread'de `rclpy.spin_once(self._node, ...)` çağırıyor. Böylece aynı node'u, ve
onun action client'ını, iki executor aynı anda bekliyor; rclpy yukarıdaki
`RCLError`'ı fırlatıyor ve **arka plan thread'i ölüyor**. O andan sonra
`io_states` hiç güncellenmiyor, DIN7 eski değerinde kalıyor.

Zamanlamaya bağlı bir yarış durumu olduğu için her çalıştırmada çıkmıyor
("dün çalışıyordu").

## Değişiklik: `pymoveit2_real/pymoveit2_real/moveit2.py`

Tek dosya. İki adım.

### 1) Yardımcı metodu ekle

`MoveIt2` sınıfında, `def __await_future(self, future, timeout: float) -> bool:`
satırının **hemen üstüne** (aynı girinti, 4 boşluk):

```python
    def _spin_or_sleep(self, timeout_sec: float = 1.0):
        """Let callbacks run while a blocking call waits. If the node already lives in
        the caller's own executor (e.g. a background MultiThreadedExecutor), just sleep
        and let that executor deliver the callbacks. Calling rclpy.spin_once() there
        makes TWO executors wait on the same action client at once, and rclpy raises
        'wait set index for status subscription is out of bounds' -- which kills the
        caller's executor thread and freezes every subscription (io_states, ...)."""
        executor = self._node.executor
        if (
            executor is not None
            and executor is not rclpy.get_global_executor()
            and self._node in executor.get_nodes()
        ):
            time.sleep(0.01)
        else:
            rclpy.spin_once(self._node, timeout_sec=timeout_sec)

```

`time` ve `rclpy` dosyanın başında zaten import edili; yeni import gerekmiyor.

### 2) Beş `spin_once` çağrısını değiştir

Dosyada tam olarak **5 yerde** geçen şu satır:

```python
rclpy.spin_once(self._node, timeout_sec=1.0)
```

şuna çevrilecek:

```python
self._spin_or_sleep(timeout_sec=1.0)
```

Bulundukları yerler (satır numaraları yaklaşık):

| Satır | Döngü | Metot |
|---|---|---|
| ~577 | `while not future.done():` | senkron `plan(...)` |
| ~694 | `while start_joint_state is None:` | `plan_async(...)` (joint state bekleme) |
| ~802 | `while self.__is_motion_requested or self.__is_executing:` | `wait_until_executed()` |
| ~1254 | `while not future.done():` | `compute_fk(...)` |
| ~1354 | `while not future.done():` | `compute_ik(...)` |

Tek komutla (paket kökünden, yani `pymoveit2_real/` içinden):

```bash
sed -i 's/rclpy\.spin_once(self\._node, timeout_sec=1\.0)$/self._spin_or_sleep(timeout_sec=1.0)/' pymoveit2_real/moveit2.py
```

İki adımın sırası önemli değil: `sed` yardımcı metodun içindeki
`rclpy.spin_once(...)` satırına dokunmaz, çünkü o satırda
`timeout_sec=timeout_sec` yazıyor.

### Kontrol

```bash
grep -n "spin_once(\|_spin_or_sleep(" pymoveit2_real/moveit2.py
```

Beklenen: 5 adet `self._spin_or_sleep(timeout_sec=1.0)`, 1 adet
`def _spin_or_sleep`, ve yalnızca yardımcı metodun içinde 1 adet gerçek
`rclpy.spin_once(self._node, timeout_sec=timeout_sec)` çağrısı (diğer
eşleşmeler docstring/yorumdur).

```bash
python3 -m py_compile pymoveit2_real/moveit2.py
```

## Uyguladıktan sonra

```bash
cd ~/colcon_ws
colcon build --packages-select pymoveit2_real
source install/setup.bash
```

Paket symlink ile kurulmuyor; **build yapılmadan değişiklik etkili olmaz.**
Çalışan senaryo varsa kapatıp yeniden başlatın.

Doğrulama: senaryo logunda artık
`Node executor'den kopmustu (pymoveit2 spin_once), geri baglandi.` uyarısı
çıkmamalı, buton beklenirken `BAYAT` hatası görülmemeli.

## Davranış notları

- Node kendi executor'üne bağlı **değilse** (tek thread'li eski scriptler)
  metot eskisi gibi `rclpy.spin_once` çağırır; o scriptlerin davranışı
  değişmez.
- Senaryodaki `_ensure_executor()` olduğu gibi kalıyor, zararı yok.
- `pymoveit2_sim`, `pymoveit2_kawasaki_real`, `pymoveit2_kawasaki_sim`
  paketlerinin kendi `moveit2.py` kopyalarına dokunulmadı.
- **Kalıcılık:** `~/colcon_ws/src` GitHub'dan (`ESOGU-HILTest-DualRobot`)
  yeniden klonlanınca bu değişiklik silinir (30 Eyl'de uygulanmış, 1 Ekim
  14:10'daki klonlamayla kaybolmuştu). Kalıcı olması için repoya push
  edilmesi gerekiyor.
- **Durum:** IFARLAB'da 1 Ekim 2026'da uygulanıp derlendi; gerçek robotta
  dört vidalık tam turla henüz doğrulanmadı.

## İlgili ama ayrı sorun: trajectory kaydında 360° kayık bilek

Aynı gün `pymoveit2_real/trajectories/hrc_screwing.json` içinde 29 segmentin
8'inde `ur10e_wrist_1_joint` olması gerekenden 360° fazla kayıtlıydı (ör.
−103° yerine +257°). Sonuç: 10/48 adımı (`tookScrew`) üç denemede de
`Planning failed! Error code: FAILURE` verip atlanıyor. Bu dosya makineye
özgü bir önbellek; diğer bilgisayarda aynı belirti görülürse robot home'da ve
`wrist_1` ≈ −90° iken dosyayı silip ilk turu yeniden kaydettirin:

```bash
rm ~/colcon_ws/src/pymoveit2_real/trajectories/hrc_screwing.json
```
