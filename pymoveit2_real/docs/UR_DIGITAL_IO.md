# UR10e Dijital I/O — IFARLAB hücresi pin haritası

Pendant'taki **GPIO** sekmesinden yürütülen işlerin ROS 2 karşılığı. Bu dosyanın
varlık sebebi: pendant'ta pinlere verilen **isimler RTDE üzerinden yayınlanmıyor**,
`ur_msgs/msg/Digital` yalnızca sayısal indeks taşıyor (`uint8 pin`, `bool state`).
Yani isimden pin bulmanın yolu yok — bir kez ölçüp yazmak gerekiyor.

> Ölçüm: 3 Eylül 2026, `find_io_pins.py` ile, pinler tek tek elle tetiklenerek.

## Pin haritası

| Yön | Pin | İşlev | Kodda kullanılıyor mu |
|---|---|---|---|
| ÇIKIŞ | `standard_digital_out[0]` | Vidalama **SIKMA** | Evet — `screwdriver_pin` |
| ÇIKIŞ | `standard_digital_out[1]` | Vidalama **SÖKME** | Bağlı, senaryoda kullanılmıyor — `screwdriver_reverse_pin` |
| GİRİŞ | `standard_digital_in[7]` | **YEŞİL** buton (operatör onayı) | Evet — `green_button_pin` |
| GİRİŞ | `standard_digital_in[6]` | **KIRMIZI** buton | Rezerve — `red_button_pin` |
| GİRİŞ | `standard_digital_in[5]` | **BEYAZ** buton | Rezerve — `white_button_pin` |

## ROS 2 arayüzleri

| İş | Arayüz | Tip |
|---|---|---|
| Çıkış yazma | `/io_and_status_controller/set_io` | `ur_msgs/srv/SetIO` |
| Giriş okuma | `/io_and_status_controller/io_states` | `ur_msgs/msg/IOStates` |

Kontrolcü `ur_controllers/GPIOController`; `ur_control.launch.py` içinde
varsayılan olarak aktif edilir.

### Pin numaralandırması

Yazma ve okuma **aynı indeks uzayını** kullanır:

```
 0 -  7   standard digital out / in
 8 - 15   configurable out / in
16 - 17   tool digital out / in
```

`SetIO` çağrısında `fun: 1` (`FUN_SET_DIGITAL_OUT`) üç aralığı da kapsar —
`gpio_controller.cpp` pin 0..17 arasını tek dalda ele alıp `hardware_interface.cpp`
içinde RTDE writer'ın `sendStandardDigitalOutput` / `sendConfigurableDigitalOutput`
/ `sendToolDigitalOutput` çağrılarına dağıtır.

### Elle deneme

```bash
# Çıkış sür (pin 0 = vidalama sıkma)
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO \
    "{fun: 1, pin: 0, state: 1.0}"

# Girişleri izle
ros2 topic echo /io_and_status_controller/io_states --field digital_in_states
```

## Bilinmesi gerekenler

**IO yazma External Control programından bağımsızdır.** Komut RTDE üzerinden
gider; sürücü bağlıysa program koşmasa bile çıkış sürülür. (Kolu *hareket*
ettirmek için program şart.)

**Güvenlik konfigürasyonunda rezerve edilmiş configurable output'lar (8–15)
RTDE'den yazılamaz.** Yeni bir röle bağlarken standard output'lardan birini
(0–7) seçmek en güvenlisi.

**Sim / fake donanımda GPIO çalışmaz.** `mock_components` dijital girişleri hep
`0` döndürür; yeşil buton beklemesi sonsuza kadar takılırdı. Senaryo bunu
kendisi tespit edip GPIO'yu devre dışı bırakır (aşağıya bakın).

**AÇIK SORU — seviye mi, darbe mi?** Senaryo vidalama çıkışını iniş boyunca
HIGH tutup dönüşten önce LOW'a çeker (seviye). Aletin darbeyle tetiklenip kendi
çevrimini çalıştırdığı ortaya çıkarsa bu mantığın değişmesi gerekir. İlk gerçek
turda bakılacak ilk şey budur.

## Pinleri yeniden ölçmek

Kablolama değişirse:

```bash
ros2 run pymoveit2_real find_io_pins.py
```

İlk mesajı referans alır, sonra değişen her pini zaman damgasıyla basar; Ctrl+C
ile çıkınca özeti ve doğrudan kopyalanabilir launch satırını verir. Çıkışı ve
butonu **ayrı ayrı, aralarında birkaç saniye bırakarak** tetikleyin — robot
ayaktayken bazı çıkışlar kendiliğinden de değişebilir.

## Senaryodaki kullanımı

`examples/human_robot_collaboration_scenario.py` ilgili parametreleri
varsayılan olarak yukarıdaki haritayla gelir; normal kullanımda argüman
vermeniz gerekmez.

```bash
ros2 launch pymoveit2_real human_robot_collaboration_scenario.launch.py
```

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `screwdriver_pin` | `0` | Vidalama sıkma çıkışı |
| `screwdriver_reverse_pin` | `1` | Vidalama sökme çıkışı |
| `green_button_pin` | `7` | Operatör onay butonu |
| `red_button_pin` | `6` | Rezerve |
| `white_button_pin` | `5` | Rezerve |
| `green_button_active_high` | `true` | Buton NC ise `false` |
| `green_button_timeout` | `0.0` | `0.0` = süresiz bekle |
| `gpio_mode` | `auto` | `force_on` / `force_off` ile zorlanır |
| `fake_button_delay` | `0.0` | GPIO kapalıyken buton yerine beklenecek süre |

`gpio_mode:=off` yazımına dikkat: ROS parametre CLI'ında YAML kuralı gereği
`on`/`off`/`yes`/`no` **boolean** olarak ayrıştırılır. Kanonik değerler bu yüzden
`force_on` / `force_off`; launch tarafında string'e zorlanır ve `off` yazımı da
normalize edilir.

### `gpio_mode: auto` neye bakıyor

Sırayla:

1. `use_fake_hardware` parametresi
2. `ENV_USE_FAKE_HARDWARE` ortam değişkeni
3. **`robot_description`** — `/robot_state_publisher`'dan okunur; gerçek kolun
   (`ur10e_` önekli, `sim_ur10e_` değil) `ros2_control` bloğundaki donanım
   eklentisine bakılır. `mock_components`, `gz_ros2_control`, `ign_ros2_control`
   görülürse GPIO kapatılır.
4. `io_states` yayını + `set_io` servisinin varlığı

Hiçbiri gerçek donanıma işaret etmezse GPIO kapatılır, vidalama adımları atlanır
ve yeşil buton beklenmez — bu durumda konsola `ERROR` düşer.

## Vidalama akışı

Her vida için waypoint listesindeki sıra:

```
xTop
  → yeşil buton beklenir            (DIN7 yükselen kenar)
  → vidalama AÇ                     (DOUT0 HIGH)
  → xOpt      yavaş iniş            (screw_speed, varsayılan 0.01)
  → 1.5 sn bekleme
  → vidalama KAPAT                  (DOUT0 ve DOUT1 birlikte LOW)
  → xTop      yavaş çıkış           (screw_speed)
```

Buton beklemesi **yükselen kenar** arar: çağrı anında buton zaten basılıysa önce
bırakılması beklenir, yoksa tek basış iki vidalamayı tetiklerdi. Vidalama
kapatılırken sıkma ve sökme çıkışlarının ikisi birden düşürülür.
