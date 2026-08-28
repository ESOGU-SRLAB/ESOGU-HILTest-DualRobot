# recordings/

`gemini_pick_place.launch.py` her koşuyu buraya, kendi klasörüne yazar:

    recordings/<YYYYmmdd_HHMMSS>[_<run_name>]/

Kayıt düğümü PASİFTİR: yalnız dinler, hiçbir şey komutlamaz. Kapatmak için
`record:=false`.

## Kullanım

    ros2 launch gemini_robotics_ros gemini_pick_place.launch.py mode:=real \
        run_name:=kabartma_01 note:="kucuk kutu, bandin sonunda"

Sonra komutu her zamanki gibi gönderin:

    ros2 topic pub --once /gemini/command std_msgs/String \
        "{data: 'Pick up the object on the conveyor belt and ...'}"

## Klasörün içeriği

| dosya | ne var |
|---|---|
| `meta.json` | koşu kimliği, not, git commit'i ve kirli olup olmadığı, topic adları |
| `events.jsonl` | `/gemini/status` + `/gemini/record` ham akışı, satır başına bir olay |
| `timeline.csv` | durum geçişleri ve her aşamanın süresi |
| `arrivals.csv` | hedef konum, TF'ten okunan kap ucu, sapma (mm), doğrusal eksen |
| `surfaces.csv` | yama/bant nokta sayısı, RMS artık, normal, oturtma sapması, parça boyu |
| `er_queries.csv` | model çağrısı: sorgu, gecikme, dönen ham noktalar, kare dosyası |
| `vacuum.csv` | VGC10'un bildirdiği bağıl vakum zaman serisi |
| `joint_states.csv` | eklem konumları (seyreltilmiş) |
| `tcp_track.csv` | kap ucunun TF izi |
| `frames/` | her model çağrısı ANINDA: `_er.png` (modele giden kare), `_depth.npy` (ham), `_depth_mm.png`, `_intensity.png`, `_caminfo.json` |
| `run_summary.txt` | koşu bitince yazılan insan-okur özet |

## Neden `.npy`

`frames/*_depth.npy` ham derinliktir, ölçeklenmemiştir. `_caminfo.json` ile
birlikte render'ı sonradan yeniden üretmeye yeter; yani "kabartma yerine normal
gölgeleme verseydik model neyi işaret ederdi" sorusu robota bir daha dokunmadan
yanıtlanabilir. PNG sıkıştırması bu iş için yeterli değildir.

## Sayıların anlamı

`arrivals.csv` içindeki sapma, TF'ten okunan kap ucu ile o harekete KOMUT
EDİLEN konumun farkıdır: izleme doğruluğudur, bağımsız bir metroloji sistemine
karşı mutlak doğruluk değildir.
