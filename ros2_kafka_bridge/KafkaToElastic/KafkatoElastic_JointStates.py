#!/usr/bin/env python3
"""dynamic_joint_states_topic  ->  ros-joint-states

Ortak toplu yazma donguşune gecirildi (4 Eyl 2026). Bu yazici zaten toplu
yaziyordu ve 500 Hz'e yetisiyordu; degisimin sebebi hiz degil, iki sey:

  1. Mesaj basina/yigin basina print. 500 Hz'de saniyede ~5 satir uretip
     run_all.sh ciktisini okunmaz hale getiriyordu.
  2. Zamanli flush'in `for msg in consumer` dongusunun ICINDE olmasi. Hucre
     susunca eldeki yarim yigin (500'e kadar belge) yeni mesaj gelene dek
     yazilmadan bekliyordu - yani bir kosunun SON belgeleri ancak bir sonraki
     kosu basladiginda Elasticsearch'e dusuyordu. Ortak dongu poll() kullanip
     mesaj gelmese de flush ediyor.
"""

import kafka_es_common as common

if __name__ == "__main__":
    common.run(
        topic="dynamic_joint_states_topic",
        index="ros-joint-states",
        group_id="ros2_kafka_bridge_realtime",
        # 'latest': bu topic 500 Hz akiyor, gecmisi bastan indekslemek
        # istenmiyor. Orijinal davranis buydu, korundu.
        auto_offset_reset="latest",
    )
