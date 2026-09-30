#!/usr/bin/env python3
"""sim_image_topic  ->  ros_sim_image

Toplu yazmaya gecirildi (4 Eyl 2026). Mesaj basina print eden son yaziciydi;
5 Hz'de bogucu degildi ama run_all.sh ciktisini okunmaz hale getiriyordu.

Ayrica onceki surum @timestamp'i datetime.utcnow() ile, yani ISLENME aniyla
damgaliyordu. Ortak dongu once mesajin kendi ROS zaman damgasini ariyor
(kopru bu topic'e header.stamp.sec yaziyor), boylece karenin CEKILDIGI an
kaydediliyor - backlog boşaltilirken aradaki fark dakikalara cikabiliyor.
"""

import kafka_es_common as common

if __name__ == "__main__":
    common.run(
        topic="sim_image_topic",
        index="ros_sim_image",
        group_id="ros2_elasticsearch_consumer_v2",
        auto_offset_reset="earliest",
        # Kareler 30 KB civari; 500'luk yigin tek istekte 15 MB olurdu.
        batch_size=50,
    )
