#!/usr/bin/env python3
"""ur_rtde_data  ->  ur-rtde-data

Toplu yazmaya gecirildi (4 Eyl 2026). Onceki surum mesaj basina es.index() +
consumer.commit() + print() yapiyordu ve 114 Hz'de tavan yapiyordu; kopru ayni
anda 495 Hz uretiyordu, yani her saniye 381 mesaj geride kaliniyordu. Ortak
dongu ve gerekcesi icin kafka_es_common.py'ye bak.
"""

import kafka_es_common as common

if __name__ == "__main__":
    common.run(
        topic="ur_rtde_data",
        index="ur-rtde-data",
        group_id="ros2_elasticsearch_consumer",
        auto_offset_reset="earliest",
    )
