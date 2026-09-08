#!/usr/bin/env python3
"""kawasaki_dynamic_joint_states_topic  ->  ros-kawasaki-joint-states

4 Eyl 2026'da eklendi. Bu indeks 8 Haziran'dan beri bostu ve arayuzdeki
"Kawasaki — Joint Positions (Real)" paneli hicbir sey cizmiyordu; sebep
yazicida degil, koprude eksik bir abonelikti (bkz. double_ros2_kafka_bridge.py
icindeki kawasaki_joint_states_callback).

Kendi tuketici grubu var: paylasilan gruba katilsaydi mevcut 10 uyeyi
yeniden dengelemeye zorlardi ve 'latest' baslangici oradaki 'earliest'
davranisiyla celisirdi.
"""

import kafka_es_common as common

if __name__ == "__main__":
    common.run(
        topic="kawasaki_dynamic_joint_states_topic",
        index="ros-kawasaki-joint-states",
        group_id="ros2_kafka_bridge_kawasaki_real",
        # 'latest': gecmisi degil, bundan sonrasini topluyoruz.
        auto_offset_reset="latest",
    )
