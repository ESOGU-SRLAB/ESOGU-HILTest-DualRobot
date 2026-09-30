#!/usr/bin/env python3
"""joint_states_topic  ->  ros-agv-joint-states | ros-kawasaki-joint-states

Ortak toplu yazma donguşune gecirildi (4 Eyl 2026). Filtre ve indeks secimi
orijinaliyle birebir ayni; degisen sey print gurultusu ve bos gecen anlarda
yarim yiginin yazilmadan beklemesi (bkz. kafka_es_common.py).

DIKKAT: bu topic koprunun /joint_states aboneliginden besleniyor ve o topic
bu hucrede UR10e eklemlerini tasiyor (ur10e_*). Kawasaki'nin gercek verisi
/kawasaki/dynamic_joint_states uzerinde ve kopru ONA ABONE DEGIL - bu yuzden
ros-kawasaki-joint-states pratikte bos kaliyor. Buradaki filtre dogru
calisiyor, eksik olan koprudeki abonelik.
"""

import kafka_es_common as common


def is_world_to_agv(data):
    return "world_to_agv" in data


def is_kawasaki(data):
    return "joint1" in data


def should_index(data):
    return is_world_to_agv(data) or is_kawasaki(data)


def index_for(data):
    if is_world_to_agv(data):
        return "ros-agv-joint-states"
    if is_kawasaki(data):
        return "ros-kawasaki-joint-states"
    return "ros-joint-states-unknown"


if __name__ == "__main__":
    common.run(
        topic="joint_states_topic",
        index="ros-kawasaki-joint-states",   # index_for baskin, bu yalnizca varsayilan
        group_id="ros2_kafka_bridge_kawa_agv",
        auto_offset_reset="latest",
        should_index=should_index,
        index_for=index_for,
    )
