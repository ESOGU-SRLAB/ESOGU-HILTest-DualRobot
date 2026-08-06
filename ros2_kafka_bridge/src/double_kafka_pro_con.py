#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Dummy Kafka 2 Producer - 2 Consumer example: double_kafka_pro_con.py
    # V.2.0.0.
#----------------------------------------------------------------------------------------

import time
import json
import threading
from kafka import KafkaProducer
from kafka import KafkaConsumer

class double_kafka_pro_con():

    def __init__(self):

        time.sleep(5) # Wait for the bridge to start

        self.producer1 = KafkaProducer(bootstrap_servers='localhost:9092', value_serializer=lambda m: json.dumps(m).encode('ascii'))
        self.producer2 = KafkaProducer(bootstrap_servers='localhost:9092', value_serializer=lambda m: json.dumps(m).encode('ascii'))

        self.consumer1 = KafkaConsumer('bridge_to_kafka_topic_1',
                                  bootstrap_servers='localhost:9092',
                                  value_deserializer=lambda m: json.loads(m.decode('ascii')),
                                  auto_offset_reset='latest',
                                  consumer_timeout_ms=2000)
        
        self.consumer2 = KafkaConsumer('bridge_to_kafka_topic_2',
                                  bootstrap_servers='localhost:9092',
                                  value_deserializer=lambda m: json.loads(m.decode('ascii')),
                                  auto_offset_reset='latest',
                                  consumer_timeout_ms=2000)

        
        
        self.i = 0
        self.j = 0

        self.topic1()
        self.topic2()
        self.reader1()
        self.reader2()
        self.main()

    def topic1(self):
        def topic1_callback_local():
            while (1):
                msg = 'KAFKA: Hello World A: {0}'.format(self.i)
                self.i += 1
                self.producer1.send('kafka_to_bridge_topic_1', {"data": msg})
                time.sleep(0.7)
        t1 = threading.Thread(target=topic1_callback_local)
        t1.daemon = True
        t1.start()
    
    def topic2(self):
        def topic2_callback_local():
            while (1):
                msg = 'KAFKA: Hello World B: {0}'.format(self.j)
                self.j += 1
                self.producer2.send('kafka_to_bridge_topic_2', {"data": msg})
                time.sleep(0.5)
        t2 = threading.Thread(target=topic2_callback_local)
        t2.daemon = True
        t2.start()

    def reader1(self):
        def reader1_callback_local():
            while (1):
                for message in self.consumer1:
                    print("I heard %s", message.value)
                    break
                time.sleep(0.01)
        t3 = threading.Thread(target=reader1_callback_local)
        t3.daemon = True
        t3.start()
    
    def reader2(self):
        def reader2_callback_local():
            while (1):
                for message in self.consumer2:
                    print("I heard %s", message.value)
                    break
                time.sleep(0.01)
        t4 = threading.Thread(target=reader2_callback_local)
        t4.daemon = True
        t4.start()
    
    def main(self):
        while (1):
            pass
    
    # consumer_dynamic_joint_states = KafkaConsumer(
    #         'dynamic_joint_states_topic',
    #         bootstrap_servers='localhost:9092',
    #         value_deserializer=lambda m: json.loads(m.decode('ascii')),
    #         auto_offset_reset='latest'
    #     )

    # for message in consumer_dynamic_joint_states:
    #     print("Received joint states:", message.value)

    # consumer_monitored_planning_scene_topic = KafkaConsumer(
    #         'monitored_planning_scene_topic',
    #         bootstrap_servers='localhost:9092',
    #         value_deserializer=lambda m: json.loads(m.decode('ascii')),
    #         auto_offset_reset='latest'
    #     )

    # for message in consumer_monitored_planning_scene_topic:
    #     print("Received consumer_monitored_planning_scene_topic:", message.value)
    
    # consumer_interactive_marker_update_topic = KafkaConsumer(
    #         'interactive_marker_update_topic',
    #         bootstrap_servers='localhost:9092',
    #         value_deserializer=lambda m: json.loads(m.decode('ascii')),
    #         auto_offset_reset='latest'
    #     )

    # for message in consumer_interactive_marker_update_topic:
    #     print("Received interactive_marker_update_topic:", message.value)
    
    # consumer_scaled_joint_trajectory_controller_topic = KafkaConsumer(
    #         'scaled_joint_trajectory_controller_topic',
    #         bootstrap_servers='localhost:9092',
    #         value_deserializer=lambda m: json.loads(m.decode('ascii')),
    #         auto_offset_reset='latest'
    #     )

    # for message in consumer_scaled_joint_trajectory_controller_topic:
    #     print("Received scaled_joint_trajectory_controller_topic:", message.value)
    
    # consumer_scaled_joint_trajectory_controller_state_topic = KafkaConsumer(
    #         'scaled_joint_trajectory_controller_state_topic',
    #         bootstrap_servers='localhost:9092',
    #         value_deserializer=lambda m: json.loads(m.decode('ascii')),
    #         auto_offset_reset='latest'
    #     )

    # for message in consumer_scaled_joint_trajectory_controller_state_topic:
    #     print("Received scaled_joint_trajectory_controller_state_topic:", message.value)
    # #asdasdas
    # consumer_tool_data_topic = KafkaConsumer(
    #         'tool_data_topic',
    #         bootstrap_servers='localhost:9092',
    #         value_deserializer=lambda m: json.loads(m.decode('ascii')),
    #         auto_offset_reset='latest'
    #     )

    # for message in consumer_tool_data_topic:
    #     print("Received tool_data_topic:", message.value)

    # consumer_force_torque_sensor_topic = KafkaConsumer(
    #         'force_torque_sensor_topic',
    #         bootstrap_servers='localhost:9092',
    #         value_deserializer=lambda m: json.loads(m.decode('ascii')),
    #         auto_offset_reset='latest'
    #     )

    # for message in consumer_force_torque_sensor_topic:
    #     print("Received force_torque_sensor_topic:", message.value)

    # consumer_tcp_pose_topic = KafkaConsumer(
    #         'tcp_pose_topic',
    #         bootstrap_servers='localhost:9092',
    #         value_deserializer=lambda m: json.loads(m.decode('ascii')),
    #         auto_offset_reset='latest'
    #     )

    # for message in consumer_tcp_pose_topic:
    #     print("Received tcp_pose_topic:", message.value)

    topics = [
    'sim_point_cloud_topic',
    'sim_image_topic',
    'sim_joint_states_topic',
    'dynamic_joint_states_topic',
    'monitored_planning_scene',
    'interactive_marker',
    'scaled_joint_trajectory',
    'scaled_joint_trajectory_controller_state_topic',
    'tool_data_topic',
    'force_torque_sensor_topic',
    'tcp_pose_topic'
    ]

    # Kafka consumer oluştur
    consumer = KafkaConsumer(
        *topics,  # Tüm topic'leri burada dinliyoruz
        bootstrap_servers='localhost:9092',
        value_deserializer=lambda m: json.loads(m.decode('utf-8')),
        auto_offset_reset='latest',  # 'earliest' yaparsan tüm geçmişi okur
        group_id='debug_consumer'  # İstersen bir grup ID ver
    )

    # Sonsuz döngüde verileri dinle
    for message in consumer:
        print(f"[{message.topic}] Received: {message.value}")
    
if __name__ == '__main__':
    try:
        double_kafka_pro_con()
    except KeyboardInterrupt:
        pass
