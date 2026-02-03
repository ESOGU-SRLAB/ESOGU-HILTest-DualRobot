#!/usr/bin/env python3
"""
Sensing ve Cleaning Mission Launch Dosyası

Bu launch dosyası 3 node çalıştırır:
1. sensing_robot - Robot şaseyi tarar
2. red_detection_node - Kırmızı noktaları tespit eder
3. cleaning_mission_runner - Tespit edilen noktalara gider (sensing tamamlandıktan sonra)

sensing_robot /progress_status topic'inden "SENSING COMPLETED" yayınladığında,
20 saniye beklenir ve cleaning_mission_runner başlatılır.
"""

import os
import subprocess
import threading
import time

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

import rclpy
from rclpy.node import Node as RclpyNode
from std_msgs.msg import String


class SensingCompletedWatcher:
    """SENSING COMPLETED mesajını bekleyen ve cleaning mission başlatan sınıf"""
    
    def __init__(self):
        self.sensing_completed = False
        self.cleaning_started = False
        self.lock = threading.Lock()
    
    def start_watching(self):
        """Topic'i dinlemeye başla"""
        self.watch_thread = threading.Thread(target=self._watch_topic, daemon=True)
        self.watch_thread.start()
    
    def _watch_topic(self):
        """Topic'i dinle ve SENSING COMPLETED gelince cleaning mission başlat"""
        rclpy.init()
        
        node = rclpy.create_node('sensing_completed_watcher')
        
        def callback(msg):
            if msg.data == "SENSING COMPLETED" and not self.cleaning_started:
                with self.lock:
                    if not self.cleaning_started:
                        self.cleaning_started = True
                        
                        # Ayrı thread'de bekle ve cleaning başlat
                        def start_cleaning():
                            time.sleep(20)
                            
                            # Cleaning mission runner'ı başlat
                            subprocess.Popen([
                                'ros2', 'run', 'pymoveit2_sim', 'cleaning_mission_runner.py'
                            ])
                        
                        cleaning_thread = threading.Thread(target=start_cleaning, daemon=True)
                        cleaning_thread.start()
        
        subscription = node.create_subscription(
            String,
            '/progress_status',
            callback,
            10
        )
        
        try:
            rclpy.spin(node)
        except:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


# Global watcher instance
watcher = SensingCompletedWatcher()


def start_watcher(context):
    """Watcher'ı başlat"""
    watcher.start_watching()
    return []


def generate_launch_description():
    
    # 1. Sensing Robot Node
    sensing_robot_node = Node(
        package='pymoveit2_sim',
        executable='sensing_robot.py',
        name='sensing_robot',
        output='screen',
        emulate_tty=True,
    )
    
    # 2. Red Detection Node
    red_detection_node = Node(
        package='pymoveit2_sim',
        executable='red_detection_node.py',
        name='red_detection_node',
        output='screen',
        emulate_tty=True,
    )
    
    # 3. Watcher'ı başlat (OpaqueFunction ile)
    start_watcher_action = OpaqueFunction(function=start_watcher)
    
    return LaunchDescription([
        # Önce watcher'ı başlat
        start_watcher_action,
        
        # Sonra sensing robot ve red detection node'larını başlat
        sensing_robot_node,
        red_detection_node,
    ])
