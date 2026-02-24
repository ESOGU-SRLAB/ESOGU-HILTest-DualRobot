import os
from launch import LaunchDescription
from launch_ros.actions import Node

# Bu launch dosyası, hem gerçek robot hem de simülasyon için
# joint hedefi gönderen Python betiklerini başlatır.
# DİKKAT: İki düğümün de aynı anda çalışması topic/service çakışmalarına
# yol açabilir ve genellikle tavsiye edilmez.

def generate_launch_description():

    return LaunchDescription([
        # 1. Simülasyon Robot Kontrolcüsü Düğümü
        Node(
            package='pymoveit2_kawasaki_sim',
            executable='agv_kawa_goal.py',
            name='kawasaki_joint_controller',
            output='screen',
            emulate_tty=True, # Logların düzgün görünmesi için
        ),
        
        # 2. Gerçek Robot Kontrolcüsü Düğümü
        Node(
            package='pymoveit2_sim',
            executable='sim_robot_joint_goal.py',
            name='ur10e_joint_controller',
            output='screen',
            emulate_tty=True, # Logların düzgün görünmesi için
        ),
    ]
)