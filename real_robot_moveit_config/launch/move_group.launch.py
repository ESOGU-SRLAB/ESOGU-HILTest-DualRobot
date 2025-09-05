from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 1. MoveIt konfigürasyonlarını her zamanki gibi yükle
    moveit_config = (
        MoveItConfigsBuilder("my_robot_cell", package_name="real_robot_moveit_config")
        .to_moveit_configs()
    )

    # 2. Yüklenen tüm konfigürasyonları bir Python sözlüğüne (dictionary) dönüştür
    move_group_params = moveit_config.to_dict()

    # 3. Kilit Adım: 'use_sim_time' parametresini sözlüğe manuel olarak ekle
    move_group_params["use_sim_time"] = False

    # 4. 'move_group' düğümünü manuel olarak oluştur ve değiştirilmiş parametreleri kullan
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        # Tüm parametreleri (bizim eklediğimiz dahil) düğüme aktar
        parameters=[move_group_params],
    )

    # 5. Oluşturduğumuz düğümü içeren LaunchDescription'ı döndür
    return LaunchDescription([move_group_node])