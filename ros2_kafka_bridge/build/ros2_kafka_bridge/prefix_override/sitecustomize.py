import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cem/colcon_ws/src/ros2_kafka_bridge/install/ros2_kafka_bridge'
