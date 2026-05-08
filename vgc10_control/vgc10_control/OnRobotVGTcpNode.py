#!/usr/bin/env python3

import rclpy
import vgc10_modbus_tcp.comModbusTcp
import vgc10_control.baseOnRobotVG
from vgc10_msgs.msg import OnRobotVGInput
from vgc10_msgs.msg import OnRobotVGOutput
import threading
import time


def main():
    try:
        rclpy.init()
        node = rclpy.create_node("OnRobotVGTcpNode")
        node.declare_parameter('/onrobot/ip', '192.168.0.4')
        node.declare_parameter('/onrobot/port', '502')
        node.declare_parameter('/onrobot/dummy', False)

        ip = node.get_parameter('/onrobot/ip').value
        port = node.get_parameter('/onrobot/port').value
        dummy = node.get_parameter('/onrobot/dummy').value

        # Gripper is a VG gripper with a Modbus/TCP connection
        gripper = vgc10_control.baseOnRobotVG.onrobotbaseVG()
        gripper.client = vgc10_modbus_tcp.comModbusTcp.communication(dummy)

        # Connects to the ip address received as an argument
        gripper.client.connectToDevice(ip, port)
        node.get_logger().info(f"Connected to gripper at {ip}:{port}")

        # The Gripper status is published on the topic named 'OnRobotVGInput'
        pub = node.create_publisher(OnRobotVGInput, 'OnRobotVGInput', 1)

        def on_command(msg):
            """Subscription callback: immediately send command to gripper."""
            gripper.refreshCommand(msg)
            node.get_logger().info(
                f"Command received: rmca={msg.rmca} rvca={msg.rvca} "
                f"rmcb={msg.rmcb} rvcb={msg.rvcb} — sending to gripper."
            )
            gripper.sendCommand()

        # The Gripper command is received from the topic named 'OnRobotVGOutput'
        subscription = node.create_subscription(
            OnRobotVGOutput, 'OnRobotVGOutput', on_command, 10)

        # Status publishing timer at 10 Hz
        def publish_status():
            status = gripper.getStatus()
            pub.publish(status)

        node.create_timer(0.1, publish_status)

        # Spin in main thread
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
