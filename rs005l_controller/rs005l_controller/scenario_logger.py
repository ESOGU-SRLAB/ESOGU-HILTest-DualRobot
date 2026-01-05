#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
import csv
import os

BASE_FRAME = 'base_link'   # gerekirse değiştir: 'world' vb.
TCP_FRAME  = 'link6'    # sende 'tool0', 'ee_link' olabilir

class Logger(Node):
    def __init__(self):
        super().__init__('scenario_logger')
        self.buf = Buffer()
        self.tl  = TransformListener(self.buf, self)

        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.on_js, 10
        )

        out = 'rs_log.csv'
        self.csv_f = open(out, 'w', newline='')
        self.w = csv.writer(self.csv_f)
        self.w.writerow([
            't_sec',
            # joint states
            'q1','q2','q3','q4','q5','q6',
            # tcp pose in BASE_FRAME
            'x','y','z','qx','qy','qz','qw'
        ])
        self.get_logger().info(f'Writing to {os.path.abspath(out)}')

    def on_js(self, msg: JointState):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

        # joint angles (pad to 6 for RS005L)
        q = list(msg.position[:6]) + [0.0]*(6-len(msg.position))
        try:
            tf: TransformStamped = self.buf.lookup_transform(
                BASE_FRAME, TCP_FRAME, Time(), rclpy.duration.Duration(seconds=0.05)
            )
            tr = tf.transform
            row = [t] + q + [tr.translation.x, tr.translation.y, tr.translation.z,
                             tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]
        except Exception as e:
            # tf henüz hazır değilse NaN yaz
            row = [t] + q + [float('nan')]*7

        self.w.writerow(row)

    def destroy_node(self):
        self.csv_f.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
