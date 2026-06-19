#!/usr/bin/env python3
"""
Real world UR10e Executor - Continuous Mode

UR10e (+ lineer eksen) sonsuz döngüde JSON waypoint'lerini takip eder.
"""

import json
import os
import time
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as realrobot

UR_VELOCITY  = 0.2
UR_ACCEL     = 0.1


class URExecutor(Node):
    def __init__(self):
        super().__init__("ur_executor")
        cb = ReentrantCallbackGroup()

        self.ur_moveit = MoveIt2_Real(
            node=self,
            joint_names=realrobot.joint_names(),
            base_link_name="world",
            end_effector_name=realrobot.end_effector_name(),
            group_name=realrobot.MOVE_GROUP_ARM,
            callback_group=cb,
        )
        self.ur_moveit.max_velocity     = UR_VELOCITY
        self.ur_moveit.max_acceleration = UR_ACCEL

        self.get_logger().info("URExecutor ready.")


# ---------------------------------------------------------------------- #
#  Helpers                                                                 #
# ---------------------------------------------------------------------- #
def load_json(path):
    with open(path) as f:
        return json.load(f)


def parse_ur_waypoints(d):
    return [dict(zip(t["joint_names"], t["points"][-1]["positions"]))
            for t in [d[k] for k in sorted(d)]]


# ---------------------------------------------------------------------- #
#  Main                                                                    #
# ---------------------------------------------------------------------- #
def main():
    rclpy.init()
    node = URExecutor()

    ur_file = "real_ur_trajectories.json"
    if not os.path.exists(ur_file):
        node.get_logger().error(f"JSON file not found: {ur_file}")
        rclpy.shutdown()
        return

    ur_wps = parse_ur_waypoints(load_json(ur_file))
    node.get_logger().info(f"Loaded {len(ur_wps)} UR waypoints.")

    # ROS executor on a background thread
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()

    time.sleep(3.0)  # wait for action servers

    # UR10e runs on THIS (main) thread — safe for MoveIt action clients
    joint_order = node.ur_moveit.joint_names
    loop = 0
    try:
        while rclpy.ok():
            loop += 1
            node.get_logger().info(f"[UR] === Loop {loop} ===")
            for i, wp in enumerate(ur_wps):
                positions = [wp.get(n, 0.0) for n in joint_order]
                node.get_logger().info(f"[UR] Step {i+1}/{len(ur_wps)}")
                node.ur_moveit.move_to_configuration(positions)
                node.ur_moveit.wait_until_executed()
                node.get_logger().info(f"[UR] Step {i+1} done.")
    except KeyboardInterrupt:
        pass

    node.get_logger().info("All done.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
