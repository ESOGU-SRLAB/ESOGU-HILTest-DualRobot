#!/usr/bin/env python3
"""
Real world Dual Robot Executor - Independent Continuous Mode

UR10e runs on the main thread (safe for MoveIt action clients).
Kawasaki runs on a background thread (topic-based, no action clients).
"""

import json
import math
import os
import time
from threading import Thread, Lock

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String
from nav_msgs.msg import Odometry

from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as realrobot

LOOP_COUNT   = 10  # artık kullanılmıyor — sonsuz döngü aktif
UR_VELOCITY  = 0.2
UR_ACCEL     = 0.1


class RealDualExecutor(Node):
    def __init__(self):
        super().__init__("real_dual_executor")
        cb = ReentrantCallbackGroup()

        # --- UR10e (MoveIt) ---
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

        # --- Kawasaki (topic-based) ---
        self.agv_pub      = self.create_publisher(Float64,          "/agv/goal_position",  10)
        self.kawa_arm_pub = self.create_publisher(Float64MultiArray, "/kawa/target_joints", 10)

        self.agv_x       = 0.0
        self.agv_status  = "Idle"
        self.kawa_status = "Idle"
        self._lock = Lock()

        self.create_subscription(Odometry, "/agv/odom",
            lambda m: setattr(self, "agv_x", m.pose.pose.position.x),
            10, callback_group=cb)
        self.create_subscription(String, "/agv/mobile_status",
            lambda m: setattr(self, "agv_status", m.data),
            10, callback_group=cb)
        self.create_subscription(String, "/kawa/status",
            lambda m: setattr(self, "kawa_status", m.data),
            10, callback_group=cb)

        self.get_logger().info("RealDualExecutor ready.")

    # ------------------------------------------------------------------ #
    #  Kawasaki loop  (background thread, pure topic I/O)                 #
    # ------------------------------------------------------------------ #
    def run_kawasaki_loop(self, kawa_wps):
        loop = 0
        while rclpy.ok():
            loop += 1
            self.get_logger().info(f"[Kawa] === Loop {loop} ===")
            for i, (agv_target, arm_deg) in enumerate(kawa_wps):
                self.get_logger().info(
                    f"[Kawa] Step {i+1}/{len(kawa_wps)}: AGV→{agv_target:.2f}")

                agv_msg = Float64()
                agv_msg.data = agv_target
                self.agv_pub.publish(agv_msg)

                self.kawa_status = "Pending"
                arm_msg = Float64MultiArray()
                arm_msg.data = arm_deg
                self.kawa_arm_pub.publish(arm_msg)

                time.sleep(1.5)  # let C++ driver turn "Busy"

                agv_ok = arm_ok = False
                while not (agv_ok and arm_ok):
                    if not agv_ok and abs(self.agv_x - agv_target) <= 0.1 \
                            and self.agv_status == "Idle":
                        agv_ok = True
                    if not arm_ok:
                        if self.kawa_status == "Done":
                            arm_ok = True
                        elif "Error" in self.kawa_status:
                            self.get_logger().error(
                                f"[Kawa] ARM ERROR: {self.kawa_status}")
                            arm_ok = True
                    time.sleep(0.3)

                self.get_logger().info(f"[Kawa] Step {i+1} done.")
        self.get_logger().info("[Kawa] Sonsuz döngü sonlandı (rclpy kapatıldı).")


# ---------------------------------------------------------------------- #
#  Helpers                                                                 #
# ---------------------------------------------------------------------- #
def load_json(path):
    with open(path) as f:
        return json.load(f)


def parse_ur_waypoints(d):
    return [dict(zip(t["joint_names"], t["points"][-1]["positions"]))
            for t in [d[k] for k in sorted(d)]]


def parse_kawa_waypoints(d):
    wps = []
    for k in sorted(d):
        last = d[k]["points"][-1]["positions"]
        wps.append((last[0], [math.degrees(r) for r in last[1:7]]))
    return wps


# ---------------------------------------------------------------------- #
#  Main                                                                    #
# ---------------------------------------------------------------------- #
def main():
    rclpy.init()
    node = RealDualExecutor()

    ur_file   = "real_ur_trajectories.json"
    kawa_file = "real_kawa_trajectories.json"
    if not (os.path.exists(ur_file) and os.path.exists(kawa_file)):
        node.get_logger().error("JSON files not found.")
        rclpy.shutdown(); return

    ur_wps   = parse_ur_waypoints(load_json(ur_file))
    kawa_wps = parse_kawa_waypoints(load_json(kawa_file))
    node.get_logger().info(
        f"Loaded {len(ur_wps)} UR + {len(kawa_wps)} Kawa waypoints.")

    # ROS executor on a background thread
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()

    time.sleep(3.0)  # wait for action servers

    # Kawasaki runs on background thread (only publishes topics)
    t_kawa = Thread(target=node.run_kawasaki_loop, args=(kawa_wps,), daemon=True)
    t_kawa.start()

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

    t_kawa.join(timeout=5.0)
    node.get_logger().info("All done.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
