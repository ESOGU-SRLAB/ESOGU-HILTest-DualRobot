#!/usr/bin/env python3
"""
Find out why the controller's rail STATE disagrees with the adapter's measurement.

Run this WHILE the full system is up (it only listens, it commands nothing).

The adapter reports the rail at its true position, but the JointTrajectoryController
aborts with a ~0.5 m position error -- which is exactly the difference between the
commanded position and the URDF's `initial_value` of 1.0. That means the rail's
state interface in topic_based_ros2_control is not being updated from /joint_states.

This tells you where the chain breaks:

  1. Who publishes /joint_states, and does each publisher include the rail joint?
     Multiple publishers matter: topic_based_ros2_control keeps only the LAST
     message, so a high-rate publisher that echoes a stale rail value can
     overwrite the adapter's fresh one (and the joint_state_broadcaster echoes
     the very state interface we are trying to feed -- a self-latching loop).
  2. What position does the CONTROLLER actually see (controller_state.actual)?
     If that sits at 1.0 while /joint_states carries the real value, the break is
     inside topic_based_ros2_control, not in the adapter.
"""
import sys
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState

JOINT = sys.argv[1] if len(sys.argv) > 1 else "ur10e_base_to_robot_mount"
DURATION_S = 5.0
CONTROLLER_STATE = "/scaled_joint_trajectory_controller/controller_state"


class Checker(Node):
    def __init__(self):
        super().__init__("linear_axis_state_check")
        best_effort = QoSProfile(depth=50,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(JointState, "/joint_states", self.js_cb, best_effort)
        self.create_subscription(JointTrajectoryControllerState, CONTROLLER_STATE,
                                 self.cs_cb, best_effort)
        self.total = 0
        self.with_joint = 0
        self.values = []
        self.name_sets = defaultdict(int)
        self.ctrl_actual = []
        self.ctrl_desired = []
        self.ctrl_seen = 0

    def js_cb(self, msg):
        self.total += 1
        self.name_sets[tuple(sorted(msg.name))] += 1
        if JOINT in msg.name:
            self.with_joint += 1
            idx = msg.name.index(JOINT)
            if idx < len(msg.position):
                self.values.append(msg.position[idx])

    def cs_cb(self, msg):
        self.ctrl_seen += 1
        if JOINT not in msg.joint_names:
            return
        i = msg.joint_names.index(JOINT)
        if i < len(msg.actual.positions):
            self.ctrl_actual.append(msg.actual.positions[i])
        if i < len(msg.desired.positions):
            self.ctrl_desired.append(msg.desired.positions[i])


def main():
    rclpy.init()
    node = Checker()
    print(f"Listening for {DURATION_S:.0f} s on /joint_states and {CONTROLLER_STATE}")
    print(f"Joint under test: {JOINT}\n")

    end = node.get_clock().now().nanoseconds + int(DURATION_S * 1e9)
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("=" * 70)
    print("/joint_states publishers")
    print("=" * 70)
    try:
        infos = node.get_publishers_info_by_topic("/joint_states")
        for info in infos:
            print(f"  {info.node_namespace}/{info.node_name}".replace("//", "/"))
    except Exception as exc:
        print(f"  (could not query: {exc})")

    print()
    print("=" * 70)
    print("/joint_states content")
    print("=" * 70)
    rate = node.total / DURATION_S
    print(f"  {node.total} messages ({rate:.0f} Hz total)")
    print(f"  {node.with_joint} contained '{JOINT}' "
          f"({node.with_joint / DURATION_S:.0f} Hz)")
    if node.values:
        print(f"  value range: {min(node.values):.4f} .. {max(node.values):.4f} m")
        print(f"  last value : {node.values[-1]:.4f} m")
    print("\n  distinct joint-name sets seen (message count):")
    for names, count in sorted(node.name_sets.items(), key=lambda kv: -kv[1]):
        has = "HAS RAIL" if JOINT in names else "no rail "
        preview = ", ".join(names[:4]) + (" ..." if len(names) > 4 else "")
        print(f"    {count:6d}  [{has}] {len(names)} joints: {preview}")

    print()
    print("=" * 70)
    print("What the CONTROLLER sees")
    print("=" * 70)
    if not node.ctrl_seen:
        print(f"  no messages on {CONTROLLER_STATE}")
    elif not node.ctrl_actual:
        print(f"  '{JOINT}' not present in controller_state")
    else:
        print(f"  actual : {min(node.ctrl_actual):.4f} .. {max(node.ctrl_actual):.4f} m"
              f"   (last {node.ctrl_actual[-1]:.4f})")
        if node.ctrl_desired:
            print(f"  desired: {min(node.ctrl_desired):.4f} .. "
                  f"{max(node.ctrl_desired):.4f} m   (last {node.ctrl_desired[-1]:.4f})")

    print()
    print("=" * 70)
    print("VERDICT")
    print("=" * 70)
    if node.values and node.ctrl_actual:
        spread = max(node.ctrl_actual) - min(node.ctrl_actual)
        drift = abs(node.ctrl_actual[-1] - node.values[-1])
        if spread < 1e-4 and drift > 0.01:
            print(f"  The controller's state is FROZEN at {node.ctrl_actual[-1]:.4f} m")
            print(f"  while /joint_states reports {node.values[-1]:.4f} m.")
            print("  -> topic_based_ros2_control is not consuming the adapter's state.")
            print("     Suspects: sum_wrapped_joint_states on a prismatic joint, or a")
            print("     second /joint_states publisher overwriting the rail value.")
        elif drift < 0.01:
            print("  Controller state matches /joint_states -- the state path is fine.")
        else:
            print(f"  Controller state moves but lags /joint_states by {drift * 1000:.0f} mm.")
    else:
        print("  Not enough data -- is the full system running?")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
