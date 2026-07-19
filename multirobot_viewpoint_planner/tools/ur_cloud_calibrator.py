#!/usr/bin/env python3
"""
UR SICK point-cloud optical-frame calibrator (live sliders).

The real UR SICK cloud comes out rotated (~a few tens of degrees) because its
optical frame `ur10e_depth_optical_frame` is defined rpy="0 0 0" in ur_macro.xacro
-- i.e. NOT oriented for the real SICK's optical convention (the Kawasaki got a
calibrated `kawasaki_sick_optical_frame`; the UR never did).

This tool lets you dial in that rotation LIVE while watching the cloud in RViz:

  * it subscribes to the UR cloud (`input_topic`, default /sick_points, stamped
    in `parent_frame` = ur10e_depth_optical_frame),
  * republishes the SAME points under a new frame `child_frame`
    (= ur10e_sick_optical_frame) on `output_topic` (default /sick_points_cal),
  * broadcasts a live TF  parent_frame -> child_frame  whose rotation you drive
    with the sliders (degrees) + optional xyz nudge.

Workflow:
  1. Bring up the real cell (so /sick_points and TF ur10e_depth_optical_frame exist).
  2. python3 ur_cloud_calibrator.py
  3. In RViz: Fixed Frame = world, add PointCloud2 on /sick_points_cal
     (set its Reliability = Best Effort).
  4. Drag Roll/Pitch/Yaw until the cloud sits ON the chassis. (xyz only if needed.)
  5. Click "Print origin for xacro" -> copy the printed line.

Baking the result (two options; the print gives you the exact line):
  A) SAFE / like the Kawasaki -- add a SEPARATE frame, leave depth_optical_frame
     (used by IK) untouched. In ur_macro.xacro add:
        <joint name="${tf_prefix}ur_sick_optical_joint" type="fixed">
          <origin xyz="X Y Z" rpy="R P Y"/>
          <parent link="${tf_prefix}depth_optical_frame"/>
          <child  link="${tf_prefix}ur10e_sick_optical_frame"/>
        </joint>
        <link name="${tf_prefix}ur10e_sick_optical_frame"/>
     then stamp the real UR cloud there (sick node frame_id + pointcloud_transformer
     target_frame + executor real_ur_sensor_frame -> ur10e_sick_optical_frame).
  B) SIMPLE -- put the SAME rpy straight into `depth_optical_joint` in ur_macro.xacro
     (it is currently rpy="0 0 0"). Fixes the cloud but ALSO tilts the IK view axis,
     so only do this if you will re-generate the plan afterwards.

The slider rpy uses URDF/ROS convention (extrinsic xyz, fixed axes) so the printed
values paste directly into a URDF <origin rpy=...>.
"""
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation


class CloudCalibrator(Node):
    def __init__(self):
        super().__init__("ur_cloud_calibrator")
        self.declare_parameter("input_topic", "/sick_points")
        self.declare_parameter("output_topic", "/sick_points_cal")
        self.declare_parameter("parent_frame", "ur10e_depth_optical_frame")
        self.declare_parameter("child_frame", "ur10e_sick_optical_frame")

        self.in_topic = self.get_parameter("input_topic").value
        self.out_topic = self.get_parameter("output_topic").value
        self.parent = self.get_parameter("parent_frame").value
        self.child = self.get_parameter("child_frame").value

        # Live-adjusted correction (radians / metres). Plain floats: written by the
        # Tk thread, read by the ROS timer -- atomic enough for this use.
        self.roll = self.pitch = self.yaw = 0.0
        self.x = self.y = self.z = 0.0

        self._br = TransformBroadcaster(self)
        # Publish RELIABLE so RViz's default (reliable) PointCloud2 subscription is
        # compatible -- best-effort subscribers still receive too. (A best-effort
        # publisher is NOT sent to a reliable subscriber, which is why RViz saw nothing.)
        pub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(PointCloud2, self.out_topic, pub_qos)
        # Subscribe best-effort so we receive from the SICK regardless of its QoS.
        self._sub = self.create_subscription(
            PointCloud2, self.in_topic, self._cb, qos_profile_sensor_data)
        self.create_timer(0.02, self._broadcast)  # 50 Hz live TF
        self._got = False
        self.get_logger().info(
            f"Calibrator up: republishing {self.in_topic} -> {self.out_topic} "
            f"under frame '{self.child}', live TF {self.parent} -> {self.child}.")

    def _cb(self, msg: PointCloud2):
        if not self._got:
            self._got = True
            self.get_logger().info(f"Receiving cloud on {self.in_topic}. View {self.out_topic} in RViz.")
        msg.header.frame_id = self.child
        self._pub.publish(msg)

    def _broadcast(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = float(self.z)
        # 'xyz' (lowercase) = extrinsic = URDF rpy convention.
        q = Rotation.from_euler("xyz", [self.roll, self.pitch, self.yaw]).as_quat()
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self._br.sendTransform(t)

    def origin_line(self):
        return (f'<origin xyz="{self.x:.4f} {self.y:.4f} {self.z:.4f}" '
                f'rpy="{self.roll:.5f} {self.pitch:.5f} {self.yaw:.5f}"/>')


def run_gui(node):
    import tkinter as tk

    root = tk.Tk()
    root.title("UR SICK cloud calibrator")

    readout = tk.StringVar()

    def refresh():
        readout.set(
            f"rpy(deg) = {math.degrees(node.roll):+7.2f} {math.degrees(node.pitch):+7.2f} "
            f"{math.degrees(node.yaw):+7.2f}   |   xyz(m) = {node.x:+.3f} {node.y:+.3f} {node.z:+.3f}\n"
            f"rpy(rad) = {node.roll:+.5f} {node.pitch:+.5f} {node.yaw:+.5f}")

    def slider(label, setter, lo, hi, res):
        tk.Label(root, text=label).pack(anchor="w")

        def on(v):
            setter(float(v))
            refresh()
        s = tk.Scale(root, from_=lo, to=hi, resolution=res, orient=tk.HORIZONTAL,
                     length=460, command=on)
        s.set(0.0)
        s.pack(fill="x")
        return s

    slider("Roll (deg)",  lambda v: setattr(node, "roll",  math.radians(v)), -180, 180, 0.5)
    slider("Pitch (deg)", lambda v: setattr(node, "pitch", math.radians(v)), -180, 180, 0.5)
    slider("Yaw (deg)",   lambda v: setattr(node, "yaw",   math.radians(v)), -180, 180, 0.5)
    slider("X (m)", lambda v: setattr(node, "x", v), -0.30, 0.30, 0.002)
    slider("Y (m)", lambda v: setattr(node, "y", v), -0.30, 0.30, 0.002)
    slider("Z (m)", lambda v: setattr(node, "z", v), -0.30, 0.30, 0.002)

    tk.Label(root, textvariable=readout, justify="left", font=("monospace", 10)).pack(anchor="w", pady=6)

    def do_print():
        line = node.origin_line()
        node.get_logger().info("CALIBRATED origin (paste into URDF): " + line)
        readout.set(readout.get() + "\n" + line)

    tk.Button(root, text="Print origin for xacro", command=do_print).pack(fill="x", pady=4)
    refresh()
    root.protocol("WM_DELETE_WINDOW", lambda: (rclpy.shutdown(), root.destroy()))
    root.mainloop()


def main():
    rclpy.init()
    node = CloudCalibrator()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        run_gui(node)
    except Exception as e:  # tkinter missing / no display
        node.get_logger().error(
            f"GUI unavailable ({e}). Install python3-tk, or set the correction via params:\n"
            "  ros2 run rqt_reconfigure rqt_reconfigure  (not available here) OR edit the node.\n"
            "Falling back to spin; adjust roll/pitch/yaw attributes in code.")
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
