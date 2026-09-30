#!/usr/bin/env python3
"""Live pose tuner for the Kawasaki cable channel cradle.

Publishes a draggable interactive marker carrying the kawa_cable_channel mesh, in the
frame of the link it is bolted to (link3 by default). Drag it, or right click it and use
the menu to snap 90 deg at a time, and the node prints the xacro line to paste into
rs005l_macro.xacro, along with the world position of the part centre.

Run (nothing needs installing, rclpy is on the path):

    python3 src/mobile_manipulator_description/scripts/cable_channel_tuner.py

Then in RViz: Add -> InteractiveMarkers, topic /cable_channel_tuner/update.
The URDF's own copy of the part stays visible, so you can compare the two directly.

Options:
    --parent link3            frame to mount in
    --xyz 0.137 -0.15804 0.0036
    --rpy 0 180 0             DEGREES, to match what the menu prints
    --prefix sim_kawasaki_    for the sim robot
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import Pose
from interactive_markers import InteractiveMarkerServer, MenuHandler
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

MESH = "package://mobile_manipulator_description/meshes/khi_rs/kawa_cable_channel.stl"
# The mesh still carries the UR assembly's CAD coordinates; this offset drops its
# bounding box centre onto the link origin. Same numbers as rs005l_macro.xacro.
MESH_OFFSET = (-1.079374, -0.486789, -0.638931)


# --- quaternion helpers, xyzw order throughout ---------------------------------------

def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def q_to_rpy(q):
    x, y, z, w = q
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def q_axis(axis, angle):
    s = math.sin(angle / 2)
    v = [0.0, 0.0, 0.0]
    v["xyz".index(axis)] = s
    return (v[0], v[1], v[2], math.cos(angle / 2))


def q_norm(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def tidy(v, eps=1e-6):
    """Snap values that are a hair off a multiple of pi/2, so the printed xacro is clean."""
    for k in range(-4, 5):
        t = k * math.pi / 2
        if abs(v - t) < eps:
            return t
    return v


class CableChannelTuner(Node):

    def __init__(self, args):
        super().__init__("cable_channel_tuner")
        self.parent = args.prefix + args.parent
        self.world = args.world
        self.pose = Pose()
        self.pose.position.x, self.pose.position.y, self.pose.position.z = args.xyz
        q = q_from_rpy(*[math.radians(a) for a in args.rpy])
        (self.pose.orientation.x, self.pose.orientation.y,
         self.pose.orientation.z, self.pose.orientation.w) = q

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.server = InteractiveMarkerServer(self, "cable_channel_tuner")
        self.menu = MenuHandler()
        self._build_menu()
        self._publish()
        self.report("start")

    # --- marker -----------------------------------------------------------------------

    def _build_menu(self):
        for label, frame in (("Dondur (WORLD ekseni)", "world"),
                             ("Dondur (PARCA ekseni)", "local")):
            top = self.menu.insert(label)
            for axis in "xyz":
                for deg in (90, -90):
                    self.menu.insert(
                        f"{axis.upper()} {deg:+d} deg", parent=top,
                        callback=self._make_rot_cb(frame, axis, math.radians(deg)))
        self.menu.insert("Degerleri yazdir", callback=lambda fb: self.report("print"))
        self.menu.insert("Sifirla (identity)", callback=self._reset)

    def _make_rot_cb(self, frame, axis, angle):
        def cb(_feedback):
            o = self.pose.orientation
            cur = (o.x, o.y, o.z, o.w)
            dq = q_axis(axis, angle)
            new = q_mul(dq, cur) if frame == "world" else q_mul(cur, dq)
            o.x, o.y, o.z, o.w = q_norm(new)
            self._publish()
            self.report(f"{frame} {axis.upper()} {math.degrees(angle):+.0f}")
        return cb

    def _reset(self, _feedback):
        o = self.pose.orientation
        o.x, o.y, o.z, o.w = 0.0, 0.0, 0.0, 1.0
        self._publish()
        self.report("reset")

    def _publish(self):
        im = InteractiveMarker()
        im.header.frame_id = self.parent
        im.name = "cable_channel"
        im.description = ""
        im.scale = 0.25
        im.pose = self.pose

        mesh = Marker()
        mesh.type = Marker.MESH_RESOURCE
        mesh.mesh_resource = MESH
        mesh.scale.x = mesh.scale.y = mesh.scale.z = 0.001
        mesh.pose.position.x, mesh.pose.position.y, mesh.pose.position.z = MESH_OFFSET
        mesh.pose.orientation.w = 1.0
        mesh.color.r, mesh.color.g, mesh.color.b, mesh.color.a = 1.0, 0.35, 0.0, 0.9

        body = InteractiveMarkerControl()
        body.always_visible = True
        body.interaction_mode = InteractiveMarkerControl.MENU
        body.markers.append(mesh)
        im.controls.append(body)

        # 3 rotate rings + 3 translate arrows, in the marker's own frame
        for axis, quat in (("x", (1.0, 0.0, 0.0, 1.0)),
                           ("y", (0.0, 0.0, 1.0, 1.0)),
                           ("z", (0.0, 1.0, 0.0, 1.0))):
            for mode, suffix in ((InteractiveMarkerControl.ROTATE_AXIS, "rot"),
                                 (InteractiveMarkerControl.MOVE_AXIS, "mov")):
                c = InteractiveMarkerControl()
                c.name = f"{suffix}_{axis}"
                qx, qy, qz, qw = q_norm(quat)
                c.orientation.x, c.orientation.y = qx, qy
                c.orientation.z, c.orientation.w = qz, qw
                c.interaction_mode = mode
                c.orientation_mode = InteractiveMarkerControl.FIXED
                im.controls.append(c)

        self.server.insert(im, feedback_callback=self._feedback)
        self.menu.apply(self.server, im.name)
        self.server.applyChanges()

    def _feedback(self, fb):
        if fb.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.pose = fb.pose
        elif fb.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.pose = fb.pose
            self.report("drag")

    # --- reporting --------------------------------------------------------------------

    def report(self, why):
        p, o = self.pose.position, self.pose.orientation
        r, pi_, y = [tidy(v) for v in q_to_rpy((o.x, o.y, o.z, o.w))]
        lines = [
            "",
            f"--- {why} " + "-" * (60 - len(why)),
            f'  <origin xyz="{p.x:.5f} {p.y:.5f} {p.z:.5f}" '
            f'rpy="{r:.6f} {pi_:.6f} {y:.6f}"/>',
            f"  rpy deg      = ({math.degrees(r):7.2f}, {math.degrees(pi_):7.2f}, "
            f"{math.degrees(y):7.2f})",
            # rpy is one of several valid ZYX decompositions, so e.g. (0, 180, 0) can come
            # back out as (180, 0, 180). Same rotation, and the xacro line above is always
            # correct; the quaternion is here as the unambiguous form.
            f"  quat xyzw    = ({o.x:+.6f}, {o.y:+.6f}, {o.z:+.6f}, {o.w:+.6f})",
            f"  parent frame = {self.parent}",
        ]
        w = self._world_centre()
        if w is not None:
            lines.append(f"  world centre = ({w[0]:.4f}, {w[1]:.4f}, {w[2]:.4f})")
        else:
            lines.append(f"  world centre = <no TF {self.world} -> {self.parent} yet>")
        text = "\n".join(lines)
        if rclpy.ok():
            self.get_logger().info(text)
        else:
            print(text, flush=True)  # on shutdown rosout is already gone

    def _world_centre(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.world, self.parent, rclpy.time.Time()).transform
        except Exception:
            return None
        q = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        x, y, z, w = q
        # rotate the marker's local position into the world frame
        px, py, pz = self.pose.position.x, self.pose.position.y, self.pose.position.z
        vq = q_mul(q_mul(q, (px, py, pz, 0.0)), (-x, -y, -z, w))
        return (t.translation.x + vq[0], t.translation.y + vq[1], t.translation.z + vq[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--parent", default="link3")
    ap.add_argument("--prefix", default="")
    ap.add_argument("--world", default="world")
    ap.add_argument("--xyz", nargs=3, type=float, default=[0.137, -0.15804, 0.0036])
    ap.add_argument("--rpy", nargs=3, type=float, default=[0.0, 180.0, 0.0],
                    help="DEGREES")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = CableChannelTuner(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        # SIGTERM tears the context down under spin(); nothing worth a traceback.
        if rclpy.ok():
            raise
    finally:
        node.report("final")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
