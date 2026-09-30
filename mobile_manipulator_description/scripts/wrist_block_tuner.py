#!/usr/bin/env python3
"""Live position tuner for the wrist underside block on Kawasaki link7.

Publishes a draggable interactive marker shaped exactly like the block declared in
rs005l_macro.xacro, in link7's frame. Drag it, and the node prints the three xacro
properties to paste back into that file, plus the world centre.

The block's whole difficulty is that link7's own Montaj mesh is large and hollow, so
"is it buried?" cannot be judged by eye. Every report therefore also states how many of
the block's 8 corners are INSIDE link7's mesh and inside link6's flange, which is the
thing to drive to 0/8 and 0/8. That check needs trimesh; without it the tuner still
works and simply omits those lines.

Run (nothing needs installing, rclpy is on the path):

    python3 src/mobile_manipulator_description/scripts/wrist_block_tuner.py

Then in RViz: Add -> InteractiveMarkers, topic /wrist_block_tuner/update.
The URDF's own copy of the block stays visible, so you can compare the two directly.

Options:
    --xyz 0.016382 -0.018751 0.009    starting offset in link7's frame
    --rpy 0 0 -157.700273             DEGREES; the default cancels link7's yaw so the
                                      block's edges land on the world axes
    --size 0.025 0.05 0.02            world X x Y x Z extents at the home pose
    --parent link7 / --prefix sim_kawasaki_
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

SHARE = ("/home/cem/colcon_ws/install/mobile_manipulator_description/share/"
         "mobile_manipulator_description/meshes/khi_rs/")
# link name -> (mesh file, scale, origin xyz, origin rpy) exactly as rs005l_macro.xacro
# declares its COLLISION geometry. Kept here rather than parsed from the URDF so the
# tuner runs even when no robot_description is up.
NEIGHBOURS = {
    "link7": ("Montaj.stl", 0.001, (-0.035, -0.04, 0.2175), (1.570796, 3.1415, 0.0)),
    "link6": ("RS005L_J6.STL", 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
}


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


def q_rot(q, v):
    x, y, z, w = q
    vq = q_mul(q_mul(q, (v[0], v[1], v[2], 0.0)), (-x, -y, -z, w))
    return (vq[0], vq[1], vq[2])


def _tri_box_overlap(V, h):
    """Separating-axis test, vectorised. V: (N,3,3) triangle vertices in the box's frame
    (box axis aligned, centred at the origin, half-extents h). Returns (N,) bool.

    Exact, and defined on an OPEN surface, which point-in-mesh is not. 13 axes: the
    box's 3, the triangle's normal, and the 9 edge cross products."""
    import numpy as np
    v0, v1, v2 = V[:, 0], V[:, 1], V[:, 2]
    sep = np.zeros(len(V), bool)
    for i in range(3):
        mn = np.minimum(np.minimum(v0[:, i], v1[:, i]), v2[:, i])
        mx = np.maximum(np.maximum(v0[:, i], v1[:, i]), v2[:, i])
        sep |= (mn > h[i]) | (mx < -h[i])
    n = np.cross(v1 - v0, v2 - v0)
    sep |= np.abs(np.einsum('ij,ij->i', n, v0)) > (np.abs(n) @ h)
    for e in (v1 - v0, v2 - v1, v0 - v2):
        for j in range(3):
            u = np.zeros(3)
            u[j] = 1.0
            a = np.cross(u, e)
            p = np.einsum('nij,nj->ni', V, a)
            r = np.abs(a) @ h
            sep |= (p.min(1) > r) | (p.max(1) < -r)
    return ~sep


def tidy(v, eps=1e-6):
    """Snap values that are a hair off a multiple of pi/2, so the printed xacro is clean."""
    for k in range(-4, 5):
        t = k * math.pi / 2
        if abs(v - t) < eps:
            return t
    return v


class WristBlockTuner(Node):

    def __init__(self, args):
        super().__init__("wrist_block_tuner")
        self.parent_link = args.parent          # unprefixed, keys NEIGHBOURS
        self.parent = args.prefix + args.parent
        self.prefix = args.prefix
        self.world = args.world
        self.size = tuple(args.size)
        self._start_xyz = tuple(args.xyz)
        self._start_rpy = tuple(args.rpy)
        self.pose = Pose()
        self.pose.position.x, self.pose.position.y, self.pose.position.z = args.xyz
        q = q_from_rpy(*[math.radians(a) for a in args.rpy])
        (self.pose.orientation.x, self.pose.orientation.y,
         self.pose.orientation.z, self.pose.orientation.w) = q

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.meshes = self._load_meshes()

        self.server = InteractiveMarkerServer(self, "wrist_block_tuner")
        self.menu = MenuHandler()
        self._build_menu()
        self._publish()
        self.report("start")

    # --- neighbour meshes, for the "is it buried?" check --------------------------------

    def _load_meshes(self):
        """{link name: (triangles, surface samples) in THAT LINK's frame}.

        Empty if trimesh is unavailable. Triangles drive the exact hit test; the samples
        give the clearance number when there is no hit.
        """
        try:
            import numpy as np
            import trimesh
        except ImportError:
            self.get_logger().warning(
                "trimesh/numpy not importable: the tuner will still drag and print, but "
                "cannot tell you whether the block is inside link7 or link6.")
            return {}
        out = {}
        for link, (fname, scale, xyz, rpy) in NEIGHBOURS.items():
            try:
                m = trimesh.load(SHARE + fname, force="mesh")
            except Exception as e:
                self.get_logger().warning(f"could not load {fname}: {e}")
                continue
            m.apply_scale(scale)
            T = np.eye(4)
            T[:3, :3] = trimesh.transformations.euler_matrix(*rpy)[:3, :3]
            T[:3, 3] = xyz
            m.apply_transform(T)
            # NOTE no watertight requirement and no `contains`: these STLs are open
            # shells, so point-in-mesh is undefined and the convex hull (1.94x the real
            # volume on Montaj.stl) over-reports badly. Box-vs-triangle overlap is exact
            # on an open shell AND is what MoveIt's FCL mesh-box check actually tests.
            n = max(20000, min(120000, len(m.faces) * 4))
            pts = np.vstack([np.asarray(trimesh.sample.sample_surface(m, n)[0]),
                             m.vertices])
            out[link] = (np.asarray(m.triangles), pts,
                         float(np.sqrt(m.area / n)))
        return out

    def _overlap(self, link):
        """(hits: bool, clearance_m: float, sample_spacing_m: float) or None.

        clearance is 0.0 when the block cuts the shell; otherwise it is the distance
        from the block to the nearest sampled surface point, good to ~spacing.
        """
        if link not in self.meshes:
            return None
        import numpy as np
        tris, pts, spacing = self.meshes[link]
        h = np.array(self.size) / 2.0
        q = (self.pose.orientation.x, self.pose.orientation.y,
             self.pose.orientation.z, self.pose.orientation.w)
        p = np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])

        if link != self.parent_link:
            T = self._tf(self.prefix + link, self.parent)   # parent -> link
            if T is None:
                return None
            tq, tt = T
            q = q_mul(tq, q)
            p = np.array(q_rot(tq, p)) + np.array(tt)

        # world/link -> block frame: R^T (x - p). R from the (possibly composed) quat.
        R = np.array([q_rot(q, (1, 0, 0)), q_rot(q, (0, 1, 0)), q_rot(q, (0, 0, 1))]).T
        V = (tris - p) @ R                      # (N,3,3), block frame
        hits = bool(_tri_box_overlap(V, h).any())
        if hits:
            return True, 0.0, spacing
        local = (pts - p) @ R
        d = np.abs(local) - h
        outside = np.linalg.norm(np.maximum(d, 0.0), axis=1)
        inside = np.minimum(np.max(d, axis=1), 0.0)
        return False, float(np.min(outside + inside)), spacing

    def _tf(self, target, source):
        """(quat xyzw, translation) of `source` expressed in `target`, or None."""
        try:
            t = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time()).transform
        except Exception:
            return None
        return ((t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                (t.translation.x, t.translation.y, t.translation.z))

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
        self.menu.insert("Basa don", callback=self._reset)

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
        self.pose.position.x, self.pose.position.y, self.pose.position.z = self._start_xyz
        q = q_from_rpy(*[math.radians(a) for a in self._start_rpy])
        (self.pose.orientation.x, self.pose.orientation.y,
         self.pose.orientation.z, self.pose.orientation.w) = q
        self._publish()
        self.report("reset")

    def _publish(self):
        im = InteractiveMarker()
        im.header.frame_id = self.parent
        im.name = "wrist_block"
        im.description = ""
        im.scale = 0.12          # the block is small; keep the handles close to it
        im.pose = self.pose

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x, box.scale.y, box.scale.z = self.size
        box.pose.orientation.w = 1.0
        box.color.r, box.color.g, box.color.b, box.color.a = 1.0, 0.35, 0.0, 0.9

        body = InteractiveMarkerControl()
        body.always_visible = True
        body.interaction_mode = InteractiveMarkerControl.MENU
        body.markers.append(box)
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
            "  rs005l_macro.xacro icin:",
            f'    <xacro:property name="wrist_block_x" value="{p.x:.6f}"/>',
            f'    <xacro:property name="wrist_block_y" value="{p.y:.6f}"/>',
            f'    <xacro:property name="wrist_block_z" value="{p.z:.6f}"/>',
            f'    <xacro:property name="wrist_block_yaw" value="{y:.6f}"/>',
            f'    <xacro:property name="wrist_block_size" '
            f'value="{self.size[0]} {self.size[1]} {self.size[2]}"/>',
            f"  rpy deg      = ({math.degrees(r):7.2f}, {math.degrees(pi_):7.2f}, "
            f"{math.degrees(y):7.2f})",
            f"  parent frame = {self.parent}",
        ]
        # roll/pitch are not expressible by wrist_block_yaw alone; say so rather than
        # printing a property line that would silently drop them.
        if abs(r) > 1e-6 or abs(pi_) > 1e-6:
            lines.append("  ** roll/pitch sifir degil: yukaridaki yaw tek basina bu "
                         "duruşu ifade etmiyor, tam origin satirini kullan:")
            lines.append(f'    <origin xyz="{p.x:.6f} {p.y:.6f} {p.z:.6f}" '
                         f'rpy="{r:.6f} {pi_:.6f} {y:.6f}"/>')

        w = self._world_centre()
        if w is not None:
            lines.append(f"  world centre = ({w[0]:.4f}, {w[1]:.4f}, {w[2]:.4f})")
        else:
            lines.append(f"  world centre = <no TF {self.world} -> {self.parent} yet>")

        for link in ("link7", "link6"):
            got = self._overlap(link)
            if got is None:
                lines.append(f"  {link:6s} : <TF yok / mesh yok>")
                continue
            hits, clear, spacing = got
            if hits:
                lines.append(f"  {link:6s} : ICINDE  <-- kutu bu mesh i kesiyor")
            else:
                lines.append(f"  {link:6s} : disarida, en yakin yuzey "
                             f"{clear * 1000:.1f} mm (+/- {spacing * 1000:.1f})")

        text = "\n".join(lines)
        if rclpy.ok():
            self.get_logger().info(text)
        else:
            print(text, flush=True)  # on shutdown rosout is already gone

    def _world_centre(self):
        T = self._tf(self.world, self.parent)
        if T is None:
            return None
        q, t = T
        r = q_rot(q, (self.pose.position.x, self.pose.position.y, self.pose.position.z))
        return (t[0] + r[0], t[1] + r[1], t[2] + r[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--parent", default="link7")
    ap.add_argument("--prefix", default="")
    ap.add_argument("--world", default="world")
    ap.add_argument("--xyz", nargs=3, type=float,
                    default=[0.016382, -0.018751, 0.009])
    ap.add_argument("--rpy", nargs=3, type=float, default=[0.0, 0.0, -157.700273],
                    help="DEGREES")
    ap.add_argument("--size", nargs=3, type=float, default=[0.025, 0.05, 0.02],
                    help="block extents; at the default rpy these are world X, Y, Z")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = WristBlockTuner(args)
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
