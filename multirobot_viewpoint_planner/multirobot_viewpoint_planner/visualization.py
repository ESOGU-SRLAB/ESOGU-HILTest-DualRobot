import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import json
import os


class MultiRobotViewpointVisualizer(Node):
    """Publishes the multi-robot inspection plan as RViz MarkerArray arrows.

    Same live visualizer as the solo viewpoint_planner, extended for TWO real
    arms. The multi-robot plan stores `ur_viewpoints` AND `kawasaki_viewpoints`
    (both real coverage viewpoints, each with its own IK), so -- unlike the solo
    plan where the Kawasaki entries were hidden mirror points -- BOTH arms are
    drawn by default. The two arms are told apart by colour family so it is
    obvious at a glance which arm covers what:

        UR10e     -> green (most new coverage) -> red (least)   [as the solo viz]
        Kawasaki  -> dark blue (most)          -> light blue (least)

    Params (flip to bring detail back / change the look):
        show_ur:=false               -> hide the UR arrows
        show_kawasaki:=false         -> hide the Kawasaki arrows
        color_by_contribution:=false -> solid per-arm colour (UR orange / Kawa blue)
        scale_by_contribution:=true  -> arrow length grows with new coverage
        label_top_n:=20              -> label the N most informative stops per arm
    """

    def __init__(self):
        super().__init__('multirobot_viewpoint_visualizer')
        self.marker_pub = self.create_publisher(MarkerArray, '~/viewpoint_markers', 10)

        self.declare_parameter('show_ur', True)
        self.declare_parameter('show_kawasaki', True)
        self.declare_parameter('color_by_contribution', True)
        self.declare_parameter('scale_by_contribution', False)
        self.declare_parameter('label_top_n', 0)   # 0 = no text labels
        self.declare_parameter('plan_file',
                               '/home/ifarlab/colcon_ws/src/multirobot_viewpoint_planner/plans/multirobot_viewpoint_plan.json')

        self.timer = self.create_timer(2.0, self.timer_callback)
        self.plan_file = os.path.expanduser(self.get_parameter('plan_file').value)
        self._warned_missing_plan = False
        self._last_logged_marker_count = None
        self.get_logger().info(
            f"multirobot_viewpoint_visualizer started, watching '{self.plan_file}' every 2.0s.")

    # -- Per-arm contribution colours ---------------------------------------
    @staticmethod
    def _ur_color(new_points, max_points):
        """Green (most new points) -> red (least). Solid green if no metadata."""
        if not max_points or new_points is None:
            return [0.0, 1.0, 0.0]
        t = max(0.0, min(1.0, float(new_points) / float(max_points)))
        return [1.0 - t, t, 0.0]

    @staticmethod
    def _kawa_color(new_points, max_points):
        """Dark blue (most new points) -> light blue (least). Solid blue if none."""
        if not max_points or new_points is None:
            return [0.1, 0.45, 1.0]
        t = max(0.0, min(1.0, float(new_points) / float(max_points)))
        # t=1 (most) -> deep blue (0,0.2,0.8); t=0 (least) -> pale blue (0.7,0.85,1)
        return [0.7 * (1.0 - t), 0.85 - 0.65 * t, 1.0 - 0.2 * t]

    def create_text_marker(self, marker_id, pos, text, ns):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(pos[0])
        marker.pose.position.y = float(pos[1])
        marker.pose.position.z = float(pos[2]) + 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.04  # text height
        marker.color.r = marker.color.g = marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = text
        return marker

    def create_arrow_marker(self, marker_id, pos, rot_mat, color, ns, arrow_len=0.2):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Arrow points from the camera along its line of sight (+Z optical axis).
        z_axis = rot_mat[:, 2]

        p_start = Point()
        p_start.x = float(pos[0])
        p_start.y = float(pos[1])
        p_start.z = float(pos[2])

        p_end = Point()
        p_end.x = float(pos[0] + z_axis[0] * arrow_len)
        p_end.y = float(pos[1] + z_axis[1] * arrow_len)
        p_end.z = float(pos[2] + z_axis[2] * arrow_len)

        marker.points = [p_start, p_end]

        marker.scale.x = 0.01  # shaft diameter
        marker.scale.y = 0.03  # head diameter
        marker.scale.z = 0.04  # head length

        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0
        return marker

    def _add_arm_markers(self, marker_array, vps, base_id, color_fn, solid_color,
                         arrow_ns, label_ns, use_contrib, scale_contrib, label_top_n):
        """Append one arm's arrows (+ optional labels) to marker_array."""
        contribs = [vp.get('new_points_covered') for vp in vps
                    if vp.get('new_points_covered') is not None]
        max_contrib = max(contribs) if contribs else 0

        for i, vp in enumerate(vps):
            pos = np.array(vp['position'])
            rot = np.array(vp['rotation'])
            npc = vp.get('new_points_covered')
            rank = vp.get('rank', i)

            color = color_fn(npc, max_contrib) if use_contrib else solid_color

            arrow_len = 0.2
            if scale_contrib and max_contrib and npc is not None:
                arrow_len = 0.12 + 0.18 * (float(npc) / float(max_contrib))

            marker_array.markers.append(
                self.create_arrow_marker(base_id + i, pos, rot, color=color,
                                         ns=arrow_ns, arrow_len=arrow_len))

            if label_top_n and rank is not None and rank < label_top_n:
                label = f"#{rank}" + (f" (+{npc})" if npc is not None else "")
                marker_array.markers.append(
                    self.create_text_marker(base_id + i, pos, label, ns=label_ns))
        return max_contrib

    def timer_callback(self):
        if not os.path.exists(self.plan_file):
            if not self._warned_missing_plan:
                self.get_logger().warning(
                    f"Plan file '{self.plan_file}' does not exist yet -- no markers will be "
                    "published until a plan is generated (call multirobot_planner_node ~/plan).")
                self._warned_missing_plan = True
            return
        self._warned_missing_plan = False

        try:
            with open(self.plan_file, 'r') as f:
                plan = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Could not load plan '{self.plan_file}': {e}")
            return

        show_ur = self.get_parameter('show_ur').value
        show_kawasaki = self.get_parameter('show_kawasaki').value
        color_by_contribution = self.get_parameter('color_by_contribution').value
        scale_by_contribution = self.get_parameter('scale_by_contribution').value
        label_top_n = self.get_parameter('label_top_n').value

        ur_vps = plan.get('ur_viewpoints', [])
        kawa_vps = plan.get('kawasaki_viewpoints', [])

        marker_array = MarkerArray()
        # Clear stale markers first so a smaller re-plan doesn't leave ghosts.
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        if show_ur:
            self._add_arm_markers(
                marker_array, ur_vps, base_id=0,
                color_fn=self._ur_color, solid_color=[1.0, 0.5, 0.0],
                arrow_ns="ur_viewpoints", label_ns="ur_labels",
                use_contrib=color_by_contribution, scale_contrib=scale_by_contribution,
                label_top_n=label_top_n)

        if show_kawasaki:
            # Offset ids so UR and Kawasaki markers never collide.
            self._add_arm_markers(
                marker_array, kawa_vps, base_id=100000,
                color_fn=self._kawa_color, solid_color=[0.1, 0.45, 1.0],
                arrow_ns="kawasaki_viewpoints", label_ns="kawasaki_labels",
                use_contrib=color_by_contribution, scale_contrib=scale_by_contribution,
                label_top_n=label_top_n)

        self.marker_pub.publish(marker_array)

        if len(marker_array.markers) != self._last_logged_marker_count:
            n_ur = len(ur_vps) if show_ur else 0
            n_kawa = len(kawa_vps) if show_kawasaki else 0
            self.get_logger().info(
                f"Published {n_ur} UR + {n_kawa} Kawasaki waypoint arrows"
                + (f", labels for top {label_top_n}/arm" if label_top_n else "") + ".")
            self._last_logged_marker_count = len(marker_array.markers)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotViewpointVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
