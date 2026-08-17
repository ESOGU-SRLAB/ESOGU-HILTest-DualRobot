import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import json
import os

class ViewpointVisualizer(Node):
    """
    Publishes viewpoint markers to RViz for visualization.
    """
    def __init__(self):
        super().__init__('viewpoint_visualizer')
        self.marker_pub = self.create_publisher(MarkerArray, '~/viewpoint_markers', 10)

        # Default view is deliberately CLEAN: just the UR10e waypoint arrows the
        # robot will actually visit -- no text labels, uniform arrow size. Flip the
        # params below to bring extra detail back:
        #   label_top_n:=20              -> label the N most informative waypoints
        #   scale_by_contribution:=true  -> arrow length grows with new coverage
        #   color_by_contribution:=false -> single-colour arrows instead of the
        #                                   green(most)->red(least) gradient
        self.declare_parameter('show_ur_viewpoints', True)
        self.declare_parameter('color_by_contribution', True)
        self.declare_parameter('scale_by_contribution', False)
        self.declare_parameter('label_top_n', 0)   # 0 = no text labels

        self.timer = self.create_timer(2.0, self.timer_callback)
        self.plan_file = '/home/ifarlab/colcon_ws/src/viewpoint_planner/plans/viewpoint_plan.json'
        self._warned_missing_plan = False
        self._last_logged_marker_count = None
        self.get_logger().info(f"viewpoint_visualizer started, watching '{self.plan_file}' every 2.0s.")

    def contribution_color(self, new_points, max_points):
        """Green (most new points) -> red (least). Falls back to green when the
        plan has no contribution metadata (older plans)."""
        if not max_points or new_points is None:
            return [0.0, 1.0, 0.0]
        t = max(0.0, min(1.0, float(new_points) / float(max_points)))
        return [1.0 - t, t, 0.0]

    def create_text_marker(self, id, pos, text, ns="ur_labels"):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = id
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

    def create_arrow_marker(self, id, pos, rot_mat, color, ns="viewpoints", arrow_len=0.2):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Arrow points from camera towards the look direction (Z axis)
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

        marker.scale.x = 0.01 # shaft diameter
        marker.scale.y = 0.03 # head diameter
        marker.scale.z = 0.04 # head length

        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        return marker

    def timer_callback(self):
        if not os.path.exists(self.plan_file):
            if not self._warned_missing_plan:
                self.get_logger().warning(
                    f"Plan file '{self.plan_file}' does not exist yet -- no markers will be published "
                    "until a plan is generated (call the viewpoint_planner_node ~/plan service)."
                )
                self._warned_missing_plan = True
            return
        self._warned_missing_plan = False

        try:
            with open(self.plan_file, 'r') as f:
                plan = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Could not load plan '{self.plan_file}': {e}")
            return

        show_ur = self.get_parameter('show_ur_viewpoints').value
        color_by_contribution = self.get_parameter('color_by_contribution').value
        scale_by_contribution = self.get_parameter('scale_by_contribution').value
        label_top_n = self.get_parameter('label_top_n').value

        ur_vps = plan.get('ur_viewpoints', [])

        # Peak contribution, to normalize colour/scale across the plan.
        contribs = [vp.get('new_points_covered') for vp in ur_vps
                    if vp.get('new_points_covered') is not None]
        max_contrib = max(contribs) if contribs else 0

        marker_array = MarkerArray()
        # Clear stale markers first so a smaller re-plan (or a hidden namespace)
        # doesn't leave ghost arrows behind.
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        # UR10e viewpoints -- the actual waypoints the robot will visit.
        if show_ur:
            for i, vp in enumerate(ur_vps):
                pos = np.array(vp['position'])
                rot = np.array(vp['rotation'])
                npc = vp.get('new_points_covered')
                rank = vp.get('rank', i)

                if color_by_contribution:
                    color = self.contribution_color(npc, max_contrib)
                else:
                    color = [0.0, 1.0, 0.0]

                arrow_len = 0.2
                if scale_by_contribution and max_contrib and npc is not None:
                    arrow_len = 0.12 + 0.18 * (float(npc) / float(max_contrib))

                marker_array.markers.append(
                    self.create_arrow_marker(i, pos, rot, color=color,
                                             ns="ur_viewpoints", arrow_len=arrow_len))

                # Label only the most informative viewpoints to avoid clutter.
                if label_top_n and rank is not None and rank < label_top_n:
                    label = f"#{rank}" + (f" (+{npc})" if npc is not None else "")
                    marker_array.markers.append(
                        self.create_text_marker(i, pos, label, ns="ur_labels"))

        self.marker_pub.publish(marker_array)

        if len(marker_array.markers) != self._last_logged_marker_count:
            n_ur_shown = len(ur_vps) if show_ur else 0
            self.get_logger().info(
                f"Published {n_ur_shown} UR10e waypoint arrows"
                + (f", labels for top {label_top_n}" if label_top_n else "") + "."
            )
            self._last_logged_marker_count = len(marker_array.markers)

def main(args=None):
    rclpy.init(args=args)
    node = ViewpointVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
