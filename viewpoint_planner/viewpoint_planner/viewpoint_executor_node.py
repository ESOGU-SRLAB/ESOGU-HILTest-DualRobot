import rclpy
from rclpy.node import Node
import json
import os
import time


class ViewpointExecutorNode(Node):
    """
    Executes the planned UR10e viewpoints.

    IMPORTANT: this is currently a MOCK executor. It does NOT send any goal
    to MoveIt2/pymoveit2 or to the AGV -- it only sleeps and logs what a real
    executor would do. No robot will actually move when this node runs. This
    is intentional for now (see project notes), but is called out loudly at
    startup and on every cycle so it is never mistaken for real motion.
    """
    def __init__(self):
        super().__init__('viewpoint_executor_node')

        self.declare_parameter('plan_file', '/home/cem/colcon_ws/src/viewpoint_planner/plans/viewpoint_plan.json')
        self.plan_file = self.get_parameter('plan_file').value

        self.get_logger().warning(
            "viewpoint_executor_node is running in MOCK mode: it will NOT send any real "
            "trajectory goals to the UR10e or the AGV, and will NOT trigger the SICK "
            "camera. It only simulates timing/logging of the execution sequence. Do not use "
            "this to validate real or simulated robot motion."
        )

        # Load the plan
        self.plan = self.load_plan()
        if self.plan is not None:
            n_ur = len(self.plan.get('ur_viewpoints', []))
            coverage = self.plan.get('coverage_achieved')
            coverage_str = f", coverage_achieved={coverage * 100:.1f}%" if coverage is not None else ""
            self.get_logger().info(
                f"Loaded plan from '{self.plan_file}': {n_ur} UR10e viewpoints{coverage_str}."
            )

        # In a complete implementation, this node would integrate with MoveIt2 (via pymoveit2 or action client)
        # to execute the UR10e trajectories. For this prototype, we simulate the execution flow.

        self.timer = self.create_timer(1.0, self.execution_loop)
        self.current_vp_index = 0
        self.is_executing = False
        self._start_time = time.monotonic()

    def load_plan(self):
        if not os.path.exists(self.plan_file):
            self.get_logger().error(
                f"Plan file not found: {self.plan_file}. Run viewpoint_planning.launch.py and call "
                "the ~/plan service first."
            )
            return None
        try:
            with open(self.plan_file, 'r') as f:
                plan = json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            self.get_logger().error(f"Failed to parse plan file '{self.plan_file}': {e}")
            return None

        if not plan.get('ur_viewpoints'):
            self.get_logger().error(
                f"Plan file '{self.plan_file}' contains zero UR10e viewpoints; nothing to execute."
            )
            return None
        return plan

    def execution_loop(self):
        if self.plan is None:
            return

        if self.current_vp_index >= len(self.plan['ur_viewpoints']):
            elapsed = time.monotonic() - self._start_time
            self.get_logger().info(
                f"Execution completed successfully! {len(self.plan['ur_viewpoints'])} viewpoints "
                f"'executed' (mock) in {elapsed:.1f}s."
            )
            self.timer.cancel()
            return

        if self.is_executing:
            self.get_logger().debug("Previous step still marked as executing, skipping this tick.")
            return

        self.is_executing = True

        ur_vp = self.plan['ur_viewpoints'][self.current_vp_index]
        self.get_logger().info(
            f"[MOCK] Executing viewpoint {self.current_vp_index + 1}/{len(self.plan['ur_viewpoints'])} "
            f"(id={ur_vp['id']})"
        )

        # Mock execution:
        # 1. Send goal to UR10e
        # 2. Wait for it to arrive
        # 3. Trigger SICK point cloud acquisition

        self.get_logger().info(f"  [MOCK] -> Moving UR10e to {ur_vp['id']} at pos {ur_vp['position']}")
        time.sleep(0.5)  # Simulate movement time

        self.get_logger().info("  [MOCK] -> Capturing point cloud from SICK sensor...")
        time.sleep(0.2)

        self.current_vp_index += 1
        self.is_executing = False


def main(args=None):
    rclpy.init(args=args)
    node = ViewpointExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
