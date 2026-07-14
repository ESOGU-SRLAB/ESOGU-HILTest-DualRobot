import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from scipy.spatial.transform import Rotation

# Human-readable names for the most common MoveItErrorCodes returned by
# /compute_ik, so failure logs explain *why* IK failed (no solution vs.
# collision vs. timeout) instead of just printing a raw integer.
IK_ERROR_NAMES = {
    1: 'SUCCESS',
    -31: 'NO_IK_SOLUTION',
    -12: 'GOAL_IN_COLLISION',
    -13: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
    -14: 'GOAL_CONSTRAINTS_VIOLATED',
    -10: 'START_STATE_IN_COLLISION',
    -9: 'TIMED_OUT',
    -17: 'FRAME_TRANSFORM_FAILURE',
    -18: 'COLLISION_CHECKING_UNAVAILABLE',
    -19: 'ROBOT_STATE_STALE',
    -21: 'INVALID_GROUP_NAME',
    -22: 'INVALID_GOAL_CONSTRAINTS',
    -23: 'INVALID_ROBOT_STATE',
    -24: 'INVALID_LINK_NAME',
}


class ReachabilityChecker(Node):
    """
    Checks if a given viewpoint is reachable by the robot.
    """
    def __init__(self, group_name="real_ur10e", base_frame="world", tool_frame="ur10e_depth_optical_frame",
                 avoid_collisions=True):
        super().__init__(f'reachability_checker_{group_name}')
        self.group_name = group_name
        self.base_frame = base_frame
        self.tool_frame = tool_frame
        # When True, IK only succeeds if the whole arm is collision-free at the
        # pose. For close-range inspection of a cage-like chassis this rejects
        # many poses where the ARM body would enter the structure even though the
        # camera pose itself is kinematically reachable. Set False to measure
        # pure kinematic reachability (diagnostic / when collision geometry is
        # not yet trustworthy).
        self.avoid_collisions = avoid_collisions

        # Accumulates {error_code: count} across all check_ik calls so callers
        # can log a breakdown of *why* IK failed (see log_error_breakdown).
        self.ik_error_counts = {}

        self.get_logger().info(
            f"Initializing ReachabilityChecker: group='{group_name}', base_frame='{base_frame}', "
            f"tool_frame(ik_link)='{tool_frame}'."
        )
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

    def wait_for_service(self, timeout_sec=5.0):
        available = self.ik_client.wait_for_service(timeout_sec=timeout_sec)
        if not available:
            self.get_logger().error(
                f"/compute_ik service did not become available within {timeout_sec}s "
                f"(group='{self.group_name}'). Is move_group running?"
            )
        return available

    def check_ik(self, position, orientation, timeout=0.1):
        """
        Checks if the given pose (position, orientation) is reachable.
        orientation should be a quaternion [x, y, z, w].
        """
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout - int(timeout)) * 1e9)
        req.ik_request.avoid_collisions = self.avoid_collisions
        # Solve for the actual sensor/tool frame rather than the group's default
        # tip link -- without this the request silently ignores tool_frame and
        # may solve IK for the wrong link on multi-tip groups.
        if self.tool_frame:
            req.ik_request.ik_link_name = self.tool_frame

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.pose.position.x = float(position[0])
        pose_stamped.pose.position.y = float(position[1])
        pose_stamped.pose.position.z = float(position[2])

        pose_stamped.pose.orientation.x = float(orientation[0])
        pose_stamped.pose.orientation.y = float(orientation[1])
        pose_stamped.pose.orientation.z = float(orientation[2])
        pose_stamped.pose.orientation.w = float(orientation[3])

        req.ik_request.pose_stamped = pose_stamped

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            self.get_logger().error(
                f"/compute_ik call returned no response (timed out or service crashed) for group '{self.group_name}'."
            )
            return False, None

        if response.error_code.val == response.error_code.SUCCESS:
            self.get_logger().debug(
                f"IK success for group '{self.group_name}' at position={list(position)}."
            )
            return True, response.solution.joint_state
        else:
            err = response.error_code.val
            self.ik_error_counts[err] = self.ik_error_counts.get(err, 0) + 1
            self.get_logger().debug(
                f"IK failed for group '{self.group_name}' at position={list(position)}: "
                f"error_code={err} ({IK_ERROR_NAMES.get(err, 'UNKNOWN')})."
            )
            return False, None

    def log_error_breakdown(self):
        """Logs a per-error-code summary of every IK failure seen so far.
        Helps diagnose low coverage: e.g. mostly NO_IK_SOLUTION means the arm
        physically cannot reach the poses, whereas GOAL_IN_COLLISION means the
        poses are reachable but blocked by the scene/self-collision."""
        if not self.ik_error_counts:
            return
        breakdown = ', '.join(
            f"{IK_ERROR_NAMES.get(code, 'UNKNOWN')}({code})={count}"
            for code, count in sorted(self.ik_error_counts.items(), key=lambda kv: -kv[1])
        )
        self.get_logger().info(
            f"IK failure breakdown for group '{self.group_name}': {breakdown}."
        )
        # If *every* failure is NO_IK_SOLUTION it is often a configuration
        # problem rather than genuine unreachability -- point the user at the
        # usual culprits so they don't chase viewpoint tuning in vain.
        if set(self.ik_error_counts) == {-31}:
            self.get_logger().warning(
                f"All IK failures for group '{self.group_name}' are NO_IK_SOLUTION(-31). "
                "If NONE succeed, this is usually a config issue, not the viewpoints: check "
                f"(1) a kinematics_solver is defined for '{self.group_name}' in kinematics.yaml "
                "(move_group logs 'No kinematics solver instantiated' otherwise), "
                f"(2) the group's base link is connected to base_frame '{self.base_frame}' in the "
                "TF tree, and (3) kinematics_solver_timeout is not too small (KDL often needs >=0.05s)."
            )

    def filter_reachable_viewpoints(self, viewpoints):
        """
        Filters out unreachable viewpoints and adds joint solutions.
        """
        reachable_viewpoints = []
        unreachable_count = 0

        if not self.wait_for_service():
            # Previously this silently returned all input viewpoints, marking
            # them as "reachable" without ever checking IK -- that hid real
            # unreachability problems. Fail loudly instead so callers can
            # decide how to handle it (e.g. abort planning) instead of
            # emitting a plan with unverified poses.
            self.get_logger().error(
                f"/compute_ik service not available for group '{self.group_name}'. "
                f"Refusing to mark {len(viewpoints)} viewpoints as reachable without verification."
            )
            raise RuntimeError(
                f"IK service unavailable for group '{self.group_name}'; cannot verify reachability."
            )

        self.get_logger().info(
            f"Checking IK reachability for {len(viewpoints)} viewpoints (group='{self.group_name}')..."
        )
        for i, vp in enumerate(viewpoints):
            pos = vp['position']
            rot_mat = vp['rotation']
            
            r = Rotation.from_matrix(rot_mat)
            quat = r.as_quat() # [x, y, z, w]
            
            success, joint_state = self.check_ik(pos, quat)
            
            if success:
                vp['joint_solution'] = joint_state
                reachable_viewpoints.append(vp)
            else:
                unreachable_count += 1
                
            if (i + 1) % 10 == 0:
                self.get_logger().info(f"Checked IK for {i+1} viewpoints...")
                
        total = len(viewpoints)
        unreachable_pct = 100 * unreachable_count / total if total else 0.0
        self.get_logger().info(
            f"IK filtering complete for group '{self.group_name}'. "
            f"{len(reachable_viewpoints)} reachable, {unreachable_count} unreachable ({unreachable_pct:.1f}%)."
        )
        self.log_error_breakdown()
        if unreachable_pct > 25.0:
            self.get_logger().warning(
                f"{unreachable_pct:.1f}% of candidate viewpoints were unreachable for group "
                f"'{self.group_name}'. This will reduce achievable coverage -- consider revisiting "
                "viewpoint_distances/tilt_variations or the robot's mounting position."
            )
        return reachable_viewpoints
