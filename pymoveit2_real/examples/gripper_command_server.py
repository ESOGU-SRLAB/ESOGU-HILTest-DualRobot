#!/usr/bin/env python3
"""
Gripper test/komut sunucusu.

human_robot_collaboration_scenario.py içindeki `move_gripper()` ile birebir
aynı yolu kullanır (gripper_controller'ın FollowJointTrajectory action'ı,
MoveIt atlanır); farkı, bunu terminalden çağırabileceğiniz bir action ve
birkaç servis olarak dışarı açmasıdır.

Terminalden kullanımı
---------------------
  # İstediğiniz konumu doğrudan gönderin (senaryodaki değerler: 0.02 / 0.003)
  ros2 action send_goal /gripper_command control_msgs/action/GripperCommand \
      "{command: {position: 0.02, max_effort: 0.0}}"

  # Hazır kısayollar (open_position / closed_position parametrelerini kullanır)
  ros2 service call /gripper/close std_srvs/srv/Trigger
  ros2 service call /gripper/open  std_srvs/srv/Trigger

  # Kapat-aç çevrimi yapıp ölçülen konumları raporlar (çalışıyor mu testi)
  ros2 service call /gripper/cycle std_srvs/srv/Trigger

Her komuttan sonra hedef konumla /joint_states'ten OKUNAN gerçek konum
karşılaştırılır; gripper gerçekten hareket etmediyse bu farktan anlaşılır.
"""

import math
import time
from threading import Lock

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GripperCommandServer(Node):
    def __init__(self):
        super().__init__("gripper_command_server")

        # --- Parametreler -------------------------------------------------
        # Varsayılanlar senaryo koduyla ve onrobot_2fg7 URDF limitleriyle uyumlu
        # (prismatic gripper_joint: lower=-0.001, upper=0.026).
        self.declare_parameter("gripper_joint", "ur10e_gripper_joint")
        self.declare_parameter("controller_action", "/gripper_controller/follow_joint_trajectory")
        self.declare_parameter("closed_position", 0.02)    # senaryo: vidayı tutar
        self.declare_parameter("open_position", 0.003)     # senaryo: vidayı bırakır
        self.declare_parameter("move_duration", 1.0)       # trajectory süresi (sn)
        self.declare_parameter("min_position", -0.001)     # URDF alt limiti
        self.declare_parameter("max_position", 0.026)      # URDF üst limiti
        self.declare_parameter("position_tolerance", 0.003)  # "ulaştı" kabul sınırı
        self.declare_parameter("settle_time", 0.3)         # ölçüm öncesi bekleme

        self.gripper_joint = self.get_parameter("gripper_joint").value
        self.controller_action = self.get_parameter("controller_action").value
        self.closed_position = float(self.get_parameter("closed_position").value)
        self.open_position = float(self.get_parameter("open_position").value)
        self.move_duration = float(self.get_parameter("move_duration").value)
        self.min_position = float(self.get_parameter("min_position").value)
        self.max_position = float(self.get_parameter("max_position").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.settle_time = float(self.get_parameter("settle_time").value)

        callback_group = ReentrantCallbackGroup()

        # --- Kontrolcüye giden action client (senaryodakiyle aynı) ---------
        self._traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.controller_action,
            callback_group=callback_group,
        )

        # --- /joint_states'ten gerçek konumu oku --------------------------
        self._joint_lock = Lock()
        self._measured_position = None
        self._joint_seen = False
        self.create_subscription(
            JointState, "/joint_states", self._joint_states_callback, 10,
            callback_group=callback_group,
        )

        # --- Dışarı açılan arayüzler --------------------------------------
        self._action_server = ActionServer(
            self,
            GripperCommand,
            "/gripper_command",
            execute_callback=self._execute_gripper_command,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger, "/gripper/close", self._close_callback, callback_group=callback_group
        )
        self.create_service(
            Trigger, "/gripper/open", self._open_callback, callback_group=callback_group
        )
        self.create_service(
            Trigger, "/gripper/cycle", self._cycle_callback, callback_group=callback_group
        )

        self.get_logger().info(
            f"Gripper komut sunucusu hazır.\n"
            f"  eklem            : {self.gripper_joint}\n"
            f"  kontrolcü action : {self.controller_action}\n"
            f"  kapalı / açık    : {self.closed_position} / {self.open_position}\n"
            f"  action           : /gripper_command (control_msgs/action/GripperCommand)\n"
            f"  servisler        : /gripper/close, /gripper/open, /gripper/cycle"
        )

    # ------------------------------------------------------------------
    def _joint_states_callback(self, msg: JointState):
        try:
            index = msg.name.index(self.gripper_joint)
        except ValueError:
            return
        if index < len(msg.position):
            with self._joint_lock:
                self._measured_position = float(msg.position[index])
                self._joint_seen = True

    def measured_position(self):
        with self._joint_lock:
            return self._measured_position

    # ------------------------------------------------------------------
    def _wait_for_future(self, future, timeout):
        """Node zaten executor tarafından spin ediliyor; sadece bekliyoruz.

        (spin_until_future_complete node'un sahipliğini executor'dan çalar,
        bu yüzden bilerek kullanılmıyor.)
        """
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline and rclpy.ok():
            time.sleep(0.01)
        return future.done()

    def move_gripper(self, position, timeout=15.0, feedback_cb=None):
        """
        Gripper eklemini doğrudan kontrolcüye gönderir (MoveIt atlanır).
        (success, message, measured_position) döner.
        """
        if not (self.min_position <= position <= self.max_position):
            message = (
                f"Hedef {position} eklem limitleri dışında "
                f"[{self.min_position}, {self.max_position}]"
            )
            self.get_logger().error(message)
            return False, message, self.measured_position()

        start_position = self.measured_position()
        self.get_logger().info(
            f"Gripper hareketi: hedef={position:.4f} "
            f"(başlangıç={'?' if start_position is None else f'{start_position:.4f}'})"
        )

        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            message = (
                f"'{self.controller_action}' action sunucusu yok! "
                "gripper_controller aktif mi? (ros2 control list_controllers)"
            )
            self.get_logger().error(message)
            return False, message, start_position

        trajectory = JointTrajectory()
        trajectory.joint_names = [self.gripper_joint]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.velocities = [0.0]
        point.accelerations = [0.0]
        seconds = int(self.move_duration)
        point.time_from_start = Duration(
            sec=seconds, nanosec=int((self.move_duration - seconds) * 1e9)
        )
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        goal_future = self._traj_client.send_goal_async(goal)
        if not self._wait_for_future(goal_future, timeout):
            message = "Kontrolcü goal isteğine yanıt vermedi (timeout)."
            self.get_logger().error(message)
            return False, message, self.measured_position()

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            message = "Gripper goal kontrolcü tarafından REDDEDİLDİ."
            self.get_logger().error(message)
            return False, message, self.measured_position()

        result_future = goal_handle.get_result_async()
        deadline = time.time() + timeout
        while not result_future.done() and time.time() < deadline and rclpy.ok():
            if feedback_cb is not None:
                feedback_cb(self.measured_position())
            time.sleep(0.05)

        if not result_future.done():
            message = "Gripper hareketi zaman aşımına uğradı."
            self.get_logger().error(message)
            return False, message, self.measured_position()

        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            message = (
                f"Kontrolcü hata döndürdü: error_code={result.error_code} "
                f"({result.error_string})"
            )
            self.get_logger().warn(message)
            return False, message, self.measured_position()

        # Kontrolcü "başarılı" dese de gerçekten kımıldadı mı, ölçerek bak
        time.sleep(self.settle_time)
        measured = self.measured_position()

        if measured is None:
            message = (
                f"Hareket tamamlandı ama '{self.gripper_joint}' /joint_states'te "
                "bulunamadı; gerçek konum doğrulanamıyor."
            )
            self.get_logger().warn(message)
            return True, message, None

        error = abs(measured - position)
        if error > self.position_tolerance:
            message = (
                f"Kontrolcü başarılı dedi ama ölçülen konum {measured:.4f}, "
                f"hedef {position:.4f} (fark {error:.4f} > tolerans "
                f"{self.position_tolerance}). Gripper takılmış olabilir."
            )
            self.get_logger().warn(message)
            return False, message, measured

        message = f"Gripper hedefe ulaştı: {measured:.4f} (hedef {position:.4f})"
        self.get_logger().info(message)
        return True, message, measured

    # ------------------------------------------------------------------
    def _execute_gripper_command(self, goal_handle):
        position = float(goal_handle.request.command.position)

        def publish_feedback(measured):
            feedback = GripperCommand.Feedback()
            feedback.position = measured if measured is not None else math.nan
            feedback.effort = 0.0
            feedback.stalled = False
            feedback.reached_goal = False
            goal_handle.publish_feedback(feedback)

        success, message, measured = self.move_gripper(position, feedback_cb=publish_feedback)

        result = GripperCommand.Result()
        result.position = measured if measured is not None else math.nan
        result.effort = 0.0
        result.reached_goal = success
        result.stalled = not success

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
            self.get_logger().error(f"Gripper komutu başarısız: {message}")
        return result

    # ------------------------------------------------------------------
    def _close_callback(self, request, response):
        success, message, _ = self.move_gripper(self.closed_position)
        response.success = success
        response.message = f"KAPAT ({self.closed_position}): {message}"
        return response

    def _open_callback(self, request, response):
        success, message, _ = self.move_gripper(self.open_position)
        response.success = success
        response.message = f"AÇ ({self.open_position}): {message}"
        return response

    def _cycle_callback(self, request, response):
        """Kapat -> aç çevrimi; senaryodaki iki komutun da çalıştığını gösterir."""
        self.get_logger().info("=== GRIPPER ÇEVRİM TESTİ ===")
        lines = []
        overall = True

        for label, target in (
            ("KAPAT", self.closed_position),
            ("AÇ", self.open_position),
        ):
            success, message, measured = self.move_gripper(target)
            overall = overall and success
            measured_text = "?" if measured is None else f"{measured:.4f}"
            lines.append(
                f"{label}: hedef={target:.4f} ölçülen={measured_text} "
                f"{'OK' if success else 'BAŞARISIZ'} -> {message}"
            )
            time.sleep(0.5)

        response.success = overall
        response.message = "\n".join(lines)
        self.get_logger().info(f"=== ÇEVRİM TESTİ {'BAŞARILI' if overall else 'BAŞARISIZ'} ===")
        return response


def main():
    rclpy.init()
    node = GripperCommandServer()
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
