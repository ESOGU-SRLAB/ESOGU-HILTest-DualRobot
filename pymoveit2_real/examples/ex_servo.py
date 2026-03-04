#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class TcpPathLengthTF(Node):
    def __init__(self):
        super().__init__('tcp_path_length_tf')

        # ---- Parameters ----
        self.declare_parameter('base_frame', 'world')   # kaynak frame
        self.declare_parameter('tcp_frame', 'ur10e_tool0')        # hedef frame (TCP/EE)
        self.declare_parameter('sample_hz', 10.0)           # örnekleme frekansı
        self.declare_parameter('max_jump', 0.0)             # (ops) zıplama filtresi; 0=kapalı
        self.declare_parameter('log_hz', 1.0)               # periyodik log

        self.base_frame = self.get_parameter('base_frame').value
        self.tcp_frame = self.get_parameter('tcp_frame').value
        self.sample_hz = float(self.get_parameter('sample_hz').value)
        self.max_jump = float(self.get_parameter('max_jump').value)
        self.log_hz = float(self.get_parameter('log_hz').value)

        if self.sample_hz <= 0:
            raise ValueError("sample_hz must be > 0")

        # ---- TF setup ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- State ----
        self.prev_point = None
        self.total = 0.0
        self.started = self.get_clock().now()

        # ---- Timers ----
        self.sample_timer = self.create_timer(1.0 / self.sample_hz, self.sample_once)
        if self.log_hz > 0:
            self.log_timer = self.create_timer(1.0 / self.log_hz, self.log_status)

        self.get_logger().info(
            f"Tracking TCP path via TF: '{self.base_frame}' -> '{self.tcp_frame}' at {self.sample_hz} Hz. "
            f"Stop with Ctrl+C to print total distance."
        )

    def _get_tcp_point(self):
        """
        Returns current TCP position (x,y,z) in base_frame coordinates, from TF.
        Uses latest available transform (time=0).
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tcp_frame,
                Time()  # time=0 => latest
            )
            t = tf.transform.translation
            return (float(t.x), float(t.y), float(t.z))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # TF henüz hazır değilse sessizce geç
            return None

    def sample_once(self):
        cur = self._get_tcp_point()
        if cur is None:
            return

        if self.prev_point is None:
            self.prev_point = cur
            return

        dx = cur[0] - self.prev_point[0]
        dy = cur[1] - self.prev_point[1]
        dz = cur[2] - self.prev_point[2]
        d = math.sqrt(dx*dx + dy*dy + dz*dz)

        # opsiyonel zıplama filtresi (TF reset / outlier)
        if self.max_jump > 0.0 and d > self.max_jump:
            self.get_logger().warn(f"Jump detected ({d:.3f} m) > max_jump; ignoring this step.")
            self.prev_point = cur
            return

        self.total += d
        self.prev_point = cur

    def log_status(self):
        elapsed = (self.get_clock().now() - self.started).nanoseconds / 1e9
        v_avg = self.total / elapsed if elapsed > 0 else 0.0
        self.get_logger().info(f"Total={self.total:.4f} m | elapsed={elapsed:.2f} s | avg={v_avg:.4f} m/s")

    def print_final(self):
        elapsed = (self.get_clock().now() - self.started).nanoseconds / 1e9
        v_avg = self.total / elapsed if elapsed > 0 else 0.0
        self.get_logger().info("========== FINAL TCP PATH LENGTH (TF) ==========")
        self.get_logger().info(f"Base frame     : {self.base_frame}")
        self.get_logger().info(f"TCP frame      : {self.tcp_frame}")
        self.get_logger().info(f"Total distance : {self.total:.6f} m")
        self.get_logger().info(f"Elapsed time   : {elapsed:.3f} s")
        self.get_logger().info(f"Avg speed      : {v_avg:.6f} m/s")
        self.get_logger().info("===============================================")


def main():
    rclpy.init()
    node = TcpPathLengthTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_final()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()