"""Optional, low-effort anti-creep hold for ERP42's Gazebo effort drive.

Ground-truth Gazebo odometry and four wheel torques are used only while there
are no other publishers on /wheel_effort/commands. The hold captures the
current pose after spawn settles, relinquishes control when another publisher
appears, and acquires a new hold point when that publisher disconnects.

It is a low-speed stationary hold, NOT a replacement for a safety brake, a
physical parking brake, or proper wheel contact modelling.
"""

from __future__ import annotations

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64MultiArray


class ERP42IdleHold(Node):
    def __init__(self):
        super().__init__("erp42_idle_hold")
        self.declare_parameter("kp_position", 20.0)  # N m / m per wheel
        self.declare_parameter("kd_velocity", 100.0)  # N m / (m/s) per wheel
        self.declare_parameter("max_effort", 4.0)  # N m per wheel; below normal 12 N m drive test
        self.declare_parameter("sample_timeout", 0.4)
        self.declare_parameter("nominal_axle_height", 0.28)
        self.declare_parameter("spawn_settle_tolerance", 0.012)

        self._latest = None
        self._received_at = 0.0
        self._hold_xy = None
        self._external_control = False
        self._last_torque = 0.0
        self._last_status = 0.0

        self._pub = self.create_publisher(Float64MultiArray, "/wheel_effort/commands", 10)
        self.create_subscription(Odometry, "/odom", self._receive, 10)
        self.create_timer(0.20, self._check_external_commanders)
        self.create_timer(0.05, self._apply_hold)
        self.get_logger().info(
            "ERP42 idle hold started: only commands while no other wheel-effort publisher exists"
        )

    def _receive(self, msg: Odometry) -> None:
        self._latest = msg
        self._received_at = time.monotonic()

    def _check_external_commanders(self) -> None:
        external = any(
            p.node_name != self.get_name() or p.node_namespace != self.get_namespace()
            for p in self.get_publishers_info_by_topic("/wheel_effort/commands")
        )
        if external != self._external_control:
            self._external_control = external
            self._hold_xy = None
            if external:
                self.get_logger().info("External wheel-effort publisher detected; idle hold yielding")
            else:
                self.get_logger().info("External wheel-effort publisher removed; reacquiring hold point")

    def _command(self, torque: float) -> None:
        msg = Float64MultiArray()
        msg.data = [float(torque)] * 4
        self._pub.publish(msg)
        self._last_torque = torque

    def _apply_hold(self) -> None:
        if self._external_control:
            return
        if self._latest is None:
            return
        if time.monotonic() - self._received_at > float(
            self.get_parameter("sample_timeout").value
        ):
            if self._last_torque:
                self._command(0.0)
            return

        pose = self._latest.pose.pose
        current_z = float(pose.position.z)
        nominal_z = float(self.get_parameter("nominal_axle_height").value)
        if abs(current_z - nominal_z) > float(
            self.get_parameter("spawn_settle_tolerance").value
        ):
            self._hold_xy = None
            return

        x, y = float(pose.position.x), float(pose.position.y)
        if self._hold_xy is None:
            self._hold_xy = (x, y)
            self.get_logger().info(f"Capturing idle hold point x={x:+.6f} y={y:+.6f}")

        quat = pose.orientation
        yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
        )
        dx = self._hold_xy[0] - x
        dy = self._hold_xy[1] - y
        forward_error = dx * math.cos(yaw) + dy * math.sin(yaw)
        velocity = float(self._latest.twist.twist.linear.x)
        effort = (
            float(self.get_parameter("kp_position").value) * forward_error
            - float(self.get_parameter("kd_velocity").value) * velocity
        )
        cap = float(self.get_parameter("max_effort").value)
        effort = max(-cap, min(cap, effort))
        self._command(effort)

        if time.monotonic() - self._last_status >= 5.0:
            self.get_logger().info(
                f"hold error={forward_error:+.6f}m, v={velocity:+.6f}m/s, "
                f"effort={effort:+.4f}Nm/wheel"
            )
            self._last_status = time.monotonic()

    def shutdown(self) -> None:
        # Ensure gazebo_ros2_control does not retain a nonzero command on exit.
        if not self._external_control:
            self._command(0.0)


def main(args=None) -> None:
    # Keep the ROS context alive until the zero-effort release is published.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ERP42IdleHold()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
