#!/usr/bin/env python3
"""Exercise a running ERP42 Gazebo simulation with ROS 2 topics and motion."""

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage


CAMERAS = ("middle", "left", "right", "back")
JOINTS = (
    "right_front_wheel_joint", "right_back_wheel_joint",
    "left_front_wheel_joint", "left_back_wheel_joint",
    "right_front_disk_wheel_joint", "left_front_disk_wheel_joint",
)


class ERP42Probe(Node):
    def __init__(self, drive_mode):
        super().__init__("erp42_smoke_probe")
        self.drive_mode = drive_mode
        self.position = None
        self.path_length_m = 0.0
        self.track_motion = False
        self.joints = {}
        self.images = {}
        self.camera_info = {}
        self.imu_count = 0
        self.tf_count = 0
        self.filtered_count = 0
        self.noisy_count = 0

        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)
        self.create_subscription(Odometry, "/noisy_odom", self.on_noisy, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_filtered, 10)
        for camera in CAMERAS:
            namespace = f"/erp42/{camera}_camera"
            self.create_subscription(
                Image, f"{namespace}/image_raw",
                lambda msg, name=camera: self.images.__setitem__(
                    name, (msg.width, msg.height, msg.header.frame_id)
                ), qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo, f"{namespace}/camera_info",
                lambda msg, name=camera: self.camera_info.__setitem__(
                    name, (msg.width, msg.height, msg.header.frame_id)
                ), qos_profile_sensor_data,
            )

        if drive_mode == "effort":
            self.wheel_pub = self.create_publisher(
                Float64MultiArray, "/wheel_effort/commands", 10
            )
            self.steer_pub = self.create_publisher(
                Float64MultiArray, "/steering/commands", 10
            )
        else:
            self.velocity_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        position = (p.x, p.y)
        if self.track_motion and self.position is not None:
            self.path_length_m += math.dist(self.position, position)
        self.position = position

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def on_imu(self, msg):
        self.imu_count += 1

    def on_tf(self, msg):
        self.tf_count += sum(
            t.header.frame_id == "odom" and t.child_frame_id == "structure"
            for t in msg.transforms
        )

    def on_noisy(self, msg):
        self.noisy_count += 1

    def on_filtered(self, msg):
        self.filtered_count += 1

    def sensors_ready(self, localization):
        return (
            self.position is not None
            and self.imu_count > 0
            and self.tf_count > 0
            and all(name in self.joints for name in JOINTS)
            and len(self.images) == len(CAMERAS)
            and len(self.camera_info) == len(CAMERAS)
            and (not localization or (self.filtered_count > 0 and self.noisy_count > 0))
        )

    def command(self, active):
        if self.drive_mode == "effort":
            self.wheel_pub.publish(Float64MultiArray(data=[15.0 if active else 0.0] * 4))
            self.steer_pub.publish(Float64MultiArray(data=[0.15 if active else 0.0] * 2))
        else:
            msg = Twist()
            msg.linear.x = 0.5 if active else 0.0
            msg.angular.z = 0.15 if active else 0.0
            self.velocity_pub.publish(msg)


def spin_for(node, seconds, command=None):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if command is not None:
            node.command(command)
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drive-mode", choices=("effort", "skid"), default="effort")
    parser.add_argument("--localization", action="store_true")
    parser.add_argument("--ready-timeout", type=float, default=75.0)
    parser.add_argument("--drive-seconds", type=float, default=6.0)
    args = parser.parse_args()

    rclpy.init()
    probe = ERP42Probe(args.drive_mode)
    try:
        deadline = time.monotonic() + args.ready_timeout
        while time.monotonic() < deadline and not probe.sensors_ready(args.localization):
            rclpy.spin_once(probe, timeout_sec=0.1)

        ready = probe.sensors_ready(args.localization)
        before = probe.position
        commanded_steering = None
        if ready:
            probe.track_motion = True
            spin_for(probe, args.drive_seconds, command=True)
            probe.track_motion = False
            if args.drive_mode == "effort":
                commanded_steering = [
                    probe.joints.get(name) for name in
                    ("right_front_disk_wheel_joint", "left_front_disk_wheel_joint")
                ]
            spin_for(probe, 1.0, command=False)
        after = probe.position
        displacement = math.dist(before, after) if before and after else 0.0
        steering = [
            probe.joints.get(name) for name in
            ("right_front_disk_wheel_joint", "left_front_disk_wheel_joint")
        ]
        details = {
            "drive_mode": args.drive_mode,
            "localization": args.localization,
            "sensors_ready": ready,
            "imu_count": probe.imu_count,
            "tf_count": probe.tf_count,
            "images": probe.images,
            "camera_info": probe.camera_info,
            "joint_count": len(probe.joints),
            "noisy_count": probe.noisy_count,
            "filtered_count": probe.filtered_count,
            "displacement_m": round(displacement, 3),
            "path_length_m": round(probe.path_length_m, 3),
            "steering_during_command_rad": commanded_steering,
            "final_steering_rad": steering,
        }
        print(json.dumps(details, ensure_ascii=False, indent=2))

        images_ok = all(
            probe.images.get(name, ())[:2] == (800, 800)
            and probe.camera_info.get(name, ())[:2] == (800, 800)
            and probe.images.get(name, (None, None, None))[2] == f"{name}_camera_optical_frame"
            and probe.camera_info.get(name, (None, None, None))[2] == f"{name}_camera_optical_frame"
            for name in CAMERAS
        )
        if not ready or not images_ok:
            raise RuntimeError("One or more camera, IMU, joint, TF or EKF topics are missing")
        if probe.path_length_m < 0.1:
            raise RuntimeError("ERP42 did not travel at least 0.1 m under drive commands")
        if args.drive_mode == "effort" and (
            commanded_steering is None
            or any(angle is None or abs(angle - 0.15) > 0.1 for angle in commanded_steering)
        ):
            raise RuntimeError("The steering joints did not follow the 0.15 rad command")
        print("ERP42 ROS 2 Gazebo smoke test passed")
    finally:
        # Release any active command even when the test itself raises.
        try:
            spin_for(probe, 0.3, command=False)
        except Exception:
            pass
        probe.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
