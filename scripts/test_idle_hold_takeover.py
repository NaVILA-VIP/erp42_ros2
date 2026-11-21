#!/usr/bin/env python3
"""Temporarily publish wheel effort to verify ERP42 idle-hold handoff.

Publish a small forward effort for 3 seconds and release. The separate
idle_hold.py node should yield to this publisher, then recapture position.
"""
import time

import rclpy
from std_msgs.msg import Float64MultiArray


def main():
    rclpy.init()
    node = rclpy.create_node("erp42_manual_test_drive")
    pub = node.create_publisher(Float64MultiArray, "/wheel_effort/commands", 10)
    msg = Float64MultiArray()
    msg.data = [1.0, 1.0, 1.0, 1.0]
    try:
        # Allow ROS graph changes to reach the hold controller before driving.
        time.sleep(1.0)
        print("Manual drive publisher started", flush=True)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            pub.publish(msg)
            time.sleep(0.05)
        print("Manual drive publisher stopping", flush=True)
        # Safe zero-command release before removing publisher.
        msg.data = [0.0, 0.0, 0.0, 0.0]
        for _ in range(4):
            pub.publish(msg)
            time.sleep(0.05)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
