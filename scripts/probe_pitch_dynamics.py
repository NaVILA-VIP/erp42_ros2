#!/usr/bin/env python3
"""Measure live Gazebo pose during a brief test wheel-torque pulse.

Requires a running local ERP42 Gazebo simulation in effort mode. Sends zero
wheel effort in finally, even when a measurement fails. Drives the simulation
vehicle temporarily but does not edit models or persistent ROS configuration.
"""
import argparse
import math
import subprocess
import time

import rclpy
from std_msgs.msg import Float64MultiArray


def pose():
    result = subprocess.run(
        ["timeout", "5s", "gz", "model", "-m", "erp42", "-p"],
        capture_output=True, text=True, timeout=7,
    )
    if result.returncode:
        raise RuntimeError(f"Gazebo pose query failed: {result.stderr.strip()}")
    result = [float(x) for x in result.stdout.split()]
    if len(result) != 6:
        raise RuntimeError("Unexpected Gazebo model pose: " + str(result))
    return result


def display(t, label):
    x, y, z, r, p, yaw = pose()
    print(f"{label} t={t:4.1f}s x={x:+.5f} y={y:+.5f} z={z:.5f}"
          f" roll={math.degrees(r):+.5f}deg pitch={math.degrees(p):+.5f}deg"
          f" yaw={math.degrees(yaw):+.5f}deg", flush=True)


def main():
    args = argparse.ArgumentParser()
    args.add_argument("--torque", type=float, default=12.0)
    args.add_argument("--seconds", type=float, default=4.0)
    options = args.parse_args()
    if abs(options.torque) > 15 or not (0.2 <= options.seconds <= 8):
        raise ValueError("Test limited to <=15 N m and 0.2-8 seconds.")

    rclpy.init()
    node = rclpy.create_node("erp42_pitch_audit")
    pub = node.create_publisher(Float64MultiArray, "/wheel_effort/commands", 10)
    def command(torque):
        msg = Float64MultiArray()
        msg.data = [torque] * 4
        pub.publish(msg)
    start = time.monotonic()
    try:
        deadline = start + 10
        while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
            time.sleep(0.2)
        if pub.get_subscription_count() == 0:
            raise RuntimeError("No wheel effort controller subscriber; simulation not ready")
        display(0, "idle")
        command(options.torque)
        while time.monotonic() - start < options.seconds:
            t = time.monotonic() - start
            if t >= 1:
                display(t, "driving")
            command(options.torque)
            time.sleep(1)
    finally:
        command(0.0)
        time.sleep(0.2)
        command(0.0)
        print("Wheel effort reset to zero", flush=True)
        node.destroy_node()
        rclpy.shutdown()
    display(time.monotonic() - start, "stopped")


if __name__ == "__main__":
    main()
