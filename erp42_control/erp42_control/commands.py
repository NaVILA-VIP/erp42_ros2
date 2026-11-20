"""Command the ERP42 group controllers in N·m and radians."""

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray


def run(name, torque_nm=None, steering_rad=None):
    rclpy.init()
    node = Node(name)
    outputs = []
    try:
        if torque_nm is not None:
            value = node.declare_parameter("torque", torque_nm).value
            if not math.isfinite(value):
                raise ValueError("torque must be finite")
            outputs.append((node.create_publisher(Float64MultiArray, "/wheel_effort/commands", 1),
                            Float64MultiArray(data=[value] * 4)))
        if steering_rad is not None:
            value = node.declare_parameter("angle", steering_rad).value
            if not -0.4 <= value <= 0.4:
                raise ValueError("angle must be within [-0.4, 0.4] radians")
            outputs.append((node.create_publisher(Float64MultiArray, "/steering/commands", 1),
                            Float64MultiArray(data=[value] * 2)))

        def publish():
            for publisher, command in outputs:
                publisher.publish(command)

        node.create_timer(0.1, publish)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def torque():
    run("erp42_torque", torque_nm=12.0)


def steer():
    run("erp42_steering", steering_rad=0.4)


def motion():
    run("erp42_motion", torque_nm=0.0, steering_rad=0.0)
