"""Sample ground truth at 5 Hz with uniform lateral position noise."""

import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class NoisyOdometry(Node):
    def __init__(self):
        super().__init__("noisy_odom")
        self.latest = None
        self.publisher = self.create_publisher(Odometry, "/noisy_odom", 10)
        self.create_subscription(Odometry, "/odom", self.receive, 10)
        self.create_timer(0.2, self.publish)

    def receive(self, msg):
        self.latest = msg

    def publish(self):
        if self.latest is None:
            return
        msg, self.latest = self.latest, None
        msg.pose.pose.position.y += random.uniform(-0.5, 0.5)
        msg.pose.covariance[7] += 1.0 / 12.0
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = NoisyOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
