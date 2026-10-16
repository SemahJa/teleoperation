#!/usr/bin/env python3
"""Explicit robot-free fixture: publish a fixed synthetic measured pose."""
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class MockMeasuredPose(Node):
    def __init__(self):
        super().__init__("mock_measured_pose")
        self.declare_parameter("base_frame", "panda_1/panda_link0")
        self.declare_parameter("position", [0.4, 0.0, 0.4])
        self.declare_parameter("orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("measured_pose_topic", "teleop_anchor_pose")
        self.pose = PoseStamped()
        self.pose.header.frame_id = self.get_parameter("base_frame").value
        position = self.get_parameter("position").value
        q = self.get_parameter("orientation_xyzw").value
        if (
            len(position) != 3
            or len(q) != 4
            or not all(math.isfinite(x) for x in position + q)
        ):
            raise ValueError("invalid mock pose")
        norm = math.sqrt(sum(x * x for x in q))
        if norm < 1e-9:
            raise ValueError("invalid mock quaternion")
        (
            self.pose.pose.position.x,
            self.pose.pose.position.y,
            self.pose.pose.position.z,
        ) = position
        (
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.pose.pose.orientation.w,
        ) = [x / norm for x in q]
        self.publisher = self.create_publisher(
            PoseStamped, self.get_parameter("measured_pose_topic").value, 10
        )
        self.create_timer(0.01, self.publish)
        self.get_logger().warning(
            "MOCK measured pose: fixed synthetic fixture, no robot connection"
        )

    def publish(self):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.pose)


def main():
    rclpy.init()
    node = MockMeasuredPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
