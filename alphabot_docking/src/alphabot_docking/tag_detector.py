#!/usr/bin/env python3
"""Simple AprilTag detection listener for docking.

Subscribes to /tag_detections and republishes a PoseStamped for the chosen tag.
"""
from typing import Optional

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped


class TagDetector(Node):
    def __init__(self) -> None:
        super().__init__("alphabot_tag_detector")

        self.declare_parameter("detection_topic", "/tag_detections")
        self.declare_parameter("target_pose_topic", "/docking/target_pose")
        self.declare_parameter("target_tag_id", -1)  # -1 means accept first detection

        detection_topic = self.get_parameter("detection_topic").value
        target_pose_topic = self.get_parameter("target_pose_topic").value
        self.target_tag_id = int(self.get_parameter("target_tag_id").value)

        self.pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self.sub = self.create_subscription(
            AprilTagDetectionArray, detection_topic, self.detection_cb, 10
        )

        self._last_log_time = self.get_clock().now()
        self.get_logger().info(
            f"Listening on {detection_topic}, publishing to {target_pose_topic}"
        )

    def _choose_detection(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            return None
        if self.target_tag_id < 0:
            return msg.detections[0]
        for det in msg.detections:
            if det.id and det.id[0] == self.target_tag_id:
                return det
        return None

    def detection_cb(self, msg: AprilTagDetectionArray) -> None:
        det = self._choose_detection(msg)
        if det is None:
            return

        pose_msg = PoseStamped()
        pose_msg.header = det.pose.header
        pose_msg.pose = det.pose.pose.pose
        self.pose_pub.publish(pose_msg)

        now = self.get_clock().now()
        if (now - self._last_log_time).nanoseconds > 1e9:
            self.get_logger().info(
                f"Published target pose for tag id {det.id[0] if det.id else 'unknown'}"
            )
            self._last_log_time = now


def main() -> None:
    rclpy.init()
    node = TagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
