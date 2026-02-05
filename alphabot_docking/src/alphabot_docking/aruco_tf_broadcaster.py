#!/usr/bin/env python3
"""Broadcast a TF frame and PoseStamped for ArUco detections.

Subscribes to /aruco_poses (geometry_msgs/PoseArray) and publishes:
- TF transform for the first pose in the array.
- PoseStamped on /detected_dock_pose.
"""
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class ArucoTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("aruco_tf_broadcaster")

        self.declare_parameter("pose_topic", "/aruco_poses")
        # If empty, use the PoseArray header frame_id
        self.declare_parameter("parent_frame", "")
        self.declare_parameter("child_frame", "aruco_tag")
        # Optional scaling if marker_size is mismatched
        self.declare_parameter("position_scale", 1.0)
        self.declare_parameter("dock_pose_topic", "/detected_dock_pose")

        self.pose_topic = self.get_parameter("pose_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.position_scale = float(self.get_parameter("position_scale").value)
        self.dock_pose_topic = self.get_parameter("dock_pose_topic").value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.dock_pose_pub = self.create_publisher(PoseStamped, self.dock_pose_topic, 10)
        self.sub = self.create_subscription(
            PoseArray, self.pose_topic, self.pose_callback, 10
        )

        self.get_logger().info(
            f"Listening on {self.pose_topic}; publishing TF {self.parent_frame} -> {self.child_frame}"
        )
        self.get_logger().info(f"Publishing detected dock pose on {self.dock_pose_topic}")

    def pose_callback(self, msg: PoseArray) -> None:
        if not msg.poses:
            return

        pose = msg.poses[0]

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame or msg.header.frame_id
        t.child_frame_id = self.child_frame

        scale = self.position_scale
        t.transform.translation.x = pose.position.x * scale
        t.transform.translation.y = pose.position.y * scale
        t.transform.translation.z = pose.position.z * scale
        t.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(t)

        dock_pose = PoseStamped()
        dock_pose.header.stamp = msg.header.stamp
        dock_pose.header.frame_id = t.header.frame_id
        dock_pose.pose.position.x = t.transform.translation.x
        dock_pose.pose.position.y = t.transform.translation.y
        dock_pose.pose.position.z = t.transform.translation.z
        dock_pose.pose.orientation = pose.orientation
        self.dock_pose_pub.publish(dock_pose)


def main() -> None:
    rclpy.init()
    node = ArucoTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
