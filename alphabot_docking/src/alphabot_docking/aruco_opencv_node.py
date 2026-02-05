#!/usr/bin/env python3
"""OpenCV-based ArUco detector node.

Subscribes to camera image + camera_info, detects a specific ArUco marker,
then publishes:
- TF transform camera_frame -> aruco_tag
- PoseStamped on /detected_dock_pose
"""
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ArucoOpenCVNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_opencv_node")

        self.declare_parameter(
            "image_topic",
            "/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Input image topic",
            ),
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic",
            ),
        )
        self.declare_parameter(
            "marker_size",
            0.18,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Marker size in meters",
            ),
        )
        self.declare_parameter(
            "dictionary_id",
            "DICT_6X6_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="ArUco dictionary name (e.g., DICT_6X6_250)",
            ),
        )
        self.declare_parameter(
            "target_id",
            0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Marker ID to detect (use -1 for any)",
            ),
        )
        self.declare_parameter(
            "camera_frame",
            "",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame (optional override)",
            ),
        )
        self.declare_parameter(
            "child_frame",
            "aruco_tag",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="TF child frame name",
            ),
        )
        self.declare_parameter(
            "dock_pose_topic",
            "/detected_dock_pose",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="PoseStamped output topic",
            ),
        )
        self.declare_parameter(
            "show_debug_image",
            False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Show OpenCV debug window",
            ),
        )

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.dictionary_id = self.get_parameter("dictionary_id").value
        self.target_id = int(self.get_parameter("target_id").value)
        self.camera_frame = self.get_parameter("camera_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.dock_pose_topic = self.get_parameter("dock_pose_topic").value
        self.show_debug_image = bool(self.get_parameter("show_debug_image").value)

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, self.dock_pose_topic, 10)

        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.info_callback, qos_profile_sensor_data
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )

        self.intrinsic_mat = None
        self.distortion = None
        self.info_frame = ""

        # Dictionary
        try:
            dict_id = getattr(cv2.aruco, self.dictionary_id)
        except AttributeError:
            self.get_logger().error(
                f"Invalid dictionary_id: {self.dictionary_id}. Using DICT_6X6_250."
            )
            dict_id = cv2.aruco.DICT_6X6_250
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Camera info topic: {self.camera_info_topic}")
        self.get_logger().info(f"Marker size: {self.marker_size}")
        self.get_logger().info(f"Dictionary: {self.dictionary_id}")
        self.get_logger().info(f"Target ID: {self.target_id}")
        self.get_logger().info(f"Publishing pose: {self.dock_pose_topic}")

    def info_callback(self, msg: CameraInfo) -> None:
        self.intrinsic_mat = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.distortion = np.array(msg.d, dtype=np.float64)
        self.info_frame = msg.header.frame_id
        self.destroy_subscription(self.info_sub)

    def image_callback(self, msg: Image) -> None:
        if self.intrinsic_mat is None or self.distortion is None:
            self.get_logger().warn("Waiting for camera_info...")
            return

        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR) if self.show_debug_image else None

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            if self.show_debug_image:
                cv2.imshow("aruco_detections", debug)
                cv2.waitKey(1)
            return

        # filter by target_id
        id_list = ids.flatten().tolist()
        if self.target_id >= 0:
            if self.target_id not in id_list:
                return
            idx = id_list.index(self.target_id)
            corners = [corners[idx]]
            ids = np.array([[self.target_id]], dtype=np.int32)

        if self.show_debug_image:
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.intrinsic_mat, self.distortion
        )

        rvec = rvecs[0]
        tvec = tvecs[0]

        if self.show_debug_image:
            if hasattr(cv2.aruco, "drawAxis"):
                cv2.aruco.drawAxis(debug, self.intrinsic_mat, self.distortion, rvec, tvec, self.marker_size * 0.5)
            else:
                cv2.drawFrameAxes(debug, self.intrinsic_mat, self.distortion, rvec, tvec, self.marker_size * 0.5)
            cv2.imshow("aruco_detections", debug)
            cv2.waitKey(1)

        # PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = self.camera_frame or self.info_frame
        pose_msg.pose.position.x = float(tvec[0][0])
        pose_msg.pose.position.y = float(tvec[0][1])
        pose_msg.pose.position.z = float(tvec[0][2])

        # Convert rvec to quaternion
        rot_mat, _ = cv2.Rodrigues(rvec)
        rot4 = np.eye(4)
        rot4[:3, :3] = rot_mat
        q = tf_quaternion_from_matrix(rot4)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.pose_pub.publish(pose_msg)

        # TF
        tf_msg = TransformStamped()
        tf_msg.header = pose_msg.header
        tf_msg.child_frame_id = self.child_frame
        tf_msg.transform.translation.x = pose_msg.pose.position.x
        tf_msg.transform.translation.y = pose_msg.pose.position.y
        tf_msg.transform.translation.z = pose_msg.pose.position.z
        tf_msg.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def tf_quaternion_from_matrix(matrix: np.ndarray) -> np.ndarray:
    # Minimal quaternion conversion to avoid extra deps
    m = matrix
    t = np.trace(m)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)


def main() -> None:
    rclpy.init()
    node = ArucoOpenCVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
