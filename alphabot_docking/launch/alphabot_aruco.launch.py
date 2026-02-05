from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/image_raw",
        description="Image topic",
    )

    camera_info_topic = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/camera_info",
        description="Camera info topic",
    )

    marker_size = DeclareLaunchArgument(
        "marker_size",
        default_value="0.18",
        description="Marker size in meters",
    )

    dictionary_id = DeclareLaunchArgument(
        "dictionary_id",
        default_value="DICT_6X6_250",
        description="ArUco dictionary name",
    )

    target_id = DeclareLaunchArgument(
        "target_id",
        default_value="0",
        description="Marker ID to detect (-1 for any)",
    )

    camera_frame = DeclareLaunchArgument(
        "camera_frame",
        default_value="camera_link_optical",
        description="Camera optical frame (optional override)",
    )

    child_frame = DeclareLaunchArgument(
        "child_frame",
        default_value="aruco_tag",
        description="TF child frame name",
    )

    dock_pose_topic = DeclareLaunchArgument(
        "dock_pose_topic",
        default_value="/detected_dock_pose",
        description="PoseStamped output topic",
    )

    show_debug_image = DeclareLaunchArgument(
        "show_debug_image",
        default_value="false",
        description="Show OpenCV debug window",
    )

    aruco_node = Node(
        package="alphabot_docking",
        executable="aruco_opencv_node.py",
        name="aruco_opencv_node",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "marker_size": LaunchConfiguration("marker_size"),
                "dictionary_id": LaunchConfiguration("dictionary_id"),
                "target_id": LaunchConfiguration("target_id"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "child_frame": LaunchConfiguration("child_frame"),
                "dock_pose_topic": LaunchConfiguration("dock_pose_topic"),
                "show_debug_image": LaunchConfiguration("show_debug_image"),
            }
        ],
    )

    return LaunchDescription([
        image_topic,
        camera_info_topic,
        marker_size,
        dictionary_id,
        target_id,
        camera_frame,
        child_frame,
        dock_pose_topic,
        show_debug_image,
        aruco_node,
    ])
