from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=FindPackageShare("alphabot_docking").find("alphabot_docking")
        + "/config/apriltag.yaml",
        description="AprilTag parameters file",
    )

    image_topic = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/image_raw",
        description="Image topic for apriltag detector",
    )

    camera_info_topic = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/camera_info",
        description="Camera info topic for apriltag detector",
    )

    rect_image_topic = DeclareLaunchArgument(
        "rect_image_topic",
        default_value="/image_rect",
        description="Rectified image topic for apriltag detector",
    )

    use_image_proc = DeclareLaunchArgument(
        "use_image_proc",
        default_value="false",
        description="Run image_proc to create rectified images",
    )

    tag_size = DeclareLaunchArgument(
        "tag_size",
        default_value="0.22",
        description="AprilTag size in meters (override config)",
    )

    visualize = DeclareLaunchArgument(
        "visualize",
        default_value="true",
        description="Launch image_view to visualize detections",
    )

    detector_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"size": LaunchConfiguration("tag_size")},
        ],
        remappings=[
            ("image_rect", LaunchConfiguration("rect_image_topic")),
            ("camera_info", LaunchConfiguration("camera_info_topic")),
        ],
    )

    image_proc_node = Node(
        package="image_proc",
        executable="image_proc",
        name="image_proc",
        output="screen",
        remappings=[
            ("image", LaunchConfiguration("image_topic")),
            ("camera_info", LaunchConfiguration("camera_info_topic")),
            ("image_rect", LaunchConfiguration("rect_image_topic")),
        ],
        condition=IfCondition(LaunchConfiguration("use_image_proc")),
    )

    docking_node = Node(
        package="alphabot_docking",
        executable="tag_detector.py",
        name="alphabot_tag_detector",
        output="screen",
        parameters=[{
            "detection_topic": "/tag_detections",
            "target_pose_topic": "/docking/target_pose",
            "target_tag_id": 0,
        }],
    )

    image_view_node = Node(
        package="image_view",
        executable="image_view",
        name="apriltag_detections_view",
        output="screen",
        remappings=[("image", "/detections")],
        condition=IfCondition(LaunchConfiguration("visualize")),
    )

    return LaunchDescription(
        [
            params_file,
            image_topic,
            camera_info_topic,
            rect_image_topic,
            use_image_proc,
            tag_size,
            visualize,
            image_proc_node,
            detector_node,
            docking_node,
            image_view_node,
        ]
    )
