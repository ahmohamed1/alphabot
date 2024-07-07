import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("alphabot_controller"), "config", "joy_teleop.yaml")],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("alphabot_controller"), "config", "joy_config.yaml")]
    )

    alphabot_controller_package = get_package_share_directory("alphabor_controller")

    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "alphabot_controller/cmd_vel_unstamped",
            "config_topics": os.path.join(alphabot_controller_package,"config","twist_mux_topic.yaml"),
            "config_locks":os.path.join(alphabot_controller_package,"config","twist_mux_lock.yaml"),
            "config_joy":os.path.join(alphabot_controller_package,"config","twist_mux_joy.yaml"),
        }.items()
    )

    return LaunchDescription(
        [
            joy_teleop,
            joy_node,
            twist_mux_launch
        ]
    )