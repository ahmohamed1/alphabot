import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    alphabot_controller_package = get_package_share_directory("alphabot_controller")

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_description"),
            "launch",
            "gazebo.launch.py",
        ),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_controller"),
            "launch",
            "controller.launch.py",
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False",
        }.items(),
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        remappings=[("/cmd_vel_out", "/alphabot_controller/cmd_vel_unstamped")],
        parameters=[
            {os.path.join(alphabot_controller_package, "config", "twist_mux.yaml")}
        ],
    )

    return LaunchDescription(
        [
            gazebo,
            controller,
            twist_mux_node,
        ]
    )
