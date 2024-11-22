import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
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

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_controller"),
            "launch",
            "joystick_teleop.launch.py",
        ),
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        remappings=[("/cmd_vel_out", "/alphabot_controller/cmd_vel_unstamped")],
        parameters=[
            {os.path.join(alphabot_controller_package, "config", "twist_mux.yaml")}
        ],
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("alphabot_mapping"), "rviz", "slam.rviz"
            ),
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            gazebo,
            controller,
            # joystick,
            twist_mux_node,
            rviz_slam,
        ]
    )
