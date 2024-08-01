import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    alphabot_localization_package = get_package_share_directory("alphabot_localization")
    params_file = os.path.join(
        alphabot_localization_package, "config", "nav_params.yaml"
    )

    mapper_params_path = os.path.join(
        alphabot_localization_package,
        "config",
        "mapper_params_online_async.yaml",
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("slam_toolbox"),
            "launch",
            "online_async_launch.py",
        ),
        launch_arguments={
            "-use_sim_time": "true",
            "params_file": mapper_params_path,
        }.items(),
    )

    # Integerating Nav2 Stack
    nav2_toolbox_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("nav2_bringup"),
            "launch",
            "navigation_launch.py",
        ),
        launch_arguments={
            "-use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            slam_toolbox_launch,
            # nav2_toolbox_launch,
        ]
    )
