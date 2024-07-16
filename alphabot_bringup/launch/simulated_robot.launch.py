import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    alphabot_controller_package = get_package_share_directory("alphabot_controller")

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    
    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "/alphabot_controller/cmd_vel_unstamped",
            "config_topics": os.path.join(alphabot_controller_package,"config","twist_mux.yaml"),
            "config_locks":os.path.join(alphabot_controller_package,"config","twist_mux_lock.yaml"),
            # "config_joy":os.path.join(alphabot_controller_package,"config","twist_mux_joy.yaml"),
        }.items()
    )
    
    return LaunchDescription([
        gazebo,
        controller,
        twist_mux_launch,
    ])