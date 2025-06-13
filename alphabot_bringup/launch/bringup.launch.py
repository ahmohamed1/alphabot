import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    alphabot_controller_pkg = get_package_share_directory('alphabot_controller')


    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    scanner = Node(package="xv_11_driver", executable="xv_11_driver")

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

    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "alphabot_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(alphabot_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(alphabot_controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(alphabot_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    twist_relay_node = Node(
        package="alphabot_controller",
        executable="twist_relay",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )


    

    return LaunchDescription(
        [
            hardware_interface,
            controller,
            # scanner,
            twist_relay_node,
            twist_mux_launch,
            # robot_localization_launch,
        ]
    )
