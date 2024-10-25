import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    alphabot_controller_package = get_package_share_directory("alphabot_controller")

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
            get_package_share_directory("twist_mux"), "launch", "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "/alphabot_controller/cmd_vel_unstamped",
            "config_topics": os.path.join(
                alphabot_controller_package, "config", "twist_mux.yaml"
            ),
            # "config_locks":os.path.join(alphabot_controller_package,"config","twist_mux_lock.yaml"),
            # "config_joy":os.path.join(alphabot_controller_package,"config","twist_mux_joy.yaml"),
        }.items(),
    )

    imu_driver_node = Node(
        package="alphabot_firmware",
        executable="mpu6050_driver.py"
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(alphabot_controller_package, "config/ekf.yaml"),
            {"use_sim_time": "false"},
        ],
    )

    return LaunchDescription(
        [
            hardware_interface,
            controller,
            scanner,
            twist_mux_launch,
            imu_driver_node,
            # robot_localization_node,
        ]
    )
