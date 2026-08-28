import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

sys.path.append(os.path.dirname(__file__))
from robot_model import get_robot_model, VALID_ROBOT_MODELS  # noqa: E402


def generate_launch_description():

    selected_model = get_robot_model()

    robot_model_arg = DeclareLaunchArgument(
        name="robot_model",
        default_value=selected_model,
        description=f"Robot model to bring up. One of {VALID_ROBOT_MODELS}"
    )
    robot_model = LaunchConfiguration("robot_model")

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="False",
                                      description="Use simulated time"
    )

    alphabot_controller_pkg = get_package_share_directory('alphabot_controller')


    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
        launch_arguments={"robot_model": robot_model}.items(),
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

    robot_localization_launch = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(
            get_package_share_directory("alphabot_localization"),
            "config",
            "ekf.yaml"
        ),
            {"use_sim_time": False}
            ]
    )

    imu_driver_node = Node(
        package="alphabot_firmware",
        executable="mpu6050_driver.py"
    )



    return LaunchDescription(
        [
            robot_model_arg,
            use_sim_time_arg,
            hardware_interface,
            controller,
            scanner,
            twist_relay_node,
            twist_mux_launch,
            robot_localization_launch,
            imu_driver_node,
        ]
    )
