import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import TimerAction


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="False",
                                      description="Use simulated time"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_slam = LaunchConfiguration("use_slam")
    mapping_backend = LaunchConfiguration("mapping_backend")
    alphabot_controller_pkg = get_package_share_directory('alphabot_controller')

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    mapping_backend_arg = DeclareLaunchArgument(
        "mapping_backend",
        default_value="slam",
        description="Mapping backend when use_slam:=true (slam|cartographer)"
    )

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

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(PythonExpression([
            "'", use_slam, "' == 'true' and '", mapping_backend, "' == 'slam'"
        ])),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    cartographer = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_mapping"),
            "launch",
            "cartographer.launch.py"
        ),
        condition=IfCondition(PythonExpression([
            "'", use_slam, "' == 'true' and '", mapping_backend, "' == 'cartographer'"
        ])),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    robot_localization_launch = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("alphabot_localization"),
                "config",
                "ekf.yaml",
            ),
            {"use_sim_time": use_sim_time},
        ]
    )

    imu_driver_node = Node(
        package="alphabot_firmware",
        executable="mpu6050_driver.py"
    )


    delayed_launch = TimerAction(
        period= 30.0,  # Adjust seconds as needed
        actions=[
            slam,
            cartographer,
            navigation,
        ]
    )
    return LaunchDescription(
        [
            use_sim_time_arg,
            use_slam_arg,
            mapping_backend_arg,
            hardware_interface,
            controller,
            scanner,
            twist_relay_node,
            twist_mux_launch,
            localization,
            delayed_launch,
            robot_localization_launch,
            imu_driver_node,
        ]
    )
