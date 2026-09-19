import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import TimerAction

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
    is_servicebot = PythonExpression(["'", robot_model, "' == 'servicebot'"])

    scan_min_range_arg = DeclareLaunchArgument(
        "scan_min_range", default_value="0.20",
        description="Discard all LiDAR returns closer than this distance in meters",
    )
    scan_wall_max_range_arg = DeclareLaunchArgument(
        "scan_wall_max_range", default_value="0.40",
        description="Discard side-wall sector returns up to this distance in meters",
    )
    scan_left_wall_min_angle_arg = DeclareLaunchArgument(
        "scan_left_wall_min_angle", default_value="1.31",
        description="Start angle in radians for the left LiDAR mounting-wall sector",
    )
    scan_left_wall_max_angle_arg = DeclareLaunchArgument(
        "scan_left_wall_max_angle", default_value="1.83",
        description="End angle in radians for the left LiDAR mounting-wall sector",
    )
    scan_right_wall_min_angle_arg = DeclareLaunchArgument(
        "scan_right_wall_min_angle", default_value="-1.83",
        description="Start angle in radians for the right LiDAR mounting-wall sector",
    )
    scan_right_wall_max_angle_arg = DeclareLaunchArgument(
        "scan_right_wall_max_angle", default_value="-1.31",
        description="End angle in radians for the right LiDAR mounting-wall sector",
    )

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="False",
                                      description="Use simulated time"
    )
    use_slam = LaunchConfiguration("use_slam")
    alphabot_controller_pkg = get_package_share_directory('alphabot_controller')

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
        launch_arguments={"robot_model": robot_model}.items(),
    )

    scanner = Node(
        package="xv_11_driver",
        executable="xv_11_driver",
        condition=UnlessCondition(is_servicebot),
    )

    servicebot_scanner = Node(
        package="xv_11_driver",
        executable="xv_11_driver",
        remappings=[("scan", "/scan_raw")],
        condition=IfCondition(is_servicebot),
    )

    servicebot_scan_filter = Node(
        package="alphabot_utils",
        executable="servicebot_scan_filter",
        name="servicebot_scan_filter",
        parameters=[{
            "min_range": LaunchConfiguration("scan_min_range"),
            "wall_max_range": LaunchConfiguration("scan_wall_max_range"),
            "left_wall_min_angle": LaunchConfiguration("scan_left_wall_min_angle"),
            "left_wall_max_angle": LaunchConfiguration("scan_left_wall_max_angle"),
            "right_wall_min_angle": LaunchConfiguration("scan_right_wall_min_angle"),
            "right_wall_max_angle": LaunchConfiguration("scan_right_wall_max_angle"),
        }],
        remappings=[("scan_raw", "/scan_raw"), ("scan", "/scan")],
        condition=IfCondition(is_servicebot),
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
        launch_arguments={"use_sim_time": "false"}.items()
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam),
        launch_arguments={"use_sim_time": "false"}.items()
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={"use_sim_time": "false"}.items()
    )

    robot_localization_launch = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    imu_driver_node = Node(
        package="alphabot_firmware",
        executable="mpu6050_driver.py"
    )


    delayed_launch = TimerAction(
        period= 30.0,  # Adjust seconds as needed
        actions=[
            slam,
            navigation,
        ]
    )
    return LaunchDescription(
        [
            robot_model_arg,
            scan_min_range_arg,
            scan_wall_max_range_arg,
            scan_left_wall_min_angle_arg,
            scan_left_wall_max_angle_arg,
            scan_right_wall_min_angle_arg,
            scan_right_wall_max_angle_arg,
            use_sim_time_arg,
            use_slam_arg,
            hardware_interface,
            controller,
            scanner,
            servicebot_scanner,
            servicebot_scan_filter,
            twist_relay_node,
            twist_mux_launch,
            localization,
            delayed_launch,
            robot_localization_launch,
            imu_driver_node,
        ]
    )
