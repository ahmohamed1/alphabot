import os
import sys
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

sys.path.append(os.path.dirname(__file__))
from robot_model import get_robot_model, VALID_ROBOT_MODELS  # noqa: E402

def generate_launch_description():
    alphabot_controller_pkg = get_package_share_directory('alphabot_controller')

    selected_model = get_robot_model()

    robot_model_arg = DeclareLaunchArgument(
        name="robot_model",
        default_value=selected_model,
        description=f"Robot model to bring up. One of {VALID_ROBOT_MODELS}"
    )
    robot_model = LaunchConfiguration("robot_model")

    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("alphabot_description"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={"robot_model": robot_model}.items(),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("alphabot_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("twist_mux"),
                "launch",
                "twist_mux_launch.py"
            )
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
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("alphabot_localization"),
                "launch",
                "global_localization.launch.py"
            )
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("alphabot_mapping"),
                "launch",
                "slam.launch.py"
            )
        ),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("alphabot_navigation"),
                "launch",
                "navigation.launch.py"
            )
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("alphabot_navigation"),
            "rviz",
            "nav2_default_view.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # This will launch SLAM/Localization, Navigation, and RViz AFTER controller.launch.py exits
    delayed_launch = TimerAction(
        period= 10.0,  # Adjust seconds as needed
        actions=[
            slam,
            navigation,
        ]
    )

    return LaunchDescription([
        robot_model_arg,
        use_slam_arg,
        gazebo,
        controller_launch,
        twist_relay_node,
        twist_mux_launch,
        localization,
        rviz,
        delayed_launch
    ])
