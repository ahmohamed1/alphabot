import os
import sys
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulated Gazebo clock",
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="small_house",
        description="Gazebo world name inside alphabot_description/worlds",
    )

    map_name_arg = DeclareLaunchArgument(
        name="map_name",
        default_value="",
        description="Optional map name inside alphabot_mapping/maps (empty starts in First-Time Standby mode controlled via Web GUI)",
    )

    has_map = PythonExpression([
        "'", LaunchConfiguration("map_name"), "' != '' and '", use_slam, "' != 'true'"
    ])
    should_autostart_nav = PythonExpression([
        "'", LaunchConfiguration("map_name"), "' != '' or '", use_slam, "' == 'true'"
    ])

    use_rviz_arg = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true",
        description="Launch RViz2 alongside simulation",
    )

    use_gui_arg = DeclareLaunchArgument(
        name="use_gui",
        default_value="true",
        description="Launch the local network Web GUI (alphabot_web_gui)",
    )

    gui_port_arg = DeclareLaunchArgument(
        name="gui_port",
        default_value="8090",
        description="HTTP/WebSocket port for the local network Web GUI",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("alphabot_description"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "robot_model": robot_model,
            "world_name": LaunchConfiguration("world_name"),
        }.items(),
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
        condition=IfCondition(has_map),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map_name": LaunchConfiguration("map_name"),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("alphabot_mapping"),
                "launch",
                "slam.launch.py"
            )
        ),
        condition=IfCondition(use_slam),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("alphabot_navigation"),
                "launch",
                "navigation.launch.py"
            )
        ),
        condition=IfCondition(should_autostart_nav),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
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
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    web_gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("alphabot_web_gui"),
                "launch",
                "web_gui.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_gui")),
        launch_arguments={
            "robot_model": robot_model,
            "port": LaunchConfiguration("gui_port"),
            "map_name": LaunchConfiguration("map_name"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # Launch SLAM/Navigation after Gazebo and controllers initialize
    delayed_launch = TimerAction(
        period= 5.0,
        actions=[
            slam,
            navigation,
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        robot_model_arg,
        use_slam_arg,
        use_sim_time_arg,
        world_name_arg,
        map_name_arg,
        use_rviz_arg,
        use_gui_arg,
        gui_port_arg,
        gazebo,
        controller_launch,
        twist_relay_node,
        twist_mux_launch,
        localization,
        rviz,
        web_gui_launch,
        delayed_launch
    ])
