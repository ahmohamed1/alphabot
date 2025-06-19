import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    alphabot_controller_pkg = get_package_share_directory('alphabot_controller')

    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

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
    
    # joystick = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("alphabot_controller"),
    #         "launch",
    #         "joystick_teleop.launch.py"
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "True"
    #     }.items()
    # )
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
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_navigation"),
            "launch",
            "navigation.launch.py"
        )
    )
    rviz= Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("alphabot_navigation"),
                "rviz",
                "nav2_default_view.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        # condition=UnlessCondition(use_slam)
    )

    # rviz_slam = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", os.path.join(
    #             get_package_share_directory("alphabot_mapping"),
    #             "rviz",
    #             "slam.rviz"
    #         )
    #     ],
    #     output="screen",
    #     parameters=[{"use_sim_time": True}],
    #     condition=IfCondition(use_slam)
    # )
    
    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        # joystick,
        twist_relay_node,
        twist_mux_launch,
        localization,
        slam,
        navigation,
        rviz
    ])