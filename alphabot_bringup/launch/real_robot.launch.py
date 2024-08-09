import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument("use_slam", default_value="false")

    alphabot_controller_package = get_package_share_directory("alphabot_controller")

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_firmware"),
            "launch",
            "hardware_interface.launch.py"
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
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
    )
    
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        remappings=[("/cmd_vel_out", "/alphabot_controller/cmd_vel_unstamped")],
        parameters=[
            {os.path.join(alphabot_controller_package, "config", "twist_mux.yaml")}
        ],
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_localization"),
            "launch",
            "global_localization.launch.py",
        ),
        condition=UnlessCondition(use_slam),
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_mapping"), "launch", "slam.launch.py"
        ),
        condition=IfCondition(use_slam),
    )

    # rviz_localization = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=[
    #         "-d",
    #         os.path.join(
    #             get_package_share_directory("alphabot_localization"),
    #             "rviz",
    #             "global_localization.rviz",
    #         ),
    #     ],
    #     output="screen",
    #     parameters=[{"use_sim_time": True}],
    #     condition=UnlessCondition(use_slam),
    # )

    # rviz_slam = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=[
    #         "-d",
    #         os.path.join(
    #             get_package_share_directory("alphabot_mapping"), "rviz", "slam.rviz"
    #         ),
    #     ],
    #     output="screen",
    #     parameters=[{"use_sim_time": True}],
    #     condition=IfCondition(use_slam),
    # )
    return LaunchDescription([
        use_slam_arg,
        hardware_interface,
        controller,
        # joystick,
        twist_mux_node,
        localization,
        slam,
        # rviz_localization,
        # rviz_slam,
    ])