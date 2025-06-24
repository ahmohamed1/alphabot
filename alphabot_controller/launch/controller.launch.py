from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    alphabot_controller_pkg = get_package_share_directory('alphabot_controller')
    
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["alphabot_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
        ]
    )