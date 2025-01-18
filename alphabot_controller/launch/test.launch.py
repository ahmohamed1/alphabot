import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    joy_config_path = os.path.join(
        get_package_share_directory('alphabot_controller'),
        'config',
        'test.yaml'
    )
    args = DeclareLaunchArgument(
        'params_file',
        default_value=joy_config_path,
        description
        ='Path to the YAML file for joy and teleop_twist_joy parameters'
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[LaunchConfiguration('params_file')]
    )

        # Launch teleop_twist_joy_node
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[LaunchConfiguration('params_file')]
    )
    return LaunchDescription([
        # Declare arguments
        args,
        joy_node,
        teleop_twist_joy_node,
        # Launch joy_node
        
    ])
