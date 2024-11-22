from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the launch argument for the parameter file
    arg_parameter = DeclareLaunchArgument(
        'params_file',
        default_value=FindPackageShare('alphabot_utils').find('alphabot_utils') + '/config/follow_wall.yaml',
        description='Full path to the parameter file to load'
    )

    # Node configuration
    follow_wall_node = Node(
        package='alphabot_utils',
        executable='follow_wall',  # Replace with your node's executable name
        name='follow_wall',         # Replace with your node's name
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    return LaunchDescription([
        arg_parameter,
        follow_wall_node,
    ])
