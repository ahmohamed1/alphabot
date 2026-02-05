from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch auto-dock node with configuration."""
    
    # Get the package share directory
    package_share = FindPackageShare("alphabot_docking").find("alphabot_docking")
    
    # Declare launch arguments
    config_file = DeclareLaunchArgument(
        "config_file",
        default_value=package_share + "/config/auto_dock.yaml",
        description="Configuration file for auto-dock node",
    )
    
    battery_threshold = DeclareLaunchArgument(
        "battery_threshold",
        default_value="25.0",
        description="Battery threshold percentage to trigger docking",
    )
    
    # Auto-dock node
    auto_dock_node = Node(
        package="alphabot_docking",
        executable="auto_dock_node.py",
        name="auto_dock_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "battery_threshold": LaunchConfiguration("battery_threshold"),
            }
        ],
    )
    
    return LaunchDescription(
        [
            config_file,
            battery_threshold,
            auto_dock_node,
        ]
    )
