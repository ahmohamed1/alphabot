import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Open RViz."
    )

    ## ***** Nodes *****
    # Define package share directory and config file for Cartographer node
    alphabot_mapping_package = get_package_share_directory("alphabot_mapping")
    cartographer_config_dir = os.path.join(alphabot_mapping_package, "config")
    cartographer_node_config = os.path.join(cartographer_config_dir, "bot_lds_2d.lua")

    # Check if the configuration file exists
    if not os.path.exists(cartographer_node_config):
        raise FileNotFoundError(f"Cartographer configuration file not found: {cartographer_node_config}")

    # Cartographer node
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "-configuration_directory", cartographer_config_dir,
            "-configuration_basename", "bot_lds_2d.lua"
        ],
        output="screen",
        remappings=[
            ("/odom", "/alphabot_controller/odom"),
            ("/imu", "/imu/out"),
        ],
    )

    # Cartographer occupancy grid node
    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"resolution": 0.05},
        ],
    )

    # Handle environment variables for robot description
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # RViz node
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", os.path.join(get_package_share_directory("alphabot_mapping"), "rviz", "cartographer.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Launch description with arguments and nodes
    return LaunchDescription(
        [
            # Arguments
            use_sim_time_arg,
            rviz_arg,
            # Nodes
            cartographer_node,
            cartographer_occupancy_grid_node,
            # rviz,
        ]
    )
