import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rmw_env = SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")

    ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
    lifecycle_nodes = ["map_saver_server"]

    has_slam_toolbox = False
    try:
        get_package_share_directory("slam_toolbox")
        has_slam_toolbox = True
    except PackageNotFoundError:
        has_slam_toolbox = False

    if has_slam_toolbox and ros_distro != "humble":
        lifecycle_nodes.append("slam_toolbox")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("alphabot_mapping"),
            "config",
            "slam_toolbox.yaml"
        ),
        description="Full path to slam yaml file to load"
    )

    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": 0.196},
            {"occupied_thresh_default": 0.65},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    actions = [
        rmw_env,
        use_sim_time_arg,
        slam_config_arg,
        nav2_map_saver,
        nav2_lifecycle_manager,
    ]

    if has_slam_toolbox:
        actions.append(
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_config,
                    {"use_sim_time": use_sim_time},
                ],
            )
        )
    else:
        # Fallback to alphabot_mapping's built-in C++ occupancy grid mapper + map->odom TF
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="slam_map_to_odom_broadcaster",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                    "--frame-id", "map", "--child-frame-id", "odom",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )
        actions.append(
            Node(
                package="alphabot_mapping",
                executable="mapping_with_known_poses",
                name="mapping_with_known_poses",
                output="screen",
                parameters=[
                    {
                        "width": 25.0,
                        "height": 25.0,
                        "resolution": 0.05,
                        "use_sim_time": use_sim_time,
                    }
                ],
            )
        )

    return LaunchDescription(actions)
