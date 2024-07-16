from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house"
    )


    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    map_path = PathJoinSubstitution([
        get_package_share_directory("alphabot_mapping"),
        "maps",
        map_name,
        "map.yaml"
    ])


    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        nav2_map_server
    ])