import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rmw_env = SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=os.environ.get("ROBOT_MODEL", "servicebot"),
        description="Robot model in use. One of ['servicebot', 'alphabot']",
    )
    host_arg = DeclareLaunchArgument(
        "host",
        default_value="0.0.0.0",
        description="Bind address for the local network Web GUI server",
    )
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="8090",
        description="HTTP/WebSocket port for the Web GUI server",
    )
    default_map_arg = DeclareLaunchArgument(
        "map_name",
        default_value="",
        description="Optional initial map name inside alphabot_mapping/maps (empty starts in Standby mode)",
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/key_vel",
        description="Velocity topic routed into twist_mux (priority 90)",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated clock",
    )

    gateway_node = Node(
        package="alphabot_web_gui",
        executable="gateway_node",
        name="alphabot_web_gateway",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "robot_model": LaunchConfiguration("robot_model"),
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "default_map": LaunchConfiguration("map_name"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "use_sim_time_for_switches": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(
        [
            rmw_env,
            robot_model_arg,
            host_arg,
            port_arg,
            default_map_arg,
            cmd_vel_topic_arg,
            use_sim_time_arg,
            gateway_node,
        ]
    )
