import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    alphabot_description = get_package_share_directory("alphabot_description")

    robot_model_arg = DeclareLaunchArgument(
        name="robot_model",
        default_value=os.environ.get("ROBOT_MODEL", "alphabot"),
        description="Robot model to spawn. One of ['alphabot', 'servicebot']"
    )

    model_arg = DeclareLaunchArgument(
        name="model", default_value=PathJoinSubstitution([
                alphabot_description, "urdf", LaunchConfiguration("robot_model"), "robot.urdf.xacro"
            ]),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution([
            alphabot_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(alphabot_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("alphabot_description"), 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", LaunchConfiguration("robot_model")],
    )

    is_servicebot = PythonExpression(["'", LaunchConfiguration("robot_model"), "' == 'servicebot'"])

    # servicebot's laser is centered on the robot, so the small side walls
    # around the sensor housing show up as near-range "obstacles". Its scan
    # is bridged onto /scan_raw and passed through servicebot_scan_filter
    # (alphabot_utils) which republishes the cleaned data on /scan.
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ],
        condition=UnlessCondition(is_servicebot),
    )

    gz_ros2_bridge_servicebot = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        remappings=[
            ('/imu', '/imu/out'),
            ('/scan', '/scan_raw'),
        ],
        condition=IfCondition(is_servicebot),
    )

    servicebot_scan_filter = Node(
        package="alphabot_utils",
        executable="servicebot_scan_filter",
        name="servicebot_scan_filter",
        parameters=[{"min_range": 0.25}],
        remappings=[
            ("scan_raw", "/scan_raw"),
            ("scan", "/scan"),
        ],
        condition=IfCondition(is_servicebot),
    )

    return LaunchDescription([
        robot_model_arg,
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        gz_ros2_bridge_servicebot,
        servicebot_scan_filter,
    ])