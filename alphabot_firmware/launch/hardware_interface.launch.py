import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_model_arg = DeclareLaunchArgument(
        name="robot_model",
        default_value=os.environ.get("ROBOT_MODEL", "alphabot"),
        description="Robot model to bring up. One of ['alphabot', 'servicebot']"
    )

    serial_device_arg = DeclareLaunchArgument(
        name="serial_device",
        default_value=os.environ.get("SERIAL_DEVICE", "/dev/ttyACM0"),
        description="Pico serial device: /dev/ttyACM0 for USB or /dev/serial0 for GPIO UART",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution([
                    get_package_share_directory("alphabot_description"),
                    "urdf",
                    LaunchConfiguration("robot_model"),
                    "robot.urdf.xacro",
                ]),
                " is_sim:=False",
                " serial_device:=",
                LaunchConfiguration("serial_device"),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            PathJoinSubstitution([
                get_package_share_directory("alphabot_controller"),
                "config",
                LaunchConfiguration("robot_model"),
                "controllers.yaml",
            ]),
        ],
    )



    return LaunchDescription(
        [
            robot_model_arg,
            serial_device_arg,
            robot_state_publisher_node,
            controller_manager,
        ]
    )