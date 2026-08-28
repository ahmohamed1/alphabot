from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    share_dir = get_package_share_directory("alphabot_description")

    robot_model_arg = DeclareLaunchArgument(
        name="robot_model",
        default_value=os.environ.get("ROBOT_MODEL", "alphabot"),
        description="Robot model to display. One of ['alphabot', 'servicebot']"
    )

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=PathJoinSubstitution([
            share_dir, "urdf", LaunchConfiguration("robot_model"), "robot.urdf.xacro"
        ]),
        description='URDF file to publish'
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            share_dir, "rviz", [LaunchConfiguration("robot_model"), ".rviz"]
        ])]
    )

    return LaunchDescription([
        robot_model_arg,
        model_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])