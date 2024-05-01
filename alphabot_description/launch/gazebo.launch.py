from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("alphabot_description"), "urdf", "alphabot.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(("alphabot_description"), "share"))

    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
    ))
    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
    ))

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "alphabot", "-topic", "robot_description"],
        output="screen"
    )


    # joint_state_publisher_gui = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui"
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", os.path.join(get_package_share_directory("square_bot_description"), "rviz", "display.rviz")]
    # )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
        # joint_state_publisher_gui,
        # rviz_node
    ])