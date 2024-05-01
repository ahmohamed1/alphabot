from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.03",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.27",
    )
    
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='True'
    )

    
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    is_sim = LaunchConfiguration('is_sim')
    
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("alphabot_description"),
                    "urdf",
                    "alphabot.urdf.xacro",
                ),
                " is_sim:=False"
            ]
        ),
        value_type=str,
    )

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("alphabot_description"), "urdf", "alphabot.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    contoller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description,
        "use_sim_time":is_sim}]
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["alphabot_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller),
    )


    return LaunchDescription(
        [
            wheel_radius_arg,
            wheel_separation_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            simple_controller,
        ]
    )