import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

sys.path.append(os.path.dirname(__file__))
from robot_model import get_robot_model, VALID_ROBOT_MODELS  # noqa: E402

def generate_launch_description():

    selected_model = get_robot_model()

    # NOTE: navigator.launch.py only starts localization/SLAM/Nav2, none of
    # which currently read model-specific config. The robot_model argument is
    # declared here for consistency with the other bringup launch files and
    # as an extension point (e.g. per-model costmap footprint/robot_radius).
    robot_model_arg = DeclareLaunchArgument(
        name="robot_model",
        default_value=selected_model,
        description=f"Robot model in use. One of {VALID_ROBOT_MODELS}"
    )

    use_slam = LaunchConfiguration("use_slam")
    alphabot_controller_pkg = get_package_share_directory('alphabot_controller')

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )


    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("alphabot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )

    
    

    return LaunchDescription(
        [
            robot_model_arg,
            use_slam_arg,
            localization,
            slam,
            navigation
            # robot_localization_launch,
        ]
    )
