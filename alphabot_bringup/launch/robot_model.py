"""Shared ROBOT_MODEL selection helper used by alphabot_bringup launch files.

Mirrors the TURTLEBOT3_MODEL-style pattern: set the ROBOT_MODEL environment
variable to pick which robot the bringup stack targets, with a CLI
`robot_model:=<name>` launch argument override always available.

    export ROBOT_MODEL=servicebot
    ros2 launch alphabot_bringup simulated_robot.launch.py

    ros2 launch alphabot_bringup simulated_robot.launch.py robot_model:=servicebot

This module is a plain sibling file (not an installed Python package) so it
is imported the same way from every launch file in this directory:

    import os, sys
    sys.path.append(os.path.dirname(__file__))
    from robot_model import get_robot_model, VALID_ROBOT_MODELS
"""
import os

VALID_ROBOT_MODELS = ['alphabot', 'servicebot']
DEFAULT_ROBOT_MODEL = 'alphabot'


def get_robot_model():
    """Read ROBOT_MODEL from the environment (default: 'alphabot').

    Raises ValueError if the value is not one of VALID_ROBOT_MODELS.
    """
    model = os.environ.get('ROBOT_MODEL', DEFAULT_ROBOT_MODEL)
    if model not in VALID_ROBOT_MODELS:
        raise ValueError(
            f"Invalid ROBOT_MODEL '{model}'. Choose from {VALID_ROBOT_MODELS}"
        )
    return model
