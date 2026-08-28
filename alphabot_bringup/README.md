# alphabot_bringup

Bringup launch files for the alphabot workspace. This workspace supports
multiple robot models (currently `alphabot` and `servicebot`) selected via the
`ROBOT_MODEL` environment variable or the `robot_model` launch argument.

## Selecting a robot model

The robot model is resolved with the following precedence:

1. `robot_model` launch argument passed on the command line (highest priority).
2. `ROBOT_MODEL` environment variable.
3. Default: `alphabot`.

Valid values are defined in `alphabot_bringup/launch/robot_model.py`:

```python
VALID_ROBOT_MODELS = ['alphabot', 'servicebot']
DEFAULT_ROBOT_MODEL = 'alphabot'
```

An invalid value (via either the env var or the launch argument) raises a
`ValueError` listing the valid choices.

### Using the environment variable

```bash
export ROBOT_MODEL=servicebot
ros2 launch alphabot_bringup simulated_robot.launch.py
```

Set this once per shell/session (or in your `.bashrc`) to make all bringup,
description, and firmware launch files default to that model.

### Using the launch argument

```bash
ros2 launch alphabot_bringup simulated_robot.launch.py robot_model:=servicebot
```

This overrides the environment variable for that single launch invocation.

## What changes per model

Selecting a model changes which files are loaded, all under the same
installed packages (no per-model packages):

| Resource | Path pattern |
|---|---|
| Robot URDF/xacro | `alphabot_description/urdf/<model>/robot.urdf.xacro` |
| Meshes | `alphabot_description/meshes/<model>/` |
| RViz config | `alphabot_description/rviz/<model>.rviz` |
| Controller manager config | `alphabot_controller/config/<model>/controllers.yaml` |

The `ros2_control` controller instance name (`alphabot_controller`) and all
topic names (`/cmd_vel`, `/odom`, twist_mux, EKF, etc.) stay the same across
models, so downstream packages (navigation, localization, teleop) work
unchanged regardless of which model is selected.

## Launch files

- `bringup.launch.py` — real hardware bringup (firmware, controllers, sensors, twist_mux, EKF).
- `real_robot.launch.py` — real hardware bringup + localization/SLAM/navigation.
- `simulated_robot.launch.py` — Gazebo simulation + controllers + localization/SLAM/navigation.
- `navigator.launch.py` — localization/SLAM/navigation only (accepts `robot_model` for consistency; not currently used to load model-specific config).

Each of these declares a `robot_model` launch argument (defaulting from
`ROBOT_MODEL`) and forwards it to the `alphabot_description` and
`alphabot_firmware` launch files it includes.
