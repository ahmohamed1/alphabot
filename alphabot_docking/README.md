# alphabot_docking

AprilTag detection package for docking.

## What it does
- Runs `apriltag_ros` to detect tags in the camera feed.
- Publishes a `geometry_msgs/PoseStamped` for the selected tag on `/docking/target_pose`.

## Launch
```bash
ros2 launch alphabot_docking alphabot_docking.launch.py
```

### Optional arguments
```bash
ros2 launch alphabot_docking alphabot_docking.launch.py \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info
```

## Parameters
- `config/apriltag.yaml` controls the detector parameters.
- `target_tag_id` defaults to `0` inside the launch file; change as needed.
