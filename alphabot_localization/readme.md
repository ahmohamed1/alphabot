## Building and Using a Map with SLAM Toolbox

To build a map using SLAM Toolbox, you need to copy the `mapper_parameters` and adjust them to suit your needs.

### Building the Map

To build the map, you can run the following command:

```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=src/alphabot/alphabot_localization/config/mapper_params_online_async.yaml use_sim_time:=true
```

## Loading an Existing Map
To load an existing map, modify the mapper_params_online_async.yaml file:

```bash
mode: mapping # Change this to localization
map_file_name: path_to_your_map_file # Specify the path to your map file
```

Replace path_to_your_map_file with the actual path to your map file, for example: map_file_name: test_steve.

```bash
Feel free to adjust the paths and filenames as needed for your specific project setup.
```