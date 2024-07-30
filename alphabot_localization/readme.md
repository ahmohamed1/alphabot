## Building and Using a Map with SLAM Toolbox

To build a map using SLAM Toolbox, you need to copy the `mapper_parameters` and adjust them to suit your needs.

### Building the Map

To build the map, you can run the following command:

```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=src/alphabot/alphabot_localization/config/mapper_params_online_async.yaml use_sim_time:=true
```
### option: to run nav2 alongside of slam 

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

## Loading an Existing Map
To load an existing map, modify the mapper_params_online_async.yaml file:

```bash
mode: mapping # Change this to localization
map_file_name: path_to_your_map_file # Specify the path to your map file
```

Replace path_to_your_map_file with the actual path to your map file, for example: map_file_name: test_steve.

Feel free to adjust the paths and filenames as needed for your specific project setup.


## Self exploring the map while building the map

To automate the creation of the map we need to use [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2).

This package provide an algorithm to explore the map during creation by using the global cost map and analysis the frontier. Frontier refere unknow location in the map, for example, when runing slam_toolbox to create the map, the Nav2 can be run which provide for us the global cost map, which provide us with a location the show unkown (until know did not indeftify us wall or empty). To run the process:

1. Run the simulation of hardware.
2. run slam_toolbox.
```
ros2 launch slam_toolbox online_async_launch.py
```
3. run nav2 navigator
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False
```
4. run explore
```
ros2 launch explore_lite explore.launch.py
```