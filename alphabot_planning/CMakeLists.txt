cmake_minimum_required(VERSION 3.8)
project(alphabot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)



include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  tf2_ros
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_msgs
  pluginlib
)

add_library(dijkstra_planner SHARED src/dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner ${dependencies})

add_library(astar_planner SHARED src/astar_planner.cpp)
ament_target_dependencies(astar_planner ${dependencies})

pluginlib_export_plugin_description_file(nav2_core global_planner_plugins.xml)

# add_executable(astar_planner src/astar_planner.cpp)
# ament_target_dependencies(astar_planner
#   rclcpp
#   nav_msgs
#   geometry_msgs
#   tf2_ros
# )

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION include
)

# install(TARGETS
#   astar_planner
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS
  dijkstra_planner
  astar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(FILES global_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_libraries(
  dijkstra_planner
  astar_planner
)
ament_export_dependencies(${dependencies})
ament_package()