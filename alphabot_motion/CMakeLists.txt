cmake_minimum_required(VERSION 3.8)
project(alphabot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

ament_python_install_package(${PROJECT_NAME})

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
)

add_library(pd_motion_planner SHARED src/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner ${dependencies})

# add_library(pure_pursuit SHARED src/pure_pursuit.cpp)
# ament_target_dependencies(pure_pursuit ${dependencies})

pluginlib_export_plugin_description_file(nav2_core motion_planner_plugins.xml)

add_executable(pure_pursuit src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS
  pd_motion_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)


install(TARGETS
  pd_motion_planner
  pure_pursuit
  DESTINATION lib/${PROJECT_NAME}
)



ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclpy
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_export_libraries(
  pd_motion_planner
  # pure_pursuit
)

install(PROGRAMS
  ${PROJECT_NAME}/absolute_move.py 
  ${PROJECT_NAME}/publish_cmd_vel.py 
  ${PROJECT_NAME}/relative_move.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
