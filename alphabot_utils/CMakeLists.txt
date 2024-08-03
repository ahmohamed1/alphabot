cmake_minimum_required(VERSION 3.8)
project(alphabot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(twist_mux_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
target_link_libraries(node
  ${OpenCV_LIBS}
)


add_subdirectory (../lib lib)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(safety_stop src/safety_stop.cpp)
ament_target_dependencies(safety_stop rclcpp std_msgs sensor_msgs geometry_msgs twist_mux_msgs rclcpp_action visualization_msgs)

add_executable(robot_drive src/robot_drive.cpp)
ament_target_dependencies(robot_drive rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

add_executable(tack_robot_position src/tack_robot_position.cpp)
ament_target_dependencies(tack_robot_position rclcpp std_msgs sensor_msgs cv_bridge)

ament_python_install_package(${PROJECT_NAME})


install(TARGETS
  safety_stop
  robot_drive
  tack_robot_position
  DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS
  ${PROJECT_NAME}/safety_stop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
