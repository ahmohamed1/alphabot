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

pkg_check_modules(SERIAL libserial)

# find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED)


# include_directories(
#   include
#   ${OpenCV_INCLUDE_DIRS}
# )

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


add_executable(safety_stop src/safety_stop.cpp)
ament_target_dependencies(safety_stop rclcpp std_msgs sensor_msgs geometry_msgs twist_mux_msgs rclcpp_action visualization_msgs)

add_executable(robot_drive src/robot_drive.cpp)
ament_target_dependencies(robot_drive rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

add_executable(imu_publisher src/imu_publisher.cpp)
ament_target_dependencies(imu_publisher rclcpp std_msgs sensor_msgs)


add_executable(follow_wall src/follow_wall.cpp)
ament_target_dependencies(follow_wall rclcpp geometry_msgs sensor_msgs)

target_include_directories(imu_publisher PUBLIC
 "${SERIAL_INCLUDE_DIRS}")
 target_link_libraries(imu_publisher
 "${SERIAL_LDFLAGS}")

# add_executable(tack_robot_position src/tack_robot_position.cpp)
# ament_target_dependencies(tack_robot_position rclcpp std_msgs sensor_msgs cv_bridge)

# target_link_libraries(tack_robot_position
#   ${OpenCV_LIBS}
# )

ament_python_install_package(${PROJECT_NAME})


install(TARGETS
  safety_stop
  robot_drive
  imu_publisher
  follow_wall
  # tack_robot_position
  DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS
  ${PROJECT_NAME}/safety_stop.py
  ${PROJECT_NAME}/virtual_joystick.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
