cmake_minimum_required(VERSION 3.8)
project(alphabot_mpu6050)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)

# COMPILE
add_library(mpu6050_hardware_interface SHARED
  src/mpu6050_hardware_interface.cpp
  src/mpu6050_lib.cpp
)

target_include_directories(mpu6050_hardware_interface PRIVATE	include)

ament_target_dependencies(mpu6050_hardware_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Link i2c library to mpu6050_hardware_interface target
target_link_libraries(mpu6050_hardware_interface i2c)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface mpu6050_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/mpu6050_hardware_interface
)

install(
  TARGETS mpu6050_hardware_interface
  DESTINATION share/${PROJECT_NAME}
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# EXPORTS
ament_export_libraries(mpu6050_hardware_interface)

include_directories(include)


ament_package()
