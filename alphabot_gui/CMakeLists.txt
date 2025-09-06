cmake_minimum_required(VERSION 3.8)
project(alphabot_gui)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(twist_mux_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)


set(PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  nav_msgs
)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Enable Qt automatic processing
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


set(headers_to_moc
  include/alphabot_gui/mainWindow.hpp
)

set(headers_to_moc
  include/alphabot_gui/mainWindow.hpp
)

foreach(header "${headers_to_moc}")
  qt5_wrap_cpp(moc_files "${header}")
endforeach()

set(SRC_FILES
  src/mainWindow.cpp
  src/main.cpp
  ${moc_files}
)

# Executable
add_executable(alphabot_gui_node ${SRC_FILES})
ament_target_dependencies(alphabot_gui_node ${PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(alphabot_gui_node
  Qt5::Core
  Qt5::Widgets
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

# Install target
install(TARGETS alphabot_gui_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install(PROGRAMS
#   ${PROJECT_NAME}/main.py
#   DESTINATION lib/${PROJECT_NAME}
# )
ament_package()
