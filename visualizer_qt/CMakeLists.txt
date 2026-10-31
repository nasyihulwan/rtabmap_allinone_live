cmake_minimum_required(VERSION 3.8)
project(visualizer_qt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Qml Quick REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

add_executable(visualizer_qt_node
  src/main.cpp
  src/ros_bridge.cpp
  src/cv_image_item.cpp
  src/position_log_watcher.cpp
  src/session_meta.cpp
  src/status_strip.cpp
  src/rosout_log.cpp
  src/map_panel.cpp
  include/visualizer_qt/ros_bridge.h
  include/visualizer_qt/cv_image_item.h
)
target_include_directories(visualizer_qt_node PRIVATE include)
target_link_libraries(visualizer_qt_node
  Qt5::Core
  Qt5::Gui
  Qt5::Qml
  Qt5::Quick
  ${OpenCV_LIBS}
  ${YAML_CPP_TARGET}
)
ament_target_dependencies(visualizer_qt_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  nav_msgs
  rcl_interfaces
  tf2
  tf2_ros
  OpenCV
)

install(TARGETS visualizer_qt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY qml
  DESTINATION share/${PROJECT_NAME}
)

# Husky A200 meshes (clearpath_platform_description), same files the
# web_dashboard React/Three.js viewer uses — copied here, not referenced
# via a path into web_dashboard/, so this package doesn't depend on a
# sibling directory existing at a fixed relative path.
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
