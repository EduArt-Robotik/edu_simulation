cmake_minimum_required(VERSION 3.5)
project(edu_simulation)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(edu_robot REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(laser_geometry REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)

add_subdirectory(src)

ament_package()

install(
  TARGETS
    collision_avoidance_lidar_node
    eduard-object-detection-sensor-plugin
    eduard-model-plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    launch/spawn_eduard.py
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    model
    world
    parameter
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
