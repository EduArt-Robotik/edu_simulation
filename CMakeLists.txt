cmake_minimum_required(VERSION 3.5)
project(edu_simulation)

# ROS 2 Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(edu_robot REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(laser_geometry REQUIRED)

# Gazebo Packages
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

# General Compiler Settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)

add_subdirectory(src)

install(
  TARGETS
    # collision_avoidance_lidar_node
    # eduard-object-detection-sensor-plugin
    eduard-model-plugin
    eduard-motor-plugin
  DESTINATION lib
)

install(
  DIRECTORY
    model
    world
    parameter
    config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_libraries(
  eduard-model-plugin
)

ament_package()
