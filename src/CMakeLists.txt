# Object Detection Sensor Plugin
add_library(eduard-object-detection-sensor-plugin SHARED
  object_detection_sensor.cpp
)
target_include_directories(eduard-object-detection-sensor-plugin
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(eduard-object-detection-sensor-plugin
  "gazebo_dev"
  "rclcpp"
  "gazebo_ros"
  "geometry_msgs"
)
