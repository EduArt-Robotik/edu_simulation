# Object Detection Sensor Plugin
add_library(eduard-object-detection-sensor-plugin SHARED
  object_detection_sensor.cpp
)
target_compile_features(eduard-object-detection-sensor-plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
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

# Eduard Model Plugin
add_library(eduard-model-plugin SHARED
  eduard_model_plugin.cpp
  eduard_gazebo_bot.cpp
)
target_compile_features(eduard-model-plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduard-model-plugin
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(eduard-model-plugin
  "gazebo_dev"
  "rclcpp"
  "gazebo_ros"
  "geometry_msgs"
  "sensor_msgs"
  "edu_robot"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
)
