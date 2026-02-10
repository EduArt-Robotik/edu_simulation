# Object Detection Sensor Plugin
# add_library(eduard-object-detection-sensor-plugin SHARED
#   object_detection_sensor.cpp
# )
# target_compile_features(eduard-object-detection-sensor-plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# target_include_directories(eduard-object-detection-sensor-plugin
# PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(eduard-object-detection-sensor-plugin
#   "gazebo_dev"
#   "rclcpp"
#   "gazebo_ros"
#   "geometry_msgs"
# )

# Eduard Model Plugin
add_library(eduard-model-plugin SHARED
  eduard_model_plugin.cpp
  eduard_gazebo_bot.cpp
  gazebo_motor_controller.cpp
  gazebo_imu_sensor.cpp
  gazebo_tof_sensor.cpp
  gazebo_tof_ring_sensor.cpp
  gazebo_virtual_range_sensor.cpp
  gazebo_lighting.cpp
  gazebo_hardware_adapter.cpp
  eduard_hardware_component_factory.cpp
)
target_include_directories(eduard-model-plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(eduard-model-plugin
  gz-sim::gz-sim
  gz-plugin::register
)
ament_target_dependencies(eduard-model-plugin
  "rclcpp"
  "edu_robot"
)

# Eduart Motor Plugin
add_library(eduard-motor-plugin SHARED
  gazebo_motor_plugin.cpp
)
target_include_directories(eduard-motor-plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(eduard-motor-plugin
  gz-sim::gz-sim
  gz-plugin::register
)
ament_target_dependencies(eduard-motor-plugin
  "edu_robot"
)

# Collision Avoidance Lidar
add_executable(collision_avoidance_lidar_node
  collision_avoidance_lidar_node.cpp
)
target_link_libraries(collision_avoidance_lidar_node

)
ament_target_dependencies(collision_avoidance_lidar_node
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_sensor_msgs"
  "laser_geometry"
)

# Export Pose Plugin
add_library(model-pose-exporter-plugin SHARED
  gazebo_pose_exporter.cpp
)
target_include_directories(model-pose-exporter-plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(model-pose-exporter-plugin
  gz-sim::gz-sim
  gz-plugin::register
)
ament_target_dependencies(model-pose-exporter-plugin
)

# Lighting Plugin
add_library(gazebo-lighting-plugin SHARED
  gazebo_lighting_plugin.cpp
)
target_include_directories(gazebo-lighting-plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(gazebo-lighting-plugin
  gz-sim::gz-sim
  gz-plugin::register
)
ament_target_dependencies(gazebo-lighting-plugin
  "edu_robot"
)
