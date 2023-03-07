cmake_minimum_required(VERSION 3.5)
project(edu_simulation)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_subdirectory(src)

# install(DIRECTORY world  
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()

install(TARGETS
    eduard-object-detection-sensor-plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/lib
)

install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
