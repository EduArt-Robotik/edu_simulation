cmake_minimum_required(VERSION 3.5)
project(edu_simulation)

find_package(ament_cmake REQUIRED)

# install(DIRECTORY world  
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()

install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
