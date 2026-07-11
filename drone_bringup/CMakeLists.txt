cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz scripts
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()