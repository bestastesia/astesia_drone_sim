cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz scripts
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Python scripts as executables (so ros2 run can find them)
install(PROGRAMS
  scripts/waypoint_sender.py
  scripts/trajectory_generator.py
  scripts/auto_eval.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()