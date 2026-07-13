cmake_minimum_required(VERSION 3.8)
project(drone_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(drone_common REQUIRED)

add_executable(controller_node src/controller_node.cpp)
target_include_directories(controller_node PRIVATE include)
target_link_libraries(controller_node drone_common::drone_common)
ament_target_dependencies(controller_node
  rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)

install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
