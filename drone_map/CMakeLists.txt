cmake_minimum_required(VERSION 3.8)
project(drone_map)

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
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(drone_common REQUIRED)

add_executable(map_node src/map_node.cpp)
target_include_directories(map_node PRIVATE include)
target_link_libraries(map_node drone_common::drone_common)
ament_target_dependencies(map_node
  rclcpp std_msgs geometry_msgs visualization_msgs
)

install(TARGETS map_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
