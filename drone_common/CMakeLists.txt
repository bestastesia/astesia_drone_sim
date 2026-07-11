cmake_minimum_required(VERSION 3.8)
project(drone_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_common test/test_common.cpp)
  target_link_libraries(test_common drone_common)
  target_include_directories(test_common PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIRS}
  )
endif()

# header-only INTERFACE library so downstream can link drone_common::drone_common
add_library(drone_common INTERFACE)
target_include_directories(drone_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(drone_common INTERFACE Eigen3::Eigen)

install(TARGETS drone_common
  EXPORT drone_common_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(drone_common_targets)
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)

ament_package()