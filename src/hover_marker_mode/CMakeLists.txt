cmake_minimum_required(VERSION 3.5)
project(hover_marker_mode)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(hover_mode_node
  src/main.cpp
  src/hover_mode.cpp
)

ament_target_dependencies(hover_mode_node
  rclcpp geometry_msgs px4_msgs px4_ros2_cpp
)

target_link_libraries(hover_mode_node
  Eigen3::Eigen
  px4_ros2_cpp::px4_ros2_cpp
)

install(TARGETS hover_mode_node DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

