cmake_minimum_required(VERSION 3.8)
project(aruco_detector_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Зависимости
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Исполняемый файл
add_executable(detector_node src/detector_node.cpp)

ament_target_dependencies(detector_node
    rclcpp
    sensor_msgs
    cv_bridge
    geometry_msgs
)

target_link_libraries(detector_node ${OpenCV_LIBS})

# Установка
install(TARGETS
    detector_node
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

