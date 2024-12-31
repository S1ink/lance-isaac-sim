cmake_minimum_required(VERSION 3.5)
project(isaac_launch)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT MSVC)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joy_axis_exporter "src/joy_axis_exporter.cpp")
ament_target_dependencies(joy_axis_exporter "rclcpp" "std_msgs" "sensor_msgs")
ament_export_dependencies(rclcpp std_msgs sensor_msgs)

install(
    TARGETS joy_axis_exporter
    DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY assets config description launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
