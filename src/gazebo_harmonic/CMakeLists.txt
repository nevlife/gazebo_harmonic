cmake_minimum_required(VERSION 3.5)
project(gazebo_harmonic)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(gps_covariance_relay src/gps_covariance_relay.cpp)
ament_target_dependencies(gps_covariance_relay rclcpp sensor_msgs)

add_executable(vehicle_speed_publisher src/vehicle_speed_publisher.cpp)
ament_target_dependencies(vehicle_speed_publisher rclcpp nav_msgs std_msgs)

install(TARGETS gps_covariance_relay vehicle_speed_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch world urdf models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
