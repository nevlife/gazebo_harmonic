cmake_minimum_required(VERSION 3.8)
project(scout_base)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
