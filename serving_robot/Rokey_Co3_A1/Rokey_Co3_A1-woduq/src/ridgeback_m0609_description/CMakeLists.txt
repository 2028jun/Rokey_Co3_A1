cmake_minimum_required(VERSION 3.8)
project(ridgeback_m0609_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  meshes
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/validate_description.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
