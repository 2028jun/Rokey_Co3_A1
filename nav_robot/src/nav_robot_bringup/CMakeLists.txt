cmake_minimum_required(VERSION 3.8)
project(nav_robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config behavior_trees rviz
  DESTINATION share/${PROJECT_NAME}
)

# Optional: ship a copy of the restaurant map next to the package for convenience.
set(NAV_ROBOT_MAP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/restaurant")
if(EXISTS "${NAV_ROBOT_MAP_DIR}/map.yaml")
  install(DIRECTORY "${NAV_ROBOT_MAP_DIR}/"
    DESTINATION share/${PROJECT_NAME}/maps/restaurant
  )
endif()

ament_package()
