cmake_minimum_required(VERSION 3.8)
project(m0609_isaac_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Reuse the detailed RG2 meshes that are also used by the Isaac Sim asset.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../onrobot_rg2/meshes/
  DESTINATION share/${PROJECT_NAME}/onrobot_rg2
)

ament_package()
