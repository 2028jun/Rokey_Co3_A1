cmake_minimum_required(VERSION 3.8)
project(serving_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TaskCommand.srv"
  "srv/OrderRequest.srv"
)

ament_package()
