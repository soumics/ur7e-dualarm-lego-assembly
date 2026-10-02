cmake_minimum_required(VERSION 3.16)
project(lego_dualarm_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
