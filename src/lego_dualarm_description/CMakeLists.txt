cmake_minimum_required(VERSION 3.16)
project(lego_dualarm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf worlds models rviz
  DESTINATION share/${PROJECT_NAME}
)

# Lets Gazebo resolve model://lego_brick_2x4 from this package.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
