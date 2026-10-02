cmake_minimum_required(VERSION 3.16)
project(lego_dualarm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/kill_sim.sh scripts/demo_view.py scripts/demo_windows.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
