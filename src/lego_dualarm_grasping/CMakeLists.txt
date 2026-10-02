cmake_minimum_required(VERSION 3.16)
project(lego_dualarm_grasping)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lego_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pick_place_once src/pick_place_once.cpp)
target_compile_features(pick_place_once PUBLIC cxx_std_17)
ament_target_dependencies(pick_place_once
  rclcpp rclcpp_action moveit_ros_planning_interface control_msgs geometry_msgs
  moveit_msgs sensor_msgs shape_msgs tf2 tf2_geometry_msgs tf2_ros lego_msgs std_msgs)

install(TARGETS pick_place_once DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/run_trials.py scripts/dual_arm_timing.py scripts/stack_test.py DESTINATION lib/${PROJECT_NAME})

ament_package()
