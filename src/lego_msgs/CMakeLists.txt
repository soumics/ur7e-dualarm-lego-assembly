cmake_minimum_required(VERSION 3.16)
project(lego_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# BrickState, AssemblyAction and ExecuteAssembly are added when the world model
# and planner (Phase 5) need them.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BrickDetection.msg"
  "msg/BrickDetectionArray.msg"
  "srv/RequestPlace.srv"
  "srv/ReleaseZone.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
