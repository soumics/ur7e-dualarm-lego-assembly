cmake_minimum_required(VERSION 3.16)
project(lego_gz_plugins)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

add_library(lego_grasp_lock SHARED src/grasp_lock.cpp)
target_compile_features(lego_grasp_lock PUBLIC cxx_std_17)
target_link_libraries(lego_grasp_lock
  gz-sim::gz-sim
  gz-plugin::register
  gz-transport::gz-transport
  gz-msgs::gz-msgs
)

install(TARGETS lego_grasp_lock LIBRARY DESTINATION lib)

# Lets Gazebo find the system plugin by filename.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
