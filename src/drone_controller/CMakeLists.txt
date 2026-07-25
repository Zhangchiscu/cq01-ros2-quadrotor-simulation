cmake_minimum_required(VERSION 3.8)
project(drone_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(
  quadrotor_controller_node
  src/quadrotor_controller_node.cpp
)

ament_target_dependencies(
  quadrotor_controller_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

target_link_libraries(
  quadrotor_controller_node
  Eigen3::Eigen
)

install(
  TARGETS quadrotor_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/send_goal
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
