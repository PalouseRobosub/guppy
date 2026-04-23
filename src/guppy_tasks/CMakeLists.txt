cmake_minimum_required(VERSION 3.8)
project(guppy_tasks)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(guppy_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(align src/align.cpp)

ament_target_dependencies(align rclcpp rclcpp_action geometry_msgs guppy_msgs eigen3_cmake_module Eigen3)

# install(PROGRAMS
#   guppy_tasks/
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS align
  DESTINATION lib/${PROJECT_NAME}
) 



ament_package()