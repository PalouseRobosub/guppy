cmake_minimum_required(VERSION 3.5)
project(guppy_localization)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dvl_msgs REQUIRED)

add_executable(barometer_pub src/barometer_pub.cpp)
add_executable(dvl_a50_pub src/dvl_a50_pub.cpp)
ament_target_dependencies(barometer_pub dvl_a50_pub rclcpp geometry_msgs std_msgs dvl_msgs)

install(TARGETS
  barometer_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()