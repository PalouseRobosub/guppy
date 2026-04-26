cmake_minimum_required(VERSION 3.8)
project(guppy_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(guppy_msgs REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(control_toolbox REQUIRED)

add_library(action_server SHARED src/navigate_action_server.cpp)

#target_include_directories(action_server PRIVATE
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

include_directories(include)

#add_executable(action_server src/navigate_action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action rclcpp_components guppy_msgs nav_msgs geometry_msgs Eigen3 control_toolbox)
rclcpp_components_register_node(action_server PLUGIN "NavigateActionServer" EXECUTABLE navigate_action_server)

install(TARGETS
  action_server

  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  #DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
