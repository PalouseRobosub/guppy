cmake_minimum_required(VERSION 3.8)
project(guppy_safety)

# Default to C++17 for ROS2 Jazzy compatibility
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rclcpp REQUIRED)

# Message types
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(guppy_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add executables
add_executable(checker src/checker.cpp)
ament_target_dependencies(checker rclcpp std_msgs geometry_msgs guppy_msgs nav_msgs)
add_executable(responder src/responder.cpp)
ament_target_dependencies(responder rclcp)
add_executable(heartbeat src/heartbeat.cpp)
ament_target_dependencies(heartbeat rclcpp std_msgs geometry_msgs guppy_msgs)

install(
  TARGETS
    checker
    responder
    heartbeat
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
