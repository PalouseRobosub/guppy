cmake_minimum_required(VERSION 3.8)
project(can_motor_control)

# Default to C++17 for ROS2 Jazzy compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  endif()
  
  # Enable warnings
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()
    
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    
    # Throttle publisher executable
    add_executable(throttle_publisher src/throttle_publisher.cpp)
    ament_target_dependencies(throttle_publisher rclcpp std_msgs)
    
    # CAN throttle subscriber executable
    add_executable(can_throttle_subscriber src/can_throttle_subscriber.cpp)
    ament_target_dependencies(can_throttle_subscriber rclcpp std_msgs)
    
    install(TARGETS
      throttle_publisher
        can_throttle_subscriber
          DESTINATION lib/${PROJECT_NAME})
          
          ament_package()
          
