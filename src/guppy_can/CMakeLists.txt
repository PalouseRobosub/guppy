cmake_minimum_required(VERSION 3.8)
project(guppy_can)

# Default to C++17 for ROS2 Jazzy compatibility
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# Enable warnings
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(guppy_msgs REQUIRED)

# CAN RX publisher executable
add_executable(can_rx_publisher src/can_rx_publisher.cpp)
ament_target_dependencies(can_rx_publisher rclcpp guppy_msgs)

# CAN TX service executable
add_executable(can_tx_server src/can_tx_server.cpp)
ament_target_dependencies(can_tx_server rclcpp guppy_msgs)

install(TARGETS
        can_rx_publisher
        can_tx_server

        DESTINATION lib/${PROJECT_NAME})

ament_package()