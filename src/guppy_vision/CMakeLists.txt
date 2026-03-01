cmake_minimum_required(VERSION 3.8)
project(guppy_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Spinnaker paths
set(SPINNAKER_INCLUDE_DIR /opt/spinnaker/include)
set(SPINNAKER_LIBRARY /opt/spinnaker/lib/libSpinnaker.so)

# Create imported library target
add_library(Spinnaker::Spinnaker SHARED IMPORTED)

set_target_properties(Spinnaker::Spinnaker PROPERTIES
        IMPORTED_LOCATION ${SPINNAKER_LIBRARY}
        INTERFACE_INCLUDE_DIRECTORIES ${SPINNAKER_INCLUDE_DIR}
)

#add_executable(ack
#        src/ack.cpp)
#
add_executable(camera_manager src/camera_manager.cpp)

add_executable(camera_pub src/camera_pub.cpp)
ament_target_dependencies(camera_pub rclcpp sensor_msgs)

#target_link_libraries(ack
#        Spinnaker::Spinnaker
#)
#
target_link_libraries(camera_manager
        Spinnaker::Spinnaker
)

target_link_libraries(camera_pub
        Spinnaker::Spinnaker
)

install(TARGETS
        camera_pub

        DESTINATION lib/${PROJECT_NAME})

ament_package()
