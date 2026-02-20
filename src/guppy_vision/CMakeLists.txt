cmake_minimum_required(VERSION 3.8)
project(guppy_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Spinnaker paths
set(SPINNAKER_INCLUDE_DIR /opt/spinnaker/include)
set(SPINNAKER_LIBRARY /opt/spinnaker/lib/libSpinnaker.so)

# Create imported library target
add_library(Spinnaker::Spinnaker SHARED IMPORTED)

set_target_properties(Spinnaker::Spinnaker PROPERTIES
        IMPORTED_LOCATION ${SPINNAKER_LIBRARY}
        INTERFACE_INCLUDE_DIRECTORIES ${SPINNAKER_INCLUDE_DIR}
)

add_executable(ack
        src/ack.cpp)

add_executable(camera_utils src/camera_utils.cpp)

target_link_libraries(ack
        Spinnaker::Spinnaker
)

target_link_libraries(camera_utils
        Spinnaker::Spinnaker
)

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

ament_package()
