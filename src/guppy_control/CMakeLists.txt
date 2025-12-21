cmake_minimum_required(VERSION 3.8)
project(guppy_control)

# Default to C++17 for ROS2 Jazzy compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  endif()
  
  # Enable warnings
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()
    
    find_package(ament_cmake REQUIRED)
    find_package(proxsuite REQUIRED)
    find_package(eigen3_cmake_module REQUIRED)
    find_package(Eigen3)    

    # Throttle publisher executable
    add_executable(prox_test src/prox_test.cpp)
    ament_target_dependencies(prox_test Eigen3 proxsuite)


    install(TARGETS
      prox_test

      DESTINATION lib/${PROJECT_NAME}
    )
    ament_package()