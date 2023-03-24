cmake_minimum_required(VERSION 3.5)
project(zed_orientation_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)     
find_package(scion_types REQUIRED) 
find_package(rosidl_default_generators REQUIRED)

add_executable(zed_orientation_exec src/zed_orientation_node.cpp)

ament_target_dependencies(zed_orientation_exec rclcpp scion_types geometry_msgs) 

ament_package()
