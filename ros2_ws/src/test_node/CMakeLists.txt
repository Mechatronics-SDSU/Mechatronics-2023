cmake_minimum_required(VERSION 3.5)
project(test_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(scion_types REQUIRED)

add_executable(test_exec src/test_node.cpp)

ament_target_dependencies(test_exec scion_types rclcpp)

target_include_directories(test_exec PUBLIC ../control_interface/include)

install(TARGETS
  test_exec
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
