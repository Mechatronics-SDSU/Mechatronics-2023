cmake_minimum_required(VERSION 3.5)
project(mediator_node)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(scion_types REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(mediator_exec src/mediator_node.cpp)

ament_target_dependencies(mediator_exec scion_types rclcpp rclcpp_action std_srvs)

target_include_directories(mediator_exec PUBLIC ../../../classes/pid_controller/include
                                                ../control_interface/include)

install(TARGETS
  mediator_exec
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
