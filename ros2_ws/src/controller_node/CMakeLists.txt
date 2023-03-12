cmake_minimum_required(VERSION 3.5)
project(controller_node)

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
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
include_directories(/home/mechatronics/master/Mechatronics-2023/classes/pid_controller/include)

add_executable(controller_exec src/controller_node.cpp
                               src/mbox_can.cpp)
                              

ament_target_dependencies(controller_exec rclcpp scion_types std_msgs sensor_msgs)

install(TARGETS
  controller_exec
  DESTINATION lib/${PROJECT_NAME})


ament_package()
