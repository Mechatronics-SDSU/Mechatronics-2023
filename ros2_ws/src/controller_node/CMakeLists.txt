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

add_executable(controller_exec src/controller_node.cpp
                               ../can2ros_driver/src/mbox_can.cpp)


ament_target_dependencies(controller_exec rclcpp scion_types std_msgs sensor_msgs)

target_include_directories(controller_exec PUBLIC ../../../classes/pid_controller/include
                                                  ../can2ros_driver/include)

                                           
install(TARGETS
  controller_exec
  DESTINATION lib/${PROJECT_NAME})


ament_package()
