cmake_minimum_required(VERSION 3.5)
project(pid_node)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/home/mechatronics/nodes/Mechatronics-2023/ros2_ws")


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

include_directories(include)

#../ - src
#../../ - ros2_ws
#../../../ - Project directory

# Must include headers and source files from pid_controller class in classses/
add_executable(pid_exec src/pid_controller_node.cpp
                        ../can2ros_driver/src/mbox_can.cpp
                        ../../../classes/pid_controller/src/pid_controller.cpp
                        ../../../classes/pid_controller/src/scion_pid_controller.cpp
                        ../../../classes/pid_controller/src/pid_params.cpp)
                              

ament_target_dependencies(pid_exec rclcpp scion_types std_msgs)

target_include_directories(pid_exec PUBLIC ../../../classes/pid_controller/include
                                           ../control_interface/include
                                           ../can2ros_driver/include)

install(TARGETS
  pid_exec
  DESTINATION lib/${PROJECT_NAME})

ament_package()
