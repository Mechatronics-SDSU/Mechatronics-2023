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

add_executable(pid_exec src/pid_controller_node.cpp
                              /home/mechatronics/Mechatronics-2023/classes/pid_controller/src/pid_controller.cpp
                              /home/mechatronics/Mechatronics-2023/classes/pid_controller/src/scion_pid_controller.cpp
                              /home/mechatronics/Mechatronics-2023/classes/pid_controller/src/pid_params.cpp)

ament_target_dependencies(pid_exec rclcpp scion_types)

target_include_directories(pid_exec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
          /home/mechatronics/nodes/Mechatronics-2023/classes/pid_controller/include)

install(TARGETS
  pid_exec
  DESTINATION lib/${PROJECT_NAME})


ament_package()
