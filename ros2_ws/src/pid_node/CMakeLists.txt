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

get_filename_component(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(ROS_DIR "${SRC_DIR}" DIRECTORY)
get_filename_component(ROOT_DIR "${ROS_DIR}" DIRECTORY)

add_subdirectory(${ROOT_DIR}/classes classes_binary)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include)

add_executable(
  pid_exec 
  src/pid_node.cpp
  src/main.cpp
  ${SRC_DIR}/control_interface/src/control_interface.cpp
)
  
ament_target_dependencies(pid_exec scion_types rclcpp rclcpp_action std_msgs std_srvs)

target_link_libraries(
    pid_exec
    classes
    ${rclcpp_LIBRARIES}
)

target_include_directories(
    pid_exec PUBLIC include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SRC_DIR}/control_interface/include
)

install(TARGETS
  pid_exec
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
