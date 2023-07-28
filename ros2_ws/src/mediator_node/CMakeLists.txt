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

get_filename_component(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(ROS_DIR "${SRC_DIR}" DIRECTORY)
get_filename_component(ROOT_DIR "${ROS_DIR}" DIRECTORY)

add_subdirectory(${ROOT_DIR}/classes classes_binary)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(scion_types REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(
  mediator_exec 
  src/mediator_node.cpp
  src/main.cpp
  ${SRC_DIR}/control_interface/src/control_interface.cpp
)
  
ament_target_dependencies(mediator_exec scion_types rclcpp rclcpp_action std_srvs)

target_link_libraries(
    mediator_exec
    classes
    ${rclcpp_LIBRARIES}
)

target_include_directories(
    mediator_exec PUBLIC include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SRC_DIR}/control_interface/include
)

install(TARGETS
  mediator_exec
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
