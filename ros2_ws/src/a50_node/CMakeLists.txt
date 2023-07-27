cmake_minimum_required(VERSION 3.5)
project(a50_node)

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

include_directories(include)

add_executable(a50_exec 
    src/a50_node.cpp
    src/main.cpp
)

target_link_libraries(a50_exec
  classes
  ${rclcpp_LIBRARIES}
)

ament_target_dependencies(a50_exec rclcpp scion_types)

target_include_directories(
    a50_exec PUBLIC include ${CMAKE_CURRENT_SOURCE_DIR}
    ${SRC_DIR}/control_interface/include
)


install(TARGETS
  a50_exec
  DESTINATION lib/${PROJECT_NAME})

ament_package()
