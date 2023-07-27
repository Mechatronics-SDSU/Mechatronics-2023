cmake_minimum_required(VERSION 3.5)
project(brain_node)

# add_compile_options(-g)

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

# Must include headers and source files from pid_controller class in classses/
add_executable(
    brain_exec 
    src/brain_node.cpp
    src/main.cpp
)

target_link_libraries(
    brain_exec
    classes
    ${rclcpp_LIBRARIES}
)

ament_target_dependencies(
    brain_exec 
    rclcpp 
    scion_types 
    std_msgs 
    std_srvs 
    rclcpp_action
)

install(TARGETS
    brain_exec
    DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(
    brain_exec PUBLIC include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SRC_DIR}/control_interface/include
)
                                             
ament_package()
