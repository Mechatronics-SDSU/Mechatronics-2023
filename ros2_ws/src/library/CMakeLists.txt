cmake_minimum_required(VERSION 3.5)
project(library)

# Set the C++ standard to use (e.g., C++17)
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

get_filename_component(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(ROS_DIR "${SRC_DIR}" DIRECTORY)
get_filename_component(ROOT_DIR "${ROS_DIR}" DIRECTORY)

add_subdirectory(${ROOT_DIR}/classes ${CMAKE_BINARY_DIR}/classes-build)

# Add your package's source files
add_library(
    nodes 
    ${SRC_DIR}/brain_node/src/brain_node.cpp
    ${SRC_DIR}/control_interface/src/control_interface.cpp
    # ${SRC_DIR}/controller_node/src/controller_node.cpp
    # ${SRC_DIR}/current_state_node/src/current_state_node.cpp
    # ${SRC_DIR}/mediator_node/src/mediator_node.cpp
    ${SRC_DIR}/pid_node/src/pid_node.cpp
)

# Link gtest and other libraries to  executable
target_link_libraries(nodes
  classes
  ${rclcpp_LIBRARIES}
)

target_include_directories(
    nodes PUBLIC 
    ${SRC_DIR}/brain_node/include
    ${SRC_DIR}/control_interface/include
    # ${SRC_DIR}/controller_node/include
    # ${SRC_DIR}/current_state_node/include
    # ${SRC_DIR}/gui_node/include
    # ${SRC_DIR}/mediator_node/include
    ${SRC_DIR}/pid_node/include
)

ament_target_dependencies(nodes scion_types rclcpp rclcpp_action std_srvs std_msgs)

# Install the executable
install(TARGETS nodes
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
