cmake_minimum_required(VERSION 3.5)
project(ros2_ws)

# Set the C++ standard to use (e.g., C++17)
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

get_filename_component(PARENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(PARENT_PARENT_DIR "${PARENT_DIR}" DIRECTORY)

# Add gtest library
add_subdirectory(googletest)
add_subdirectory(${PARENT_PARENT_DIR}/classes ${CMAKE_BINARY_DIR}/classes-build)

# Add your package's source files
add_executable(
    test_exec 
    brain_node/src/brain_test.cpp
    brain_node/src/brain_node.cpp)

# Link gtest and other libraries to your executable
target_link_libraries(test_exec
  classes
  gtest
  gtest_main
  ${rclcpp_LIBRARIES}
)

target_include_directories(
    test_exec PUBLIC 
    brain_node/include
    control_interface/include 
    )

ament_target_dependencies(test_exec scion_types rclcpp rclcpp_action std_srvs std_msgs)

# Install the executable
install(TARGETS test_exec
  DESTINATION lib/${PROJECT_NAME}
)

# Don't forget to add other install rules or ament_target_dependencies if necessary

ament_package()
