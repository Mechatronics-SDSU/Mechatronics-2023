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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

include_directories(include)

# Must include headers and source files from pid_controller class in classses/
add_executable(brain_exec src/brain_node.cpp
                          src/brain_test.cpp
                          ../control_interface/src/control_interface.cpp
                          ../../../classes/filter/src/filter.cpp)

ament_add_gtest(brain_test src/brain_test.cpp src/brain_node.cpp)  # Need to link .cpp file under test

add_subdirectory(../googletest)
target_link_libraries(brain_exec
  gtest
  gtest_main
  gmock
  ${rclcpp_LIBRARIES}
)

ament_target_dependencies(brain_exec rclcpp scion_types std_msgs std_srvs rclcpp_action)

install(TARGETS
  brain_exec
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(brain_exec PUBLIC include
                                             ../../../classes/pid_controller/include
                                             ../../../classes/filter/include
                                             ../control_interface/include)

ament_package()
