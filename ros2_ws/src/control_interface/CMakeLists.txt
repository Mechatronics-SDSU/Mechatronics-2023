cmake_minimum_required(VERSION 3.5)
project(control_interface)

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

include_directories(include)

# Must include headers and source files from pid_controller class in classses/
add_executable(control_interface_exec src/control_interface.cpp)
                              
ament_target_dependencies(control_interface_exec rclcpp scion_types std_msgs std_srvs rclcpp_action)

install(TARGETS
  control_interface_exec
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(control_interface_exec PUBLIC include)

ament_package()