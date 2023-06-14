cmake_minimum_required(VERSION 3.5)
project(pid_node)

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
# find_package(ament_cmake REQUIRED)

include_directories(include)

#../ - src
#../../ - ros2_ws
#../../../ - Project directory

# Must include headers and source files from pid_controller class in classses/
add_executable(pid_test pid_controller/pid_controller.cpp
                        ../../classes/pid_controller/src/pid_controller.cpp
                        ../../classes/pid_controller/src/scion_pid_controller.cpp
                        ../../classes/pid_controller/src/pid_params.cpp)
                              

# ament_target_dependencies(pid_test)

target_include_directories(pid_test PUBLIC ../../classes/pid_controller/include
                                           )

install(TARGETS
  pid_test
  DESTINATION lib/${PROJECT_NAME})

# ament_package()