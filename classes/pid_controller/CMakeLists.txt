cmake_minimum_required(VERSION 3.5)
project(pid_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_executable(pid_controller src/pid_controller.cpp
                              src/pid_params.cpp
                              src/scion_pid_controller.cpp)

target_include_directories(pid_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS
  pid_controller
  DESTINATION lib/${PROJECT_NAME})
