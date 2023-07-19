cmake_minimum_required(VERSION 3.5)
project(filter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_executable(filter src/filter.cpp)

target_include_directories(filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                                   ../pid_controller/include)

install(TARGETS
  filter
  DESTINATION lib/${PROJECT_NAME})