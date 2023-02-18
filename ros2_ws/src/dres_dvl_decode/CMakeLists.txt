cmake_minimum_required(VERSION 3.5)
project(dres_dvl_decode)

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
find_package(can2ros_driver REQUIRED)

include_directories(include)

add_executable(
	dres_dvl_decode
	src/dres_decode_node.cpp
	src/dvl_handler.cpp
	src/mbox_can.cpp
)
ament_target_dependencies(dres_dvl_decode rclcpp scion_types can2ros_driver)

install(TARGETS
	dres_dvl_decode
	DESTINATION lib/${PROJECT_NAME})

ament_package()
