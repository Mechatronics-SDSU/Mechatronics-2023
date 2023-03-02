cmake_minimum_required(VERSION 3.5)
project(can2ros_driver)

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

include_directories(include)

add_executable(
	can2ros_driver
	src/can_mbox_pub.cpp
	src/can_send_service.cpp
	src/mbox_can.cpp
	src/can_mailbox_topic.cpp
)
ament_target_dependencies(can2ros_driver rclcpp scion_types)

install(TARGETS
	can2ros_driver
	DESTINATION lib/${PROJECT_NAME})

ament_package()
