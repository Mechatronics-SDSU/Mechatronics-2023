cmake_minimum_required(VERSION 3.5)

project(test_gui LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

if(ANDROID)
   set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
   if (ANDROID_ABI STREQUAL "armeabi-v7a")
       set(ANDROID_EXTRA_LIBS
           ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
           ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
   endif()
endif()

find_package(Qt5 COMPONENTS Widgets REQUIRED)
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(scion_types REQUIRED)
# find_package(std_msgs REQUIRED)

include_directories(include)


  #  add_library(test_gui SHARED
  #    src/main.cpp
  #    src/mainwindow.cpp
  #    include/mainwindow.h
  #    include/mission_planning.h
  #    mainwindow.ui
  #  )
add_executable(test_gui_exec
  src/main.cpp
  src/mainwindow.cpp
  src/mission_planning.cpp
  src/pid_controller.cpp
  include/mainwindow.hpp
  include/pid_controller.hpp
  include/mission_planning.hpp
  src/mainwindow.ui
  src/mission_planning.ui
  src/pid_controller.ui

)


# target_include_directories(test_gui_exec PUBLIC include
#                                     ../control_interface/include)

# ament_target_dependencies(test_gui_exec PUBLIC rclcpp scion_types std_msgs)

target_link_libraries(test_gui_exec PRIVATE Qt5::Widgets)

# set_target_properties(test_gui_exec PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
#    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
#    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
#    MACOSX_BUNDLE TRUE
#    WIN32_EXECUTABLE TRUE
# )

# install(TARGETS test_gui_exec
#     DESTINATION lib/${PROJECT_NAME}
#     BUNDLE DESTINATION .
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )

# ament_package()
