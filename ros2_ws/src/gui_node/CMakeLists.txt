cmake_minimum_required(VERSION 3.5)

project(gui_node VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(STYLESHEET_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/styles.qss")

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)

include_directories(include)

add_executable(gui_exec src/main.cpp
        include/mainwindow.hpp
        src/mainwindow.cpp
        src/mainwindow.ui
)

target_include_directories(gui_exec PUBLIC include)

ament_target_dependencies(gui_exec PUBLIC rclcpp scion_types)

target_link_libraries(gui_exec PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

set_target_properties(gui_exec PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

install(TARGETS gui_exec
    DESTINATION lib/${PROJECT_NAME}
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

ament_package()