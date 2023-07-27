add_library(classes
    filter/src/filter.cpp
    pid_controller/src/pid_controller.cpp
    pid_controller/src/scion_pid_controller.cpp
    pid_controller/src/pid_params.cpp
)

target_include_directories(
    classes
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    pid_controller/include
    filter/include
)