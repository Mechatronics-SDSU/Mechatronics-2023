cmake_minimum_required(VERSION 3.12)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(EMBEDDED_NODE_RP2040 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(
                EMBEDDED_NODE_RP2040
                main.c
                )

# Link Libraries
target_link_libraries(
                        EMBEDDED_NODE_RP2040
                        pico_stdlib
                        )

pico_enable_stdio_usb(EMBEDDED_NODE_RP2040 1)
pico_enable_stdio_uart(EMBEDDED_NODE_RP2040 0)

# Create EXEC binaries like uf2
pico_add_extra_outputs(EMBEDDED_NODE_RP2040)
