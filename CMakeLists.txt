# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(rocky-telemetry C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(rocky-telemetry
        src/rocky-telemetry.cpp
        src/state_machine.cpp
        src/button.cpp
        src/imu.cpp
        src/logger.cpp
        src/storage.cpp
        src/sd_card.cpp
        src/fat32.cpp
        src/sd_storage.cpp)

pico_set_program_name(rocky-telemetry "rocky-telemetry")
pico_set_program_version(rocky-telemetry "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(rocky-telemetry 0)
pico_enable_stdio_usb(rocky-telemetry 1)

# Add the standard library to the build
target_link_libraries(rocky-telemetry
        pico_stdlib
        pico_multicore
        hardware_dma
        hardware_i2c
        hardware_spi
        pico_util)

# Add the standard include files to the build
target_include_directories(rocky-telemetry PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_add_extra_outputs(rocky-telemetry)

