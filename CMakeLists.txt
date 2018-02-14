# Adapted from 'queezythegreat' example at the following URL
# https://github.com/queezythegreat/arduino-cmake/blob/master/CMakeLists.txt

set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake)

# Set default options for the ARGO
set(BOARD_TYPE mega2560 CACHE STRING "The target Arduino board")

cmake_minimum_required(VERSION 2.8)

# Setup project details
project(ARGO C CXX)

# Add external library files
add_subdirectory(lib)

generate_arduino_library(argo_lib
    SRCS src/argo_rc_lib.cpp
    HDRS src/argo_rc_lib.hpp
BOARD ${BOARD_TYPE})

generate_arduino_firmware(argo_rc
    SRCS src/argo_rc.cpp
    LIBS argo_lib
BOARD ${BOARD_TYPE})

message("test")