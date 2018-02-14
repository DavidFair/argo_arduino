# Adapted from 'queezythegreat' example at the following URL
# https://github.com/queezythegreat/arduino-cmake/blob/master/CMakeLists.txt

# The following tools are required to be installed on the local system
# most are pulled in with the Arduino SDK
# gcc-avr binutils-avr avr-libc avrdude picocom

# The following targets are then used as appropriate
# argo_rc, argo_rc-serial, argo_rc-upload

# To run unit tests locally use
# -DUNIT_TESTING=ON

execute_process(COMMAND git submodule update --init
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)

# Setup project details
project(ARGO C CXX)
                

if(UNIT_TESTING)
    # Use native compiler and add test directory
    add_subdirectory(test)
else()
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/ext/arduino-cmake/cmake/ArduinoToolchain.cmake)

    # Set default options for the ARGO
    set(BOARD_TYPE mega2560 CACHE STRING "The target Arduino board")
    set(PORT_PATH /dev/ttyACM0 CACHE STRING "The path to the Arduino TTY device")
    set(BAUD_RATE 115200 CACHE STRING "The baud rate of the serial device")

    # Add external library files
    add_subdirectory(ext)
endif()

# Add source files
add_subdirectory(src)

