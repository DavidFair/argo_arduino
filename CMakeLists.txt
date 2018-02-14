# Adapted from 'queezythegreat' example at the following URL
# https://github.com/queezythegreat/arduino-cmake/blob/master/CMakeLists.txt

# The following tools are required to be installed on the local system
# most are pulled in with the Arduino SDK

# gcc-avr binutils-avr avr-libc avrdude picocom

# The following targets are then used as appropriate
# argo_rc, argo_rc-serial, argo_rc-upload


set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake)

# Set default options for the ARGO
set(BOARD_TYPE mega2560 CACHE STRING "The target Arduino board")
set(PORT_PATH /dev/ttyUSB0 CACHE STRING "The path to the Arduino TTY device")
set(BAUD_RATE 115200 CACHE STRING "The baud rate of the serial device")

cmake_minimum_required(VERSION 2.8)

# Setup project details
project(ARGO C CXX)

# Add external library files
add_subdirectory(lib)

# Add source files
add_subdirectory(src)

message("test")