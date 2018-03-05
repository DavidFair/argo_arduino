# The following tools are required to be installed on the local system
# most are pulled in with the Arduino SDK
# gcc-avr binutils-avr avr-libc avrdude picocom

# The following targets are then used as appropriate
# argo_rc, argo_rc-serial, argo_rc-upload

# To run unit tests locally use
# -DUNIT_TESTING=ON

# Pull in our dependencies before starting
execute_process(COMMAND git submodule update --init
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wpedantic -Wextra")

set(UNIT_TESTING ${UNIT_TESTING} CACHE BOOL "Enables unit testing targets and builds on GCC" FORCE)

# If the user didn't specify a build type assume Release as we
# are targetting an embedded device normally
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Select the build type:" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()

# Setup toolchain
if(NOT UNIT_TESTING)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/ext/arduino-cmake/cmake/ArduinoToolchain.cmake)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch-enum")

    # Set default options for the ARGO
    set(BOARD_TYPE mega2560 CACHE STRING "The target Arduino board")
    set(PORT_PATH /dev/ttyACM0 CACHE STRING "The path to the Arduino TTY device")
    set(BAUD_RATE 115200 CACHE STRING "The baud rate of the serial device")
endif()

# Setup project details - this needs to be after setting the toolchain
project(ARGO C CXX)

# Add directories depending on whether we are targeting hardware
if(UNIT_TESTING)
    # We can only set Werror on Unit test target as Arduino libs have warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")

    # Use native compiler and add test directory
    enable_testing()
    add_definitions(-DUNIT_TESTING)
    add_subdirectory(test)
else()
    # Sized deallocation was introduced in C++-14 standard however the 
    # avr std lib does not understand it so force GCC to not use it
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-fno-sized-deallocation" COMPILER_NEEDS_NOSIZE_DEALLOC)
    if (COMPILER_NEEDS_NOSIZE_DEALLOC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-sized-deallocation")
    endif()

    # Add external library files
    add_subdirectory(ext)
endif()

# Source directories common to both targets
add_subdirectory(src)
