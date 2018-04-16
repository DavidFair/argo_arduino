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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wpedantic -Wextra" CACHE STRING "" FORCE)

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
    # Warn on missing cases in a switch - gmock prevents us doing this globally
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch-enum" CACHE STRING "" FORCE)

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
    add_compile_options(-Werror)
    set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address -fsanitize=undefined")
    set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_STATIC_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address -fsanitize=address")

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
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-sized-deallocation" CACHE STRING "" FORCE)
    endif()

    # LTO to reduce overhead from using static libs saves a few MB
    # This was taken from the Arduino IDE compilation flags
    set(LTO_COMPILER_FLAGS " -MMD -flto -fno-fat-lto-objects ")
    set(LINK_TIME_FLAGS " -flto -fuse-linker-plugin -Wl,--gc-sections ")
    CHECK_CXX_COMPILER_FLAG(${LTO_COMPILER_FLAGS} COMPILER_SUPPORTS_LTO)

    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.9.0")
        # LTO will not work with 4.8.x and below compilers
        set(COMPILER_SUPPORTS_LTO FALSE)
    endif()

    if (COMPILER_SUPPORTS_LTO)
        message("Using Link Time Optimisations")
        set(CMAKE_AR "avr-gcc-ar" CACHE STRING "" FORCE)
        set(CMAKE_RANLIB "avr-gcc-ranlib" CACHE STRING "" FORCE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LTO_COMPILER_FLAGS}" CACHE STRING "" FORCE)

        set(CMAKE_LINK_LIBRARY_FLAG "${CMAKE_LINK_LIBRARY_FLAG} ${LINK_TIME_FLAGS}" CACHE STRING "" FORCE)
        mark_as_advanced(CMAKE_LINK_LIBRARY_FLAG)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${LINK_TIME_FLAGS}" CACHE STRING "" FORCE)
    endif()
endif()

# Source directories common to both targets
add_subdirectory(src)
