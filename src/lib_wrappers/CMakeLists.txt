set(SRC_FILES
    arduino_lib_wrapper.cpp
    )

set(HDR_FILES
    arduino_lib_wrapper.hpp
    cstring_wrapper.hpp
    Distance.hpp
    move.hpp
    pair.hpp
    Speed.hpp
    Time.hpp
    unique_ptr.hpp)

include_directories(
    "${PROJECT_SOURCE_DIR}/src/arduino_interface")

if (NOT UNIT_TESTING)
    generate_arduino_library(
    lib_wrappers
    SRCS ${SRC_FILES}
    HDRS ${HDR_FILES}
    BOARD ${BOARD_TYPE}
    )
else()
    include_directories("${PROJECT_SOURCE_DIR}/test")

    add_library(
    lib_wrappers STATIC
    ${HDR_FILES}
    ${SRC_FILES}
    )
endif()