set(SRC_FILES
    arduino_lib_wrapper.cpp
    encoder_interface.cpp
    )

set(HDR_FILES
    arduino_lib_wrapper.hpp
    encoder_interface.hpp
    move.hpp
    pair.hpp
    unique_ptr.hpp)

include_directories(
    "${PROJECT_SOURCE_DIR}/src/arduino_interface")

if (NOT UNIT_TESTING)
    include_directories("${PROJECT_SOURCE_DIR}/ext/encoder")

    generate_arduino_library(
    lib_wrappers
    SRCS ${SRC_FILES}
    HDRS ${HDR_FILES} encoder_impl.hpp
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