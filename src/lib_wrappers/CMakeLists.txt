set(SRC_FILES
    arduino_lib_wrapper.cpp
    )

set(HDR_FILES
    arduino_lib_wrapper.hpp
    encoder_interface.hpp
    unique_ptr.hpp)

if (NOT UNIT_TESTING)
    generate_arduino_library(
    lib_wrappers
    SRCS ${SRC_FILES}
    HDRS ${HDR_FILES} encoder_impl.hpp
    BOARD ${BOARD_TYPE}
    )
else()
    add_library(
    lib_wrappers STATIC
    ${HDR_FILES}
    ${SRC_FILES}
    )
endif()