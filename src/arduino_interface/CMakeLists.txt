
if (NOT UNIT_TESTING)
    generate_arduino_library(
    arduino_hardware
    SRCS arduino_hardware.cpp
    HDRS arduino_hardware.hpp arduino_interface.hpp
    BOARD ${BOARD_TYPE}
    )
endif()