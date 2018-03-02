set(SRC_FILES
    arduino_hardware.cpp 
    pinTimingData.cpp
)

set(HDR_FILES
    arduino_hardware.hpp
    arduino_interface.hpp
    arduino_lib_wrapper.hpp
    pinTimingData.hpp
)

if (NOT UNIT_TESTING)
    generate_arduino_library(
    arduino_hardware
    SRCS ${SRC_FILES}
    HDRS ${HDR_FILES}
    BOARD ${BOARD_TYPE}
    )
else()
    add_library(
    arduino_hardware STATIC
    arduino_lib_wrapper.hpp
    arduino_lib_wrapper.cpp
    pinTimingData.cpp
    pinTimingData.hpp
    )
endif()