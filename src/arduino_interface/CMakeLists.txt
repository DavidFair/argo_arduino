set(SRC_FILES
    arduino_hardware.cpp 
    pinTimingData.cpp
)

set(HDR_FILES
    arduino_hardware.hpp
    arduino_interface.hpp
    pinTimingData.hpp
)

include_directories("${PROJECT_SOURCE_DIR}/src/lib_wrappers")

if (NOT UNIT_TESTING)
    generate_arduino_library(
    arduino_hardware
    SRCS ${SRC_FILES}
    HDRS ${HDR_FILES}
    BOARD ${BOARD_TYPE}
    )
else()
    # We only need the pin timing data for unit testing as the 
    # implementation is mocked by the testing framework
    add_library(
    arduino_hardware STATIC
    pinTimingData.cpp
    pinTimingData.hpp
    )
endif()