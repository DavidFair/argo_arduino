set(SRC_FILES
    ArduinoGlobals.cpp
    Encoder.cpp
)

set(HDR_FILES
    ArduinoGlobals.hpp
    arduino_enums.hpp
    arduino_interface.hpp
    Encoder.hpp
)

include_directories("${PROJECT_SOURCE_DIR}/src/lib_wrappers")

if (NOT UNIT_TESTING)
    # Bring in hardware definition for factory
    include_directories("${PROJECT_SOURCE_DIR}/src/arduino_hardware")

    generate_arduino_library(
    arduino_interface
    LIBS arduino_hardware
    SRCS ${SRC_FILES}
    HDRS ${HDR_FILES}
    BOARD ${BOARD_TYPE}
    )
else()
    add_library(
    arduino_interface STATIC
    ${SRC_FILES} ${HDR_FILES}
    )
endif()