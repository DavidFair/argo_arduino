set(ARGO_LIB_SRC 
    argo_rc_lib.cpp
    argo_encoder.cpp
    SerialComms.cpp)

set(ARGO_LIB_HDR 
    argo_rc_lib.hpp
    argo_encoder.cpp
    SerialComms.cpp)

include_directories(
    "${PROJECT_SOURCE_DIR}/src/arduino_interface"
    "${PROJECT_SOURCE_DIR}/src/lib_wrappers"
)


# Define the static library
if (UNIT_TESTING)
    # Building on X86 target
    add_library(
        argo_lib STATIC
        ${ARGO_LIB_SRC} 
        ${ARGO_LIB_HDR}
        )

else()
    # Target is AVR microprocessor
    include_directories("${PROJECT_SOURCE_DIR}/src/arduino_hardware")

    set(REQUIRED_LIBS
        arduino_interface 
        arduino_hardware 
        lib_wrappers
    )

    generate_arduino_library(argo_lib
    LIBS ${REQUIRED_LIBS}
    SRCS ${ARGO_LIB_SRC}
    HDRS ${ARGO_LIB_HDR}
    BOARD ${BOARD_TYPE}
    )

    generate_arduino_firmware(argo_rc
    SRCS argo_rc.cpp
    LIBS ${REQUIRED_LIBS} argo_lib
    BOARD ${BOARD_TYPE}
    PORT ${PORT_PATH}
    # -l Prevents picocom from attempting to lock the port
    SERIAL picocom @SERIAL_PORT@ -b ${BAUD_RATE} -l
    )

endif()
