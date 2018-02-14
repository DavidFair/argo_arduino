set(ARGO_LIB_SRC argo_rc_lib.cpp)
set(ARGO_LIB_HDR argo_rc_lib.hpp)


if (UNIT_TESTING)
    message(${PROJECT_SOURCE_DIR}/src/arduino_interface)
    include_directories(${PROJECT_SOURCE_DIR}/src/arduino_interface)

    ADD_LIBRARY(argo_lib STATIC
        ${ARGO_LIB_SRC} ${ARGO_LIB_HDR})
else()
    generate_arduino_library(argo_lib
    SRCS ${ARGO_LIB_SRC}
    HDRS ${ARGO_LIB_HDR}
    BOARD ${BOARD_TYPE})
endif()


if(NOT UNIT_TESTING)
generate_arduino_firmware(argo_rc
    SRCS argo_rc.cpp
    LIBS argo_lib
    BOARD ${BOARD_TYPE}
    PORT ${PORT_PATH}
    # -l Prevents picocom from attempting to lock the port
    SERIAL picocom @SERIAL_PORT@ -b ${BAUD_RATE} -l)
endif()