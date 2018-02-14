generate_arduino_library(argo_lib
    SRCS argo_rc_lib.cpp
    HDRS argo_rc_lib.hpp
    BOARD ${BOARD_TYPE})

generate_arduino_firmware(argo_rc
    SRCS argo_rc.cpp
    LIBS argo_lib
    BOARD ${BOARD_TYPE}
    PORT ${PORT_PATH}
    # -l Prevents picocom from attempting to lock the port
    SERIAL picocom @SERIAL_PORT@ -b ${BAUD_RATE} -l
    )