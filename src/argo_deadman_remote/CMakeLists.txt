# Include header libraries

link_directories(${PROJECT_SOURCE_DIR}/ext/rfm69)
link_directories(${PROJECT_SOURCE_DIR}/ext/spiflash)

# Specify target

generate_arduino_firmware(argo_deadman_remote
    SRCS argo_deadman_remote.cpp
    BOARD ${BOARD_TYPE}
    PORT ${PORT_PATH}
    SERIAL picocom @SERIAL_PORT@ -b ${BAUD_RATE} -l
)